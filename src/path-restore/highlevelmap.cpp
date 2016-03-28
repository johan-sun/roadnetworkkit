//auto-include {{{
#include <boost/range/algorithm.hpp>
#include <mysql++.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <list>
#include <boost/algorithm/cxx11/any_of.hpp>
#include    "highlevelmap.h"
//}}}
using namespace boost::posix_time; using namespace std;

struct VirtualEdgeWeight::Impl
{
    struct P{
        time_duration begin;
        time_duration end;
        time_duration cost;
    };
    vector<P> partten;
};

VirtualEdgeWeight::VirtualEdgeWeight(double ms):_pimpl(new Impl)
{
    Impl::P p;
    p.begin = seconds(0);
    p.end = hours(24);
    p.cost = milliseconds(ms);
    _pimpl->partten.push_back(p);
}

VirtualEdgeWeight::VirtualEdgeWeight(string const& str):_pimpl(new Impl)
{
    Impl::P p;
    istringstream in(str);
    double sec;
    while(in>>p.begin && in.ignore() 
        && in >> p.end && in.ignore() 
        && in >> sec && in.ignore())
    {
        p.cost = milliseconds(sec * 1000);
        _pimpl->partten.push_back(p);
    }
    _pimpl->partten.front().begin = seconds(0);
    _pimpl->partten.back().end = hours(24);

}

time_duration VirtualEdgeWeight::operator()(time_duration const& timeOfDay)const
{
    if ( timeOfDay > hours(24) ) throw invalid_argument("not valided time duration");
    auto it = boost::lower_bound(_pimpl->partten, timeOfDay, 
            [](Impl::P const& p, time_duration const& t){ return p.begin < t; });
    if ( it == _pimpl->partten.end() )
        --it;
    return it->cost;
    //return _pimpl->partten.begin()->cost;
}


void VirtualEdgeWeight::displayTo(ostream & o)const
{
    for(auto& p : _pimpl->partten)
    {
        o << "["<<p.begin << "," << p.end << ") " << p.cost << endl;
    }
}

struct StartEndCrossIDChecker
{
    const static int START_CROSS_ID_FIELD = 0;
    const static int END_CROSS_ID_FIELD = 1;
    std::unordered_map<int, int> IDIndexMap;
    bool cross_is_new(CrossPosInRoad pos, Point const&/*p*/, RoadSegment const&/*r*/, DBFHandle handle, int roadIndex){
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        return IDIndexMap.count(ID) == 0;
    }

    void store_index(CrossPosInRoad pos, Point const&/*p*/, int crossIndex, RoadSegment const&/*r*/, DBFHandle handle, int roadIndex)
    {
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        IDIndexMap[ID] = crossIndex;
    }

    int get_index(CrossPosInRoad pos, Point const&/*p*/, RoadSegment const&/*r*/, DBFHandle handle, int roadIndex)
    {
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        return IDIndexMap.at(ID);
    }
};

struct CrossIDEdgeMetadataIDPicker{
    const static int COUNT_FIELD = 3;
    const static int DBID_FIELD = 2;
    const static int START_CROSS_ID_FIELD = 0;
    const static int END_CROSS_ID_FIELD = 1;
    Direction pick_roadsegment(pt::ptree& p, DBFHandle handle, int roadIndex)
    {
        int DBID = DBFReadIntegerAttribute(handle, roadIndex, DBID_FIELD);
        int count = DBFReadIntegerAttribute(handle, roadIndex, COUNT_FIELD);
        p.put("METADATA-ID", DBID);
        p.put("COUNT", count);
        return Forward;
    }
    void pick_cross(CrossPosInRoad pos, pt::ptree& properties, DBFHandle handle, int roadIndex)
    {
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        properties.put("ID", ID);
    }
};

bool HighLevelMap::load(std::string const& shp, char const* server, char const* user, char const* pass)
{
    if ( !Map::load(shp, CrossIDEdgeMetadataIDPicker(), StartEndCrossIDChecker()))
    {
        cerr << "can not load " << shp << endl;
        return false;
    }
    build_graph();
    map_cross_property<int>("ID");
    map_roadsegment_property<int>("METADATA-ID");
    try
    {
        road_weight_.resize(road_size());
        mysqlpp::Connection con("path_restore", server, user, pass);
        mysqlpp::Query q = con.query("SELECT `pattern` FROM `virtual_edge_metadata` WHERE `id` = %0");
        q.parse();
        visit_roadsegment([&](RoadSegment const& r){
            string str;
            q.store(r.properties.get<int>("METADATA-ID")).at(0).at(0).to_string(str);
            road_weight_[r.index] = VirtualEdgeWeight(str);
        });
    }catch(mysqlpp::Exception const& e)
    {
        cerr << e.what() << endl;
        return false;
    }
    return true;
}

Cross const& HighLevelMap::add_tmp_cross(int ID, double x, double y)
{ 
    if ( contain_cross("ID", ID) || tmp_cross_id_index_map_.count(ID) )
        throw invalid_argument("cross already exsits");
    Cross cross;
    cross.geometry.x(x);cross.geometry.y(y);
    cross.index = cross_size() + tmp_cross_.size();
    cross.properties.put("ID", ID);
    tmp_cross_id_index_map_[ID] = cross.index;
    tmp_cross_.push_back(std::move(cross));
    return tmp_cross_.back();
}

RoadSegment const& HighLevelMap::add_tmp_virtual_edge(int crossStartIndex, int crossEndIndex, VirtualEdgeWeight const& weight)
{
    if ( crossStartIndex == crossEndIndex ) throw invalid_argument("cross index equal");
    Cross const& c1 = cross_or_tmp_one(crossStartIndex);
    Cross const& c2 = cross_or_tmp_one(crossEndIndex);
    TmpRoad tr;
    tr.road.geometry.push_back(c1.geometry);tr.road.geometry.push_back(c2.geometry);
    tr.road.index = road_size() + tmp_road_.size();
    tr.weight = weight;
    tmp_road_.push_back(tr);
    tmp_adj_list_[crossStartIndex].push_back({tr.road.index, crossEndIndex});
    return tmp_road_.back().road;
}

Cross const& HighLevelMap::cross_or_tmp_one(int index)const
{
    if ( contain_cross(index) ) return cross(index);
    return tmp_cross_.at(index - cross_size());
}

HighLevelMap::OutIter HighLevelMap::out_road_include_tmp(int crossIndex)const
{
    return OutIter(*this, crossIndex);
}

VirtualEdgeWeight HighLevelMap::weight_or_tmp_one(int index)const
{
    if ( index >= 0 && index < road_weight_.size() ) return road_weight_[index];
    return tmp_road_.at(index - road_size()).weight;
}


struct DijkstraNode
{
    DijkstraNode(int cross, time_duration const& t, DijkstraNode * pre = nullptr)
        :pre(pre), time(t), cur(cross){}
    DijkstraNode* pre;
    time_duration time;
    int cur; 
};

struct TimeGreater
{
    bool operator()(DijkstraNode const* a, DijkstraNode const* b)const
    {
        return a->time > b->time;
    }
};

vector<HighLevelMap::TimedCross> HighLevelMap::shortest_path(int indexA, time_duration const& time, 
        int indexB,
        unordered_set<int> const& deleteEdges, unordered_set<int> const& deleteCross,
        function<bool(Cross const& c)> const& crossIgnore,
        function<bool(time_duration const&)> const& timeIgnore
        )const
{
    //cout << cross(indexA).properties.get<int>("ID") << 
    //    " @ " << time << " to " << cross(indexB).properties.get<int>("ID") << endl;
    using namespace boost::heap;
    typedef fibonacci_heap<DijkstraNode *, boost::heap::allocator<boost::pool_allocator<DijkstraNode *> >,
        compare<TimeGreater> > Queue;
    Queue queue;
    boost::object_pool<DijkstraNode> pool;
    vector<TimedCross> path;
    unordered_map<int, Queue::handle_type> open;
    open[indexA] = queue.push(pool.construct(indexA, time));
    unordered_set<int> close;
    while( !queue.empty() )
    {
        DijkstraNode * top = queue.top();
        queue.pop();
        close.insert(top->cur);
        //cout << "top:" << cross(top->cur).properties.get<int>("ID") <<  " @ " << top->time << endl;
        if( top->cur == indexB )
        {
            while( top )
            {
                TimedCross tc;
                tc.cross_index = top->cur;
                tc.time = top->time;
                path.push_back(tc);
                top = top->pre;
            }
            boost::reverse(path);
            return path;
        }
        for(auto& each : out_road_range_include_tmp(top->cur))
        {
            //cout << "in for" << endl;
            time_duration t = top->time + weight_or_tmp_one(each.road_index());
            //cout << t << endl;
            if (close.count(each.cross_index()) || deleteCross.count(each.cross_index()) || deleteEdges.count(each.road_index())
                    || (crossIgnore && crossIgnore(cross_or_tmp_one(each.cross_index())))
                    || (timeIgnore && timeIgnore(t))) continue;
            if ( open.count(each.cross_index()) )
            {
                Queue::handle_type h = open[each.cross_index()];
                DijkstraNode* node = *h;
                if ( t < node->time )
                {
                    node->time = t;
                    node->pre =top;
                    queue.update_lazy(h);
                }
            }else
            {
                open[each.cross_index()] = queue.push(pool.construct(each.cross_index(), t, top));
            }
        }
    }
    return path;
}


struct TimedCrossVecGerater
{
    bool operator()(shared_ptr<vector<HighLevelMap::TimedCross> > const& a, shared_ptr<vector<HighLevelMap::TimedCross> > const& b)const
    {
        return a->back().time  > b->back().time;
    }
};


struct ShortestPath
{
    shared_ptr<vector<HighLevelMap::TimedCross> > ppath;
    int devIndex;
    bool operator > (ShortestPath const& otr)const
    {
        return ppath->back().time > otr.ppath->back().time;
    }
};
struct KShortestPathGenerator::Impl
{
    Impl(HighLevelMap const& m, int a, time_duration const& t, int b, time_duration const& tm, Box const& r):map(m),index1(a),index2(b),time(t),terminate(tm), range(r){}
    HighLevelMap const& map;
    list<ShortestPath> topk;
    int index1;
    int index2;
    time_duration time;
    time_duration terminate;
    typedef boost::heap::fibonacci_heap<
        ShortestPath,
            boost::heap::compare<greater<ShortestPath> > > Heap;
    Heap heap;
    Box range;
};

KShortestPathGenerator::KShortestPathGenerator(HighLevelMap const& map, int index1, time_duration const& t, int index2, time_duration const& tm, Box const& r)
    :_pimpl(new Impl(map, index1, t, index2, tm, r))
{
    typedef vector<HighLevelMap::TimedCross> TimedCrossVec;
    ShortestPath path;
    TimedCrossVec vec = map.shortest_path(_pimpl->index1, _pimpl->time, _pimpl->index2);
    if ( ! vec.empty())
    {
        path.ppath.reset(new TimedCrossVec(std::move(vec)));
        path.devIndex = 0;
        _pimpl->topk.push_back(path);
    }
}


int find_edge(HighLevelMap const& map, int indexA, int indexB)
{
    for(auto& p : map.out_road_range_include_tmp(indexA) )
    {
        if ( p.cross_index() == indexB )
            return p.road_index();
    }
    assert(1);
    return -1;
}

vector<HighLevelMap::TimedCross>const* KShortestPathGenerator::nextPath()
{
    typedef vector<HighLevelMap::TimedCross> TimedCrossVec;
    HighLevelMap const& map = _pimpl->map;
    auto& topk = _pimpl->topk;
    unordered_set<int> deleteEdges;
    unordered_set<int> deleteCross;
    auto& lastPath = topk.back();
    TimedCrossVec root;
    if (lastPath.ppath)
    {
        time_duration lastTime = lastPath.ppath->back().time;
        copy(lastPath.ppath->begin(), lastPath.ppath->begin() + lastPath.devIndex, back_inserter(root));
        for(int i = 0; i < lastPath.devIndex; ++i)
            deleteCross.insert(lastPath.ppath->at(i).cross_index);
        //K
        //for(size_t i  = 0; i + 1u < lastPath.size(); ++i)
        for(size_t i = lastPath.devIndex;i + 1 < lastPath.ppath->size(); ++i )
        {
            unordered_set<int> ignoreCross;
            for(int j = i + 1; j < min<int>(i+2, lastPath.ppath->size() -1); ++j)
            {
                ignoreCross.insert(lastPath.ppath->at(j).cross_index);
            }
            root.push_back(lastPath.ppath->at(i));
            int curIdx = lastPath.ppath->at(i).cross_index;
            if ( i == lastPath.devIndex )
            {
                for(auto& path : topk)
                {
                    if ( boost::starts_with(*path.ppath, root) )
                    {
                        deleteEdges.insert(find_edge(map, curIdx, path.ppath->at(i+1).cross_index));
                    }
                }
            }else
            {
                deleteEdges.insert(find_edge(map,curIdx, lastPath.ppath->at(i+1).cross_index));
            }
            
            auto isNotInDeleteEdge = [&deleteEdges](RoadIndexCrossIndexPair const& pair){
                return !deleteEdges.count(pair.road_index());
            };
            if ( boost::algorithm::any_of(map.out_road_range_include_tmp(curIdx), isNotInDeleteEdge))
            {
                auto super = map.shortest_path(curIdx, lastPath.ppath->at(i).time, _pimpl->index2, 
                        deleteEdges, deleteCross, [&ignoreCross, this](Cross const& c){
                            //return ignoreCross.count(c.index) || !bg::within(c.geometry, this->_pimpl->range);
                                return false;
                            }, [this, &lastTime](time_duration const& t){
                            return t > this->_pimpl->terminate;
                            //    return false;
                            });
                if (! super.empty() && super.back().time > lastTime)
                {
                    TimedCrossVec newPath(begin(root), prev(end(root)));
                    boost::push_back(newPath, super);
                    ShortestPath sp;
                    sp.ppath.reset(new TimedCrossVec(std::move(newPath)));
                    sp.devIndex = i;
                    _pimpl->heap.push(sp);
                }
            }
            deleteCross.insert(lastPath.ppath->at(i).cross_index);
        }
        ShortestPath sp;
        if ( !_pimpl->heap.empty() )
        {
            sp = _pimpl->heap.top();
            _pimpl->heap.pop();
        }
        topk.push_back(sp);
    }
    return lastPath.ppath.get();
}
