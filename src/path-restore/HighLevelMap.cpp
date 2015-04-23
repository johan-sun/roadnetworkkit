//auto-include {{{
#include <boost/range/algorithm.hpp>
#include <mysql++.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <list>
#include <boost/algorithm/cxx11/any_of.hpp>
#include    "HighLevelMap.h"
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
    bool crossIsNew(CrossPosInRoad pos, Point const&/*p*/, RoadSegment const&/*r*/, DBFHandle handle, int roadIndex){
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        return IDIndexMap.count(ID) == 0;
    }

    void storeIndex(CrossPosInRoad pos, Point const&/*p*/, int crossIndex, RoadSegment const&/*r*/, DBFHandle handle, int roadIndex)
    {
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        IDIndexMap[ID] = crossIndex;
    }

    int getIndex(CrossPosInRoad pos, Point const&/*p*/, RoadSegment const&/*r*/, DBFHandle handle, int roadIndex)
    {
        int field = pos == Front ? START_CROSS_ID_FIELD : END_CROSS_ID_FIELD;
        int ID = DBFReadIntegerAttribute(handle, roadIndex, field);
        return IDIndexMap.at(ID);
    }
};

struct CrossIDEdgeMetadataIDPicker{
    const static int DBID_FIELD = 2;
    const static int START_CROSS_ID_FIELD = 0;
    const static int END_CROSS_ID_FIELD = 1;
    Direction pickRoadsegment(pt::ptree& p, DBFHandle handle, int roadIndex)
    {
        int DBID = DBFReadIntegerAttribute(handle, roadIndex, DBID_FIELD);
        p.put("METADATA-ID", DBID);
        return Forward;
    }
    void pickCross(CrossPosInRoad pos, pt::ptree& properties, DBFHandle handle, int roadIndex)
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
    buildGraph();
    mapCrossProperty<int>("ID");
    mapRoadsegmentProperty<int>("METADATA-ID");
    try
    {
        _roadWeight.resize(roadSize());
        mysqlpp::Connection con("path_restore", server, user, pass);
        mysqlpp::Query q = con.query("SELECT `pattern` FROM `virtual_edge_metadata` WHERE `id` = %0");
        q.parse();
        visitRoadSegment([&](RoadSegment const& r){
            string str;
            q.store(r.properties.get<int>("METADATA-ID")).at(0).at(0).to_string(str);
            _roadWeight[r.index] = VirtualEdgeWeight(str);
        });
    }catch(mysqlpp::Exception const& e)
    {
        cerr << e.what() << endl;
        return false;
    }
    return true;
}

Cross const& HighLevelMap::addTmpCross(int ID, double x, double y)
{ 
    if ( containCross("ID", ID) || _tmpCrossIDIndexMap.count(ID) )
        throw invalid_argument("cross already exsits");
    Cross cross;
    cross.geometry.x(x);cross.geometry.y(y);
    cross.index = crossSize() + _tmpCross.size();
    cross.properties.put("ID", ID);
    _tmpCrossIDIndexMap[ID] = cross.index;
    _tmpCross.push_back(std::move(cross));
    return _tmpCross.back();
}

RoadSegment const& HighLevelMap::addTmpVirtualEdge(int crossStartIndex, int crossEndIndex, VirtualEdgeWeight const& weight)
{
    if ( crossStartIndex == crossEndIndex ) throw invalid_argument("cross index equal");
    Cross const& c1 = crossOrTmpOne(crossStartIndex);
    Cross const& c2 = crossOrTmpOne(crossEndIndex);
    TmpRoad tr;
    tr.road.geometry.push_back(c1.geometry);tr.road.geometry.push_back(c2.geometry);
    tr.road.index = roadSize() + _tmpRoad.size();
    tr.weight = weight;
    _tmpRoad.push_back(tr);
    _tmpAdjList[crossStartIndex].push_back({tr.road.index, crossEndIndex});
    return _tmpRoad.back().road;
}

Cross const& HighLevelMap::crossOrTmpOne(int index)const
{
    if ( containCross(index) ) return cross(index);
    return _tmpCross.at(index - crossSize());
}

HighLevelMap::OutIter HighLevelMap::outRoadIncludeTmp(int crossIndex)const
{
    return OutIter(*this, crossIndex);
}

VirtualEdgeWeight HighLevelMap::weightOrTmpOne(int index)const
{
    if ( index >= 0 && index < _roadWeight.size() ) return _roadWeight[index];
    return _tmpRoad.at(index - roadSize()).weight;
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

vector<HighLevelMap::TimedCross> HighLevelMap::shortestPath(int indexA, time_duration const& time, 
        int indexB,
        unordered_set<int> const& deleteEdges
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
                tc.crossIndex = top->cur;
                tc.time = top->time;
                path.push_back(tc);
                top = top->pre;
            }
            boost::reverse(path);
            return path;
        }
        for(auto& each : outRoadRangeIncludeTmp(top->cur))
        {
            //cout << "in for" << endl;
            time_duration t = top->time + weightOrTmpOne(each.roadIndex());
            //cout << t << endl;
            if (close.count(each.crossIndex()) || deleteEdges.count(each.roadIndex())) continue;
            if ( open.count(each.crossIndex()) )
            {
                Queue::handle_type h = open[each.crossIndex()];
                DijkstraNode* node = *h;
                if ( t < node->time )
                {
                    node->time = t;
                    node->pre =top;
                    //cout << "road:" << each.roadIndex() << endl;
                    //cout << weightOrTmpOne(each.roadIndex());
                    //cout <<"update:" << cross(node->cur).properties.get<int>("ID") << " @ " << t << endl;
                    queue.update_lazy(h);
                }
            }else
            {
                //cout << "road:" << each.roadIndex() << endl;
                //cout << weightOrTmpOne(each.roadIndex());
                //cout <<"push:" << cross(each.crossIndex()).properties.get<int>("ID") << " @ " << t << endl;
                open[each.crossIndex()] = queue.push(pool.construct(each.crossIndex(), t, top));
            }
        }
    }
    return path;
}


struct KShortestPathGenerator::Impl
{
    Impl(HighLevelMap const& m, int a, time_duration const& t, int b):map(m),index1(a),index2(b),time(t){}
    HighLevelMap const& map;
    list<vector<HighLevelMap::TimedCross> > topk;
    int index1;
    int index2;
    time_duration time;
};

KShortestPathGenerator::KShortestPathGenerator(HighLevelMap const& map, int index1, time_duration const& t, int index2)
    :_pimpl(new Impl(map, index1, t, index2))
{}


int findEdge(HighLevelMap const& map, int indexA, int indexB)
{
    for(auto& p : map.outRoadRangeIncludeTmp(indexA) )
    {
        if ( p.crossIndex() == indexB )
            return p.roadIndex();
    }
    assert(1);
}

vector<HighLevelMap::TimedCross>const* KShortestPathGenerator::nextPath()
{
    typedef vector<HighLevelMap::TimedCross> TimedCrossVec;
    HighLevelMap const& map = _pimpl->map;
    auto& topk = _pimpl->topk;
    if ( _pimpl->topk.empty() )
    {
        topk.push_back(map.shortestPath(_pimpl->index1, _pimpl->time, _pimpl->index2));
    }else
    {
        list<TimedCrossVec> currentRoundPath;
        unordered_set<int> deleteEdges;
        auto& lastPath = topk.back();
        TimedCrossVec root;
        //K
        for(size_t i  = 0; i + 1u < lastPath.size(); ++i)
        {
            root.push_back(lastPath[i]);
            int curIdx = lastPath[i].crossIndex;
            for(auto& path : topk)
            {
                if ( boost::starts_with(path, root) )
                {
                    deleteEdges.insert(findEdge(map, curIdx, path[i+1].crossIndex));
                }
            }

            auto isNotInDeleteEdge = [&deleteEdges](RoadIndexCrossIndexPair const& pair){
                return !deleteEdges.count(pair.roadIndex());
            };
            if ( boost::algorithm::any_of(map.outRoadRangeIncludeTmp(curIdx), isNotInDeleteEdge))
            {
                auto super = map.shortestPath(curIdx, lastPath[i].time, _pimpl->index2, deleteEdges);
                if (! super.empty() )
                {
                    TimedCrossVec newPath(begin(root), prev(end(root)));
                    boost::push_back(newPath, super);
                    currentRoundPath.push_back(std::move(newPath));
                }
            }
        }
        if ( currentRoundPath.empty() )
            return nullptr;
        auto it = boost::min_element(currentRoundPath,[](TimedCrossVec const& a, TimedCrossVec const& b){
                 return a.back().time < b.back().time;
                });
        topk.push_back(std::move(*it));
    }
    return &topk.back();
}
