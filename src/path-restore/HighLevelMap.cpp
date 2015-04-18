//auto-include {{{
#include <boost/range/algorithm.hpp>
#include <mysql++.h>
#include    "HighLevelMap.h"
//}}}
using namespace boost::posix_time;
using namespace std;
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
    return boost::lower_bound(_pimpl->partten, timeOfDay, 
            [](Impl::P const& p, time_duration const& t){ return p.begin < t; })->cost;
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

VirtualEdgeWeight HighLevelMap::weightOrTmpOne(int index)
{
    if ( index > 0 && index < _roadWeight.size() ) return _roadWeight[index];
    return _tmpRoad.at(index - roadSize()).weight;
}

