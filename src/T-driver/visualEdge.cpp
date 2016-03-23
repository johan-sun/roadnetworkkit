//auto-include {{{
#include <string>
#include <vector>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/date_time.hpp>
#include <boost/range/algorithm.hpp>
#include <mysql++.h>
#include <boost/timer.hpp>
#include "roadmap.h"
#include "time_estimate.h"
#include "ivmm/ivmm.h"
#include "sutil/boost/date_time/date_time_format.hpp"
//}}}
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using b::posix_time::ptime;

struct FilterInfo
{
    double length;
    double speed;
    int cost;
};


typedef vector<TimedCrossIndex>::iterator TimedPathIter;
TimedPathIter findNextHotCross(TimedPathIter begin, TimedPathIter end, unordered_set<int> const& hotCross, int& c, ptime& midtime)
{
    struct CrossIn
    {
        unordered_set<int> const& hotCross;
        CrossIn(unordered_set<int> const& h):hotCross(h){}
        bool operator()(TimedCrossIndex const& idx)const{
            return hotCross.count(idx.first) != 0;
        }
    };
    struct CrossNotEq
    {
        int c;
        CrossNotEq(int n):c(n){}
        bool operator()(TimedCrossIndex const& idx)const{
            return idx.first != c;
        }
    };

    TimedPathIter it = find_if(begin ,end, CrossIn(hotCross));
    if ( it == end ) return it;
    TimedPathIter itEnd = find_if(it, end, CrossNotEq(it->first));
    c = it->first;
    midtime = it->second;
    if ( distance(it, itEnd) > 1 ){
        midtime = it->second + (prev(itEnd)->second - it->second) / 2;
    }
    return itEnd;
}
namespace std
{
template<>
struct hash<pair<int, int> >{
    size_t operator()(pair<int, int> const& p)const{
        hash<int> hasher;
        return hasher(p.first) ^ (hasher(p.second) << 1);
    }
};
}
typedef unordered_map<pair<int, int>, vector<pair<ptime, ptime> > > EdgeMap;
int main(int argc, char *argv[])
{
    string road;
    string crossInfo;
    int k;
    int f;
    fs::path output;
    string type;
    double d;
    string user;
    string host;
    string passwd;
    string comment;
    string where;
    po::options_description desc(argv[0]);
    desc.add_options()
        (",r", po::value(&road)->required(), "road shp")
        (",c", po::value(&crossInfo)->required(), "cross info dbf")
        (",k", po::value(&k)->default_value(1000), "top k")
        (",f", po::value(&f)->required(), "filter")
        (",d", po::value(&d)->default_value(0.95), "remain")
        ("output,o",po::value(&output)->required(), "output")
        (",u", po::value(&user)->required()->default_value("root"), "mysql user")
        (",h", po::value(&host)->required()->default_value("localhost"), "mysql host")
        (",p", po::value(&passwd)->required(), "mysql passwd")
        ("type", po::value(&type)->default_value("CountXEnt"), "rank-type")
        ("where", po::value(&where)->default_value("1"), "mysql `traj_metadata` WHERE cause to filter traj")
        ("comment,m", po::value(&comment)->default_value(""),"comment")
        ("help,h","show help");
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if ( vm.count("help") ){
            cout << desc << endl;
            return 0;
        }
        vm.notify();
    }catch(std::exception const& e){
        if ( vm.count("help") ){
            cout << desc << endl;
            return 0;
        }
        cerr << e.what() << endl;
        return 1;
    }

    if ( output.has_parent_path() )
    {
        fs::path ppath = output.parent_path();
        if(!fs::exists(ppath))
        {
            boost::system::error_code err;
            fs::create_directories(ppath, err);
            if(err)
            {
                cerr << err.message() << endl;
                return 1;
            }
        }
    }

    DBFHandle cdbf = DBFOpen(crossInfo.c_str(), "r");
    unordered_set<int> hotCross;
    unordered_map<pair<int, int>, FilterInfo> timeFilterMap;
    EdgeMap edgeMap;
    if ( !cdbf)
    {
        cerr << "can not open dbf:" << crossInfo << endl;
        return 1;
    }
    RoadMap map;
    if ( ! map.load(road) ){
        cerr << "can not load shp " << road << endl;
        return 1;
    }

    mysqlpp::Connection con;
    try
    {
        con.connect("path_restore", host.c_str(), user.c_str(), passwd.c_str());
    }catch(mysqlpp::ConnectionFailed const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }
    mysqlpp::Transaction tan(con);

    int index;
    int topk = 0;
    topk = min(DBFGetRecordCount(cdbf), k);
    for(int i = 0; i < topk; ++i)
    {
        index = DBFReadIntegerAttribute(cdbf, i, 1);
        hotCross.insert(index);
    }
    DBFClose(cdbf);
    cout << "load hot cross" << endl;
    try
    {
        mysqlpp::Query q = con.query("SELECT `id` FROM `traj_metadata` WHERE %0");
        mysqlpp::Query dq = con.query("SELECT `cross_id`, `time` FROM `traj_data` WHERE `metadata_id` = %0");
        q.parse();
        dq.parse();
        list<mysqlpp::Row> metaIDs;
        q.storein(metaIDs, where);
        int cur = 0;
        int all = metaIDs.size();
        boost::timer t;
        while( !metaIDs.empty() )
        {
            ++cur;
            if(t.elapsed() > 0.1)
            {
                cout << cur << "/" << all << "\r" << flush;
                t.restart();
            }
            int metaID = metaIDs.front()[0];
            metaIDs.pop_front();
            vector<TimedCrossIndex> timedPath;
            mysqlpp::StoreQueryResult sr = dq.store(metaID);
            for(auto& r : sr)
            {
                int idx = r[0];
                string s;
                r[1].to_string(s);
                timedPath.push_back({idx, boost::posix_time::time_from_string(s)});
            }
            typedef vector<TimedCrossIndex>::iterator Iter;
            int cross;
            ptime midTime;
            TimedPathIter it = findNextHotCross(timedPath.begin(), timedPath.end(), hotCross, cross, midTime);
            //cout << "loading: " << metaID << endl;
            while(true)
            {
                int cross2;ptime midTime2;
                TimedPathIter it2 = findNextHotCross(it, timedPath.end(), hotCross, cross2, midTime2);
                if ( it2 == timedPath.end() )
                    break;
                it = it2;
                if ( cross != cross2 )
                {
                    //record  cross->cross2
                    pair<int, int> edge{cross, cross2};
                    FilterInfo filter;
                    if ( timeFilterMap.count(edge) ){
                        filter  = timeFilterMap[edge];
                    }else{
                        Path path = map.shortest_path_Astar(cross, cross2);
                        filter.length = path.total_length();
                        filter.speed = min(weight_speed(path, map) * 3, 33.3);
                        //filter.speed = 33.3;
                        filter.cost = static_cast<int>(filter.length / filter.speed) * 1000;
                        //cout << "from:"<< cross << " to:" << cross2 << 
                        //    " length:" << filter.length << 
                        //    " fspeed:" << filter.speed << 
                        //    " must > " << filter.cost << 
                        //    " ms" << endl;
                        timeFilterMap[edge] = filter;
                    }

                    int costMill = (midTime2 - midTime).total_milliseconds() ;
                    if ( costMill > filter.cost ){
                        edgeMap[{cross, cross2}].push_back({midTime, midTime2});
                    }
                    else{
                        //cout << "cost too less from:"<<cross << " to:" << cross2 << 
                        //    " length:" << filter.length << 
                        //    " fspeed:" << filter.speed << 
                        //    " use:" << costMill <<
                        //    " ms" << " < " << filter.cost << 
                        //    " ms"<<endl;
                    }
                    cross = cross2;
                    midTime = midTime2;
                }
            }
        }
    }
    catch(exception const& e)
    {
        cerr << "error in proc data:" << e.what() << endl;
        return 1;
    }
    mysqlpp::Query insertTypeQ = con.query("INSERT INTO `virtual_edge_type` (`rank-type`, `top`, `count-filter`, `data-use`, `comments`) VALUES(%0q, %1, %2, %3, %4Q)");
    int typeID;
    try
    {
        insertTypeQ.parse();
        typeID = insertTypeQ.execute(type, topk, f, d, comment).insert_id();
    }catch(exception const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }

    SHPHandle shp = SHPCreate(output.c_str(), SHPT_ARC);
    DBFHandle dbf = DBFCreate(output.c_str());
    DBFAddField(dbf, "START", FTInteger, 10, 0);
    DBFAddField(dbf, "END", FTInteger, 10, 0);
    DBFAddField(dbf, "DBID", FTInteger, 10, 0);
    DBFAddField(dbf, "COUNT", FTInteger, 10, 0);
    cout << "writing..." << endl;
    mysqlpp::Query insertVirtualEdgeMetaQ = con.query("INSERT INTO `virtual_edge_metadata`(`start`, `end`,`type`) VALUES(%0, %1, %2)");
    mysqlpp::Query insertVirtualEdgeDataQ = con.query("INSERT INTO `virtual_edge_data`(`metadata_id`, `enter`, `leave`, `cost`) VALUES(%0, %1Q, %2Q, %3)");
    try
    {
        insertVirtualEdgeDataQ.parse();
        insertVirtualEdgeMetaQ.parse();
        int all = edgeMap.size();
        int cur = 0;
        boost::timer t;
        for(auto& each : edgeMap){
            ++cur;
            if ( t.elapsed() > 0.1)
            {
                cout << cur << "/" << all << "\r" << flush;
                t.restart();
            }
            pair<int, int> const& edge = each.first;
            vector<pair<ptime, ptime> > & timePairs = each.second;
            boost::sort(timePairs, [](pair<ptime, ptime> const& a, pair<ptime, ptime> const& b){
                    return a.second - a.first < b.second - b.first;
            });
            timePairs.resize(timePairs.size()*d);
            if ( timePairs.size() > f ){
                int metaID = insertVirtualEdgeMetaQ.execute(edge.first, edge.second, typeID).insert_id();
                double x[2] = {map.cross(edge.first).geometry.x(), map.cross(edge.second).geometry.x()};
                double y[2] = {map.cross(edge.first).geometry.y(), map.cross(edge.second).geometry.y()};
                SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
                const int insertAtEnd = -1;
                int id = SHPWriteObject(shp, insertAtEnd, line);
                DBFWriteIntegerAttribute(dbf, id, 0, edge.first);
                DBFWriteIntegerAttribute(dbf, id, 1, edge.second);
                DBFWriteIntegerAttribute(dbf, id, 2, metaID);
                DBFWriteIntegerAttribute(dbf, id, 3, timePairs.size());
                SHPDestroyObject(line);
                for(auto& timePair : timePairs)
                {
                    insertVirtualEdgeDataQ.execute(metaID, 
                            to_format_string(timePair.first, "%Y-%m-%d %H:%M:%S%F"), 
                            to_format_string(timePair.second, "%Y-%m-%d %H:%M:%S%F"),
                            (timePair.second - timePair.first).total_milliseconds()
                    );
                }
            }
        }
    }catch(exception const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }
    SHPClose(shp);
    DBFClose(dbf);
    cout << "commting..." << endl;
    tan.commit();
    cout << "typeID:" << typeID << endl;
    return 0;
}
