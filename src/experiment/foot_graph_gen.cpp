#include <boost/program_options.hpp>
#include <mysql++.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/timer.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/filesystem/fstream.hpp>
#include "path-restore/highlevelmap.h"
#include "roadmap.h"
#include "util.h"
#include "sutil/boost/date_time/date_time_format.hpp"
#include "time_estimate.h"
#include "experiment/com.h"
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace boost::posix_time;

struct RestoreInfo
{
    int k;
    vector<TimedCrossInDay> restore;
    vector<TimedCrossInDay> prj;
    time_duration cost;
    int turn;
    int turn_hot;
    double lcs_rate;
    double lcs_rate_hot;
};

int LeftCount = 2;
int RightCount = 1;
int StraightCount = 0;

int turns(RoadMap const& map, vector<TimedCrossInDay> const& traj)
{
    int cnt = 0;
    for(size_t i = 1; i+1 < traj.size() ; ++i)
    {
        Point a = map.cross(traj[i].cross_id).geometry;
        Point b = map.cross(traj[i+1].cross_id).geometry;
        bg::subtract_point(b, a);
        bg::subtract_point(a, map.cross(traj[i-1].cross_id).geometry);
        switch( direction(a, b) )
        {
        case 1://right
            //cnt += 1;break;
            //cnt += 2;break;
            cnt += RightCount;
            break;
        case -1://left
            //cnt += 3;break;
            //cnt += 1;break;
            cnt += LeftCount;
            break;
        default:
            cnt += StraightCount;
            break;
        }
    }
    return cnt;
}
vector<TimedCrossInDay> project(HighLevelMap const& map, vector<TimedCrossInDay> const& traj)
{
    vector<TimedCrossInDay> prj;
    auto is_hot_cross = [&](TimedCrossInDay const& tc){ return map.contain_cross("ID", tc.cross_id); };
    copy_if(traj.begin(), traj.end(),  back_inserter(prj), is_hot_cross);
    return prj;
}
vector<TimedCrossInDay> transform(HighLevelMap const& map, vector<HighLevelMap::TimedCross> const& traj)
{
    vector<TimedCrossInDay> traj_in_day;
    for(auto& p : traj)
    {
        int id = map.cross(p.cross_index).properties.get<int>("ID");
        traj_in_day.push_back({id, p.time});
    }
    return traj_in_day;
}
void draw_box(string const& prefix, Box const box)
{
    SHPHandle hshp = SHPCreate(prefix.c_str(), SHPT_POLYGON);
    double x[4] =  {box.min_corner().x(), box.max_corner().x(), box.max_corner().x(), box.min_corner().x() };
    double y[4] = {box.min_corner().y(), box.min_corner().y(), box.max_corner().y(), box.max_corner().y() };
    SHPObject* sbox = SHPCreateSimpleObject(SHPT_POLYGON, 4, x, y, nullptr);
    SHPWriteObject(hshp, -1, sbox);
    SHPDestroyObject(sbox);
    SHPClose(hshp);
}
void draw_path(RoadMap const& map, string const& prefix, vector<TimedCrossInDay> const& traj)
{
    fs::path prefix_path(prefix);
    if(prefix_path.has_parent_path())
    {
        fs::path ppath = prefix_path.parent_path();
        if(!fs::exists(ppath))
        {
            fs::create_directories(ppath);
        }
    }
    string point = prefix + "-point";
    string line = prefix + "-line";
    SHPHandle hshp = SHPCreate(point.c_str(), SHPT_POINT);
    DBFHandle hdbf = DBFCreate(point.c_str());
    DBFAddField(hdbf, "CROSS", FTInteger, 10, 0);
    DBFAddField(hdbf, "TIME", FTString, 20, 0);

    for(auto& p : traj)
    {
        Cross const& c = map.cross(p.cross_id);
        double x = c.geometry.x();
        double y = c.geometry.y();
        SHPObject* pnt = SHPCreateSimpleObject(SHPT_POINT,1, &x, &y, nullptr);
        int const insertAtEnd = -1;
        int const insertedID = SHPWriteObject(hshp, insertAtEnd, pnt);
        DBFWriteIntegerAttribute(hdbf, insertedID,  0, p.cross_id);
        DBFWriteStringAttribute(hdbf, insertedID, 1, boost::lexical_cast<string>(p.time_of_day).c_str());
        SHPDestroyObject(pnt);
    }

    SHPClose(hshp);
    DBFClose(hdbf);
    hshp = SHPCreate(line.c_str(), SHPT_ARC);
    for(int i = 1; i < traj.size(); ++i)
    {
        Cross const& a = map.cross(traj[i-1].cross_id);
        Cross const& b = map.cross(traj[i].cross_id);
        double x[2] = {a.geometry.x(), b.geometry.x()};
        double y[2] = {a.geometry.y(), b.geometry.y()};
        SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, 0);
        SHPWriteObject(hshp, -1, line);
        SHPDestroyObject(line);
    }
    SHPClose(hshp);
}

size_t LCS(vector<TimedCrossInDay> const& traj, vector<TimedCrossInDay> const& restore)
{
    vector<size_t> index;
    index.reserve(restore.size());
    unordered_map<int, vector<int> > index_map;
    //save index for restore
    for(size_t i = 0; i < restore.size(); ++i)
        index_map[restore[i].cross_id].push_back(i);


    for(int idxA = 0; idxA < traj.size(); ++idxA)
    {
        int ID = traj[idxA].cross_id;
        for(int idxB : index_map[ID] | b::adaptors::reversed)
            index.push_back(idxB);
    }
    //index.second

    //LIS
    //vector<pair<int, int> > b(index.size());
    //vector<int> pre(index.size(), -1);
    ////b[0] = {numeric_limits<int>::min(), -1};
    //int len = 0;
    //for(size_t i = 0; i < index.size(); ++i)
    //{
    //    /*pair<int, int>* iter = upper_bound(&b[0], &b[len+1], index[i].second,[](int n, pair<int, int> const& p){
    //        return n <= p.first;
    //    });*/
    //    pair<int, int>* iter = lower_bound(&b[0], &b[len + 1], index[i].second,[](pair<int, int> const& p, int n) {
    //                return p.second < n;
    //            });
    //    if ( iter == &b[len+1] )
    //    {//inc len & save
    //        b[len+1] = {index[i].second,i};
    //        pre[i] = b[len++].second;
    //    }else if( iter->first > index[i].second)
    //    {//update
    //        pre[i] = pre[iter->second];
    //        *iter = { index[i].second, i };
    //    }
    //}

    //int i = b[len].second;
    //vector<Index> lisIndex;
    //while( i != -1 )
    //{
    //    lisIndex.push_back(index[i]);
    //    i = pre[i];
    //}

    vector<size_t> l;
    for(auto i : index) {
        auto iter = lower_bound(l.begin(), l.end(), i);
        if ( iter == l.end() ) {
            l.push_back(i);
        }else {
            *iter = i;
        }
    }

    return l.size();
}

double eval(vector<TimedCrossInDay> const& traj, vector<TimedCrossInDay> const& restore)
{   
    return LCS(traj, restore) * 2.0 / (traj.size() + restore.size());
}

namespace std
{
    template<>
    struct hash<pair<int, int> >
    {
        size_t operator()(pair<int, int> const& p)const
        {
            hash<int> hasher;
            return hasher(p.first) ^ (hasher(p.second) << 1);
        }
    };
}

template<typename F>
void visit_traj(mysqlpp::Connection& con, string const& where, F f)
{
    mysqlpp::Query trajMetaQ = con.query("SELECT `id` FROM `traj_metadata` WHERE %0");
    trajMetaQ.parse();
    list<mysqlpp::Row> allMetaIDs;
    trajMetaQ.storein(allMetaIDs, where);
    clog << "#For all traj where " << where << endl;
    int allSZ = allMetaIDs.size();
    int cur = 0;
    boost::timer t;
    while(!allMetaIDs.empty())
    {
        int metaID = allMetaIDs.front()[0];
        allMetaIDs.pop_front();
        ++cur;
        if ( t.elapsed() > 0.1 )
        {
            clog << "#" << cur << "/" << allSZ << "\r" << flush;
            t.restart();
        }
        vector<TimedCrossIndex> traj = load_timed_path_from_DB(con, metaID);
        f(traj);
    }
}

void dump_foot_graph(HighLevelMap const& hmap, unordered_map<pair<int, int>, int> const& foot_graph, fs::path const& shpOutput)
{
    SHPHandle shp = SHPCreate(shpOutput.c_str(), SHPT_ARC);
        DBFHandle dbf = DBFCreate(shpOutput.c_str());
        DBFAddField(dbf, "START", FTInteger, 10, 0);
        DBFAddField(dbf, "END", FTInteger, 10, 0);
        DBFAddField(dbf, "DBID", FTInteger, 10, 0);
        DBFAddField(dbf, "COUNT", FTInteger, 10, 0);
        
        for(auto&  p : foot_graph)
        {
            auto& edge = p.first;
            int count = p.second;
            for(auto& rcIdx : hmap.successor_road_of(hmap.cross("ID",edge.first).index))
            {
                if(hmap.cross("ID", edge.second).index == rcIdx.cross_index())
                {
                    Cross const& c1 = hmap.cross("ID", edge.first);
                    Cross const& c2 = hmap.cross("ID", edge.second);
                    int DBID = hmap.roadsegment(rcIdx.road_index()).properties.get<int>("METADATA-ID");
                    double x[2] = {c1.geometry.x(), c2.geometry.x()};
                    double y[2] = {c1.geometry.y(), c2.geometry.y()};
                    SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, 0);
                    const int doInsert = -1;
                    int insertedID = SHPWriteObject(shp, doInsert, line);
                    DBFWriteIntegerAttribute(dbf, insertedID, 0, edge.first);
                    DBFWriteIntegerAttribute(dbf, insertedID, 1, edge.second);
                    DBFWriteIntegerAttribute(dbf, insertedID, 2, DBID);
                    DBFWriteIntegerAttribute(dbf, insertedID, 3, count);
                }
            }
        }
        SHPClose(shp);
        DBFClose(dbf);
}

template<size_t N, typename Vec>
boost::iterator_range<typename Vec::iterator> most(Vec & v) {
    if( v.size() < N) {
        return boost::make_iterator_range(v.begin(), v.end());
    }

    return boost::make_iterator_range(v.begin(), v.begin() + N);
}

ostream& operator<< ( ostream& o, vector<size_t> const& v ) {
    if (v.empty()) return o;
    o << v[0];
    for (size_t i = 1; i < v.size();++i){
        o <<','<<v[i];
    }
    return o;
}

//std::istream& operator>>( std::istream& in, vector<size_t> v) {
//    size_t t;
//    if ( in >> t ) {
//        v.push_back(t);
//        while ( in.ignore() && in >> t ) {
//            v.push_back(t);
//        }
//    }
//
//    return in;
//}

int main(int argc, char *argv[])
{
    HighLevelMap hmap;
    RoadMap rmap;
    string host;
    string username;
    string passwd;
    string shp;
    string crossShp;
    string vshp;
    string where;
    fs::path output;
    po::options_description desc(argv[0]);
    vector<size_t> straight = {1};
    size_t leftMax = 1;
    size_t rightMax = 1;
    desc.add_options()
        ("help,h", "show help")
        ("username,u", po::value(&username)->default_value("root"), "username")
        ("host,h", po::value(&host)->default_value("localhost"), "host")
        ("passwd,p", po::value(&passwd)->required(), "password")
        ("road,r", po::value(&shp)->required()->default_value("../data/map/bj-road-epsg3785"), "road map shapefile")
        ("vroad,v", po::value(&vshp)->required()->default_value("../data/vmap/virtualEdgeTop4000Weekdays"), "virtual map shapefile")
        ("where", po::value(&where)->default_value("DAYOFWEEK(`date`) in (2,3,4,5,6)"), "where cause to filter traj")
        ("output,o",po::value(&output)->required(), "output directory")
        ("left", po::value(&leftMax), "left in turn")
        ("right", po::value(&rightMax), "right in turn")
        ("straight", po::value(&straight)->multitoken(), "straight in turn");
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if ( vm.count("help") )
        {
            cout << desc << endl;
            return 0;
        }
        vm.notify();
    }catch(exception const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }

    if ( !hmap.load(vshp, host.c_str(), username.c_str(), passwd.c_str()) )
    {
        cerr << "can not load hight level map" << endl;
        return 1;
    }
    clog << "#High level map loaded." << endl;

    if ( !rmap.load(shp) )
    {
        cerr  << "can not load " << shp << endl;
        return 1;
    }
    clog << "#Road map loaded." << endl;

    string sql;
    mysqlpp::Connection con;
    try
    {
        con.connect("path_restore", host.c_str(), username.c_str(), passwd.c_str());
    }catch(mysqlpp::Exception const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }
    clog << "#Database connected." << endl;
    if(!fs::exists(output))
    {
        boost::system::error_code err;
        fs::create_directories(output, err);
        if(err)
        {
            cerr << err.message() << endl;
            return 1;
        }
    }
    string date;
    int start, end, carid, subid;

    clog << "#Input <data> <car_id> < sub_id> <start> <end>" << endl;
    clog << "#Eg: 20121115 20334 0 127 170" << endl;
    clog << "#Result:" << endl;
    cout << "#candidate size,shortest lcs rate, shortest lcs hot rate, best lcs rate, best lcs hot rate, avg lcs rate,avg lcs hot rate, lcs better rate, lcs hot better rate,lcs not bad then, lcs hot not bad then, better avg,better hot avg,not bad avg, not bad hot avg,contain sp,cover rate, cover hot rate" << endl;
    while(cin>> date  >> carid >> subid >> start >> end)
    {
        cin.ignore(1024,'\n');
        hmap.clear_tmp();
        mysqlpp::Query q = con.query("SELECT `cross_id`, `time` FROM `traj_data`,`traj_metadata` "
                    "WHERE `metadata_id` = `traj_metadata`.`id` and "
                    "`date` = DATE(%0Q) and `car_id` = %1 and `sub_id` = %2 "
                    "LIMIT %3, %4");
        q.parse();
        mysqlpp::StoreQueryResult result = q.store(date, carid, subid, start, end - start);
        vector<TimedCrossInDay> traj;
        vector<TimedCrossInDay> trajHot;
        for(auto& r : result)
        {
            int cross_id = r[0];
            ptime time = time_from_string(r[1].c_str());
            TimedCrossInDay tc;
            tc.cross_id = cross_id;
            tc.time_of_day = time.time_of_day();
            traj.push_back(tc);
        }
        boost::trim_if(traj, [&](TimedCrossInDay const& tc){ return !hmap.contain_cross("ID", tc.cross_id); });
        trajHot = project(hmap, traj);
        fs::path  dump_prefix = output/(boost::format("traj-%s-%d-%d-%d-%d")% date % carid % subid % start %end).str();
        fs::ofstream log(dump_prefix/"log.txt");

        fs::create_directories(dump_prefix);
        draw_path(rmap, (dump_prefix/"raw").string(), traj);
        draw_path(rmap, (dump_prefix/"hot").string(), trajHot);
        unordered_set<int> start_hot_crosses, end_hot_crosses;
        start_hot_crosses.insert(traj.front().cross_id);
        end_hot_crosses.insert(traj.back().cross_id);
       
        unordered_map<pair<int, int>, int> foot_graph;
        unordered_map<pair<int, int>, int> foot_graph_out;
        unordered_map<pair<int, int>, int> foot_graph_in;
        time_duration acceptableCost = seconds((traj.back().time_of_day - traj.front().time_of_day).total_seconds() * 1.25);
        log << "start:" << traj.front().cross_id << " end:" << traj.back().cross_id << " acceptableCost:" << acceptableCost << endl;
        vector<TimedCrossInDay> hmapSPtraj = transform(hmap,
                hmap.shortest_path(hmap.cross("ID", traj.front().cross_id).index, traj.front().time_of_day,
                    hmap.cross("ID", traj.back().cross_id).index));
        if( hmapSPtraj.empty() )
        {
            cout << "#no solve." << endl;
            log << "#no solve." << endl;
            continue;
        }
        Box range;
        bg::envelope(rmap.cross(traj.front().cross_id), range);
        for(auto& p : hmapSPtraj)
        {
            bg::expand(range, rmap.cross(p.cross_id));
        }
        bg::correct(range);
        Point min = range.min_corner();
        Point max = range.max_corner();
        int w = max.x() - min.x();
        int h = max.y() - min.y();
        int modify = ::min(w,h);
        range.min_corner().x(range.min_corner().x() - modify);
        range.max_corner().x(range.max_corner().x() + modify);
        range.min_corner().y(range.min_corner().y() - modify);
        range.max_corner().y(range.max_corner().y() + modify);
        draw_box((dump_prefix/"range").string(), range);
        draw_path(rmap, (dump_prefix/"hmap-sp").string(), hmapSPtraj);
        double lcs_rate = eval(traj, hmapSPtraj);
        double lcs_rate_hot = eval(trajHot, project(hmap, hmapSPtraj));
        fs::path shpOutput = dump_prefix/"footmarkGraph";
        if (! fs::exists(dump_prefix/"footmarkGraph.shp") )
        {
            clog <<"#generating foot graph..." << endl;
            //遍历所有traj找到支撑数据集
            visit_traj(con, where, [&](vector<TimedCrossIndex> const& traj)
            {
                auto in_start_hot_cross = 
                    [&](TimedCrossIndex const& tc){ int cross_id = tc.first; return start_hot_crosses.count(cross_id); };
                auto in_end_hot_cross = 
                    [&](TimedCrossIndex const& tc){ int cross_id = tc.first; return end_hot_crosses.count(cross_id); };
            {
                auto it = find_if(traj.begin(), traj.end(), in_start_hot_cross);
                while( it != traj.end() )
                {
                    vector<TimedCrossInDay> traj_in_day;
                    traj_in_day.push_back({it->first, it->second.time_of_day()});
                    auto i = std::next(it);
                    for(; i < traj.end() && 
                        !start_hot_crosses.count(i->first) && 
                        (i->second - it->second) < acceptableCost &&
                        bg::within(rmap.cross(i->first).geometry, range)
                        ; ++i)
                    {
                        if(hmap.contain_cross("ID", i->first))
                        {
                            traj_in_day.push_back({i->first, i->second.time_of_day()});
                            if (in_end_hot_cross(*i))
                                break;
                        }
                    }
                    for(int j = 1; j < traj_in_day.size(); ++j)
                        foot_graph_out[{traj_in_day[j-1].cross_id,traj_in_day[j].cross_id}]++;
                    it = find_if(i, traj.end(), in_start_hot_cross);
                }
            }
            {
                auto it = find_if(traj.rbegin(), traj.rend(), in_end_hot_cross);
                while(it != traj.rend())
                {
                    vector<TimedCrossInDay> traj_in_day;
                    traj_in_day.push_back({it->first, it->second.time_of_day()});
                    auto i = std::next(it);
                    for(; i != traj.rend() && 
                            !in_end_hot_cross(*i) && 
                            bg::within(rmap.cross(i->first).geometry, range)&&
                            (it->second - i->second) < acceptableCost; ++i)
                    {
                        if(hmap.contain_cross("ID", i->first))
                        {
                            traj_in_day.push_back({i->first, i->second.time_of_day()});
                            if ( in_start_hot_cross(*i) )
                                break;
                        }
                    }
                    boost::reverse(traj_in_day);
                    for(int j = 1; j < traj_in_day.size(); ++j)
                        foot_graph_in[{traj_in_day[j-1].cross_id, traj_in_day[j].cross_id}]++;
                    it = find_if(i, traj.rend(), in_end_hot_cross);
                }
            }
            });
            fs::path shpOutput = dump_prefix/"footmarkGraphIn";
            dump_foot_graph(hmap, foot_graph_in, shpOutput);
            shpOutput = dump_prefix/"footmarkGraphOut";
            dump_foot_graph(hmap, foot_graph_out, shpOutput);
            shpOutput = dump_prefix/"footmarkGraph";
            for(auto& p : foot_graph_in)
            {
                if(p.second > 44)
                    foot_graph[p.first] += p.second;
            }
            for(auto& p : foot_graph_out)
            {
                if(p.second > 44)
                    foot_graph[p.first] += p.second;
            }
            dump_foot_graph(hmap, foot_graph, shpOutput);
        }
        HighLevelMap testmap;
        if(!testmap.load(shpOutput.string(), host.c_str(), username.c_str(), passwd.c_str()))
        {
            clog << "#fail to load :" << shpOutput << endl;
            continue;
        }
        auto shortestTraj = transform(testmap, 
                testmap.shortest_path(testmap.cross("ID", traj.front().cross_id).index, 
                    traj.front().time_of_day,testmap.cross("ID", traj.back().cross_id).index));
        if(shortestTraj.empty())
        {
            cout << "#no solve." << endl;
            continue;
        }
        time_duration delta = acceptableCost / 100;
        KShortestPathGenerator gen(testmap, testmap.cross("ID", traj.front().cross_id).index,
                traj.front().time_of_day,
                testmap.cross("ID", traj.back().cross_id).index,
                traj.back().time_of_day + delta,
                range
                );

        vector<RestoreInfo> candidates;
        int k = 0;
        while(auto ppath = gen.nextPath())
        {
            if ( k > 10000 ) break;
            if ( ppath->empty()) continue;
            //if( ppath->back().time < traj.back().time_of_day - delta )
             //   continue;
            RestoreInfo info;
            info.k = k++;
            info.restore = transform(testmap, *ppath);
            info.prj = project(hmap, info.restore);
            info.lcs_rate = eval(traj, info.restore);
            info.lcs_rate_hot = eval(trajHot, info.prj);
            info.cost = info.restore.back().time_of_day - info.restore.front().time_of_day;
            info.turn = turns(rmap, info.restore);
            info.turn_hot = turns(rmap, info.prj);
            candidates.push_back(std::move(info));
        }
        cout << "--------------------------------------------------------------" << endl;
        for (int s : straight) { StraightCount = s;
        for (LeftCount = 0;LeftCount < leftMax; ++LeftCount) {
            for(RightCount = 0;RightCount < rightMax; ++RightCount){
                for(auto& info : candidates) {
                    info.turn = turns(rmap, info.restore);
                    info.turn_hot = turns(rmap, info.prj);
                }

                boost::sort(candidates, 
                        [](RestoreInfo const& i1, RestoreInfo const& i2){ return i1.turn_hot < i2.turn_hot; });
                //if ( candidates.size() > 10 )
                //    candidates.resize(10);
                k = 1;
                /*
                for(auto& info : candidates)
                {
                    //draw_path(rmap, (dump_prefix/"candidate-").string() + to_string(k), info.restore);
                    //draw_path(rmap, (dump_prefix/"candidate-prj-").string() + to_string(k), info.prj);
                    //cout << info.lcs_rate << " " << info.lcs_rate_hot << " " << info.cost << " " << info.k << info.turn << " " << info.turn_hot << endl;
                    //log << k++ << " " << info.lcs_rate << " " << info.turn << " " << info.lcs_rate_hot << " " << info.turn_hot << endl;
                }*/
                double best_lcs_rate = boost::max_element(most<10>(candidates), 
                        [](RestoreInfo const& i1, RestoreInfo const& i2){
                return i1.lcs_rate < i2.lcs_rate;
                } )->lcs_rate;
                double best_lcs_rate_hot = boost::max_element(most<10>(candidates),
                        [](RestoreInfo const& i1, RestoreInfo const& i2){
                        return i1.lcs_rate_hot < i2.lcs_rate_hot;
                })->lcs_rate_hot;
                int lcs_better_cnt = 0, lcs_hot_better_cnt = 0;
                int cover_cnt = 0;
                int cover_hot_cnt = 0 ;
                int lcs_not_bad_cnt = 0;
                int lcs_hot_not_bad_cnt = 0;
                set<int> traj_cross_set;
                set<int> traj_cross_hot_set;
                set<int> restore_cross;
                set<int> restore_hot_cross;
                for(auto& p : traj)
                    traj_cross_set.insert(p.cross_id);
                for(auto& p : trajHot)
                    traj_cross_hot_set.insert(p.cross_id);
                double lcs_rate_sum = 0;
                double lcs_hot_rate_sum = 0;
                double lcs_better_sum = 0;
                double lcs_hot_better_sum = 0;
                double lcs_not_bad_sum = 0;
                double lcs_hot_not_bad_sum = 0;
                bool contain_shotest_test = false;
                for(auto& info : most<10>(candidates))
                {
                    lcs_rate_sum += info.lcs_rate;
                    lcs_hot_rate_sum += info.lcs_rate_hot;
                    if ( info.lcs_rate > lcs_rate )
                    {
                        ++lcs_better_cnt;
                        lcs_better_sum += info.lcs_rate;
                    }
                    if ( info.lcs_rate >= lcs_rate )
                    {
                        ++lcs_not_bad_cnt;
                        lcs_not_bad_sum += info.lcs_rate;
                    }
                    if ( info.lcs_rate == lcs_rate && info.lcs_rate_hot == lcs_rate_hot)
                        contain_shotest_test = true;
                    if (info.lcs_rate_hot > lcs_rate_hot)
                    {
                        ++lcs_hot_better_cnt;
                        lcs_hot_better_sum += info.lcs_rate_hot;
                    }
                    if (info.lcs_rate_hot >= lcs_rate_hot)
                    {
                        ++lcs_hot_not_bad_cnt;
                        lcs_hot_not_bad_sum += info.lcs_rate_hot;
                    }
                    for(auto& p :info.restore)
                        restore_cross.insert(p.cross_id);
                    for(auto& p:info.prj)
                        restore_hot_cross.insert(p.cross_id);
                }
                for(int i : restore_cross)
                    if( traj_cross_set.count(i) ) ++cover_cnt;
                for(int i : restore_hot_cross)
                    if(traj_cross_hot_set.count(i))++ cover_hot_cnt;
                size_t n_candidate = most<10>(candidates).size();
                cout << "("<<StraightCount<<","<<LeftCount<<","<<RightCount<<"),"
                    << n_candidate << "," 
                    << lcs_rate << "," 
                    << lcs_rate_hot << "," 
                    << best_lcs_rate << "," 
                    << best_lcs_rate_hot << "," 
                    << lcs_rate_sum / n_candidate << ","
                    << lcs_hot_rate_sum / n_candidate << ","
                    << (double)lcs_better_cnt / n_candidate << ","
                    << (double)lcs_hot_better_cnt / n_candidate << ","
                    << (double)lcs_not_bad_cnt / n_candidate << ","
                    << (double)lcs_hot_not_bad_cnt / n_candidate << ","
                    << lcs_better_sum / lcs_better_cnt << ","
                    << lcs_hot_better_sum / lcs_hot_better_cnt << ","
                    << lcs_not_bad_sum / lcs_not_bad_cnt << ","
                    << lcs_hot_not_bad_sum / lcs_hot_not_bad_cnt << ","
                    << contain_shotest_test << ","
                    << (double)cover_cnt / traj_cross_set.size() << ","
                    << (double)cover_hot_cnt / traj_cross_hot_set.size() << endl;
            }}
        }
    }
    return 0;
}
