#include <boost/program_options.hpp>
#include <mysql++.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include "path-restore/highlevelmap.h"
#include "roadmap.h"
#include "util.h"
#include "sutil/boost/date_time/date_time_format.hpp"
#include    "experiment/com.h"
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace boost::posix_time;

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
void drawPath(RoadMap const& map, string const& prefix, vector<TimedCrossInDay> const& traj)
{
    fs::path prefixPath(prefix);
    if(prefixPath.has_parent_path())
    {
        fs::path ppath = prefixPath.parent_path();
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
vector<pair<int, int> > LCS(vector<TimedCrossInDay> const& traj, vector<TimedCrossInDay> const& restore)
{
    typedef pair<int, int> Index;
    vector<Index> index;
    index.reserve(restore.size());
    unordered_map<int, vector<int> > indexMap;
    for(int i = 0; i < restore.size(); ++i)
        indexMap[restore[i].cross_id].push_back(i);
    for(int idxA = 0; idxA < traj.size(); ++idxA)
    {
        int ID = traj[idxA].cross_id;
        if ( indexMap.count(ID) )
        {
            for(int idxB : indexMap[ID] | b::adaptors::reversed)
                index.push_back({idxA, idxB});
        }
    }
    //index.second

    //LIS
    vector<pair<int, int> > b(index.size()+1);
    vector<int> pre(index.size(), -1);
    b[0] = {numeric_limits<int>::min(), -1};
    int len = 0;
    for(int i = 0; i < index.size(); ++i)
    {
        /*pair<int, int>* iter = upper_bound(&b[0], &b[len+1], index[i].second,[](int n, pair<int, int> const& p){
            return n <= p.first;
        });*/
        pair<int, int>* iter = lower_bound(&b[0], &b[len], index[i].second, [](pair<int, int> const& p, int n){
                return p.first < n;
        });
        //if ( iter == &b[len+1] )
        if ( iter == &b[len] )
        {//inc len & save
            //b[len+1] = {index[i].second,i};
            b[len] = {index[i].second,i};
            pre[i] = b[len++].second;
        }else if( iter->first > index[i].second)
        {//update
            pre[i] = pre[iter->second];
            *iter = { index[i].second, i };
        }
    }

    //int i = b[len].second;
    int i = b[len-1].second;
    vector<Index> lisIndex;
    while( i != -1 )
    {
        lisIndex.push_back(index[i]);
        i = pre[i];
    }
    return lisIndex;
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
};
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
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("username,u", po::value(&username)->default_value("root"), "username")
        ("host,h", po::value(&host)->default_value("localhost"), "host")
        ("passwd,p", po::value(&passwd)->required(), "password")
        ("road,r", po::value(&shp)->required()->default_value("../data/map/bj-road-epsg3785"), "road map shapefile")
        ("vroad,v", po::value(&vshp)->required()->default_value("../data/vmap/virtualEdgeTop4000Weekdays"), "virtual map shapefile");
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
    clog << "High level map loaded." << endl;

    if ( !rmap.load(shp) )
    {
        cerr  << "can not load " << shp << endl;
        return 1;
    }
    clog << "Road map loaded." << endl;

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
    clog << "Database connected." << endl;
    string date;
    int start, end, carid, subid;
    cout << "Input <date> <carid> <subid> <start> <end>" << endl;
    cout << "Eg: 20121101 20484 0 35 130" << endl;
    cout << "The traj can use QGis python plugin to show" << endl;
    while(cin >> date >> carid >> subid >> start >> end)
    {
        hmap.clear_tmp();
        try
        {
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
            for(auto& tc : traj)
            {
                if ( hmap.contain_cross("ID", tc.cross_id) )
                    trajHot.push_back(tc);
            }
            string dumpPrefix = (boost::format("dump/traj-%s-%d-%d-%d-%d/")%date%carid%subid%start%end).str();
            drawPath(rmap, dumpPrefix+"traj", traj);
            drawPath(rmap, dumpPrefix+"trajHot",trajHot);

            unordered_set<int> startHotCrosses, endHotCrosses;
            if (! hmap.contain_cross("ID", traj.front().cross_id ) )
            {
                Cross const& c = rmap.cross(traj.front().cross_id);
                int startHIdx = hmap.add_tmp_cross(traj.front().cross_id, c.geometry.x(), c.geometry.y()).index;
                auto nearestRange = hmap.cross_rtree | bgi::adaptors::queried(bgi::nearest(c, 1));
                double dis = bg::distance(hmap.cross_or_tmp_one(startHIdx), nearestRange.begin()->first);
                dis = min(dis*1.1, 1000.0);
                for(auto& p : hmap.cross_rtree | bgi::adaptors::queried(bgi::within(range_box(c.geometry, dis))))
                {
                    int nearbyHIdx = p.second;
                    int nearbyID = hmap.cross(nearbyHIdx).properties.get<int>("ID");
                    int startID = traj.front().cross_id;
                    Path path = rmap.shortest_path(startID, nearbyID);
                    if ( path.valid() )
                    {
                        hmap.add_tmp_virtual_edge(startHIdx, nearbyHIdx, estimation_cost(rmap, path));
                        startHotCrosses.insert(nearbyID);
                    }
                }
            }else
            {
                startHotCrosses.insert(traj.front().cross_id);
            }

            if ( ! hmap.contain_cross("ID", traj.back().cross_id) )
            {
                Cross const& c = rmap.cross(traj.back().cross_id);
                int endHIdx = hmap.add_tmp_cross(traj.back().cross_id, c.geometry.x(), c.geometry.y()).index;
                auto nearestRange = hmap.cross_rtree | bgi::adaptors::queried(bgi::nearest(c, 1));
                double dis = bg::distance(hmap.cross_or_tmp_one(endHIdx), nearestRange.begin()->first);
                dis = min(dis*1.1, 1000.0);
                for(auto& p : hmap.cross_rtree | bgi::adaptors::queried(bgi::within(range_box(c.geometry, dis))))
                {
                    int nearbyHIdx = p.second;
                    int nearbyID = hmap.cross(nearbyHIdx).properties.get<int>("ID");
                    int endID = traj.back().cross_id;
                    Path path = rmap.shortest_path(nearbyID, endID);
                    if ( path.valid() )
                    {
                        hmap.add_tmp_virtual_edge(nearbyHIdx, endHIdx, estimation_cost(rmap, path));
                        endHotCrosses.insert(nearbyID);
                    }
                }
            }else
            {
                endHotCrosses.insert(traj.back().cross_id);
            }//end of 添加临时点和临时边

            unordered_map<pair<int, int>, int> footGraph;
            

            ///////////////////////////////////
            time_duration d = seconds(5);//;(traj.back().time_of_day - traj.front().time_of_day) / 100;
            Box b(hmap.cross_or_tmp_one_by_id(traj.front().cross_id).geometry, 
                    hmap.cross_or_tmp_one_by_id(traj.back().cross_id).geometry);
            bg::correct(b);
            Point min = b.min_corner();
            Point max = b.max_corner();
            int w = max.x() - min.x();
            int h = max.y() - min.y();
            int modify = ::min(w,h);
            b.min_corner().x(b.min_corner().x() - modify);
            b.max_corner().x(b.max_corner().x() + modify);
            b.min_corner().y(b.min_corner().y() - modify);
            b.max_corner().y(b.max_corner().y() + modify);
            draw_box(dumpPrefix + "box", b);
            KShortestPathGenerator kgen(hmap, 
                    hmap.cross_or_tmp_one_by_id(traj.front().cross_id).index, 
                    traj.front().time_of_day, 
                    hmap.cross_or_tmp_one_by_id(traj.back().cross_id).index, 
                    traj.back().time_of_day + d, 
                    b
                    );

            cout << "input start time: " << traj.front().time_of_day << " end time: " << traj.back().time_of_day << endl;
            vector<vector<HighLevelMap::TimedCross> const* > candidatePath;
            int k = 0;
            vector<HighLevelMap::TimedCross> const* pre = nullptr;
            while(vector<HighLevelMap::TimedCross> const* ppath = kgen.nextPath())
            {
                cout << k ++ << ":" << ppath->back().time << endl; /*
                int s = hmap.cross_or_tmp_one(ppath->front().cross_index).properties.get<int>("ID");
                time_duration t = ppath->front().time;
                vector<TimedCross> ksp;
                ksp.push_back({s, t});
                for(int i = 1; i < ppath->size(); ++i)
                {
                    auto& tc1 = ppath->at(i-1);
                    auto& tc2 = ppath->at(i);
                    Cross const& c1 = hmap.cross_or_tmp_one(tc1.cross_index);
                    Cross const& c2 = hmap.cross_or_tmp_one(tc2.cross_index);
                    time_duration t1 = tc1.time;
                    time_duration t2 = tc2.time;
                    Path pth = rmap.shortestPath(c1.properties.get<int>("ID"), c2.properties.get<int>("ID"));
                    weightTimeInterpolation(rmap, pth, t1, t2, ksp);
                }
                drawPath(rmap, dumpPrefix+"ksp-" + to_string(k), ksp);
                */
                //if ( ppath->back().time > traj.back().time_of_day - d)
                {
                    if ( ppath->back().time < traj.back().time_of_day + d)
                    {
                        candidatePath.push_back(ppath);
                    }
                    else break;
                }
            }
            int id = 0;
            for(auto& ppath : candidatePath)
            {
                ++id;
                int s = hmap.cross_or_tmp_one(ppath->front().cross_index).properties.get<int>("ID");
                time_duration t = ppath->front().time;
                vector<TimedCrossInDay> restorePath;
                restorePath.push_back({s, t});
                for(int i = 1; i < ppath->size(); ++i)
                {
                    auto& tc1 = ppath->at(i-1);
                    auto& tc2 = ppath->at(i);
                    Cross const& c1 = hmap.cross_or_tmp_one(tc1.cross_index);
                    Cross const& c2 = hmap.cross_or_tmp_one(tc2.cross_index);
                    time_duration t1 = tc1.time;
                    time_duration t2 = tc2.time;
                    Path pth = rmap.shortest_path(c1.properties.get<int>("ID"), c2.properties.get<int>("ID"));
                    weight_time_interpolation(rmap, pth, t1, t2, restorePath);
                }
                vector<TimedCrossInDay> restoreHot;
                for(auto& tc : restorePath)
                {
                    if (hmap.contain_cross("ID", tc.cross_id))
                        restoreHot.push_back(tc);
                }
                cout << id << ": " << ppath->back().time << endl;
                auto fullLCSIndex = LCS(traj, restorePath);
                auto hotLCSIndex = LCS(trajHot, restoreHot);
                cout << "full lcs:" << fullLCSIndex.size() << " traj:" << traj.size() << " restore:" << restorePath.size() << " acc = " << fullLCSIndex.size() * 2 /double(traj.size() + restorePath.size())  << endl;
                cout << "hot lcs:" << hotLCSIndex.size() << " traj:" << trajHot.size() << " restore:" << restoreHot.size() << " acc = " << hotLCSIndex.size() * 2 /double(trajHot.size() + restoreHot.size()) << endl << endl;
                drawPath(rmap, dumpPrefix+"restorePath-" + to_string(id), restorePath);
                drawPath(rmap, dumpPrefix+"restorePathHot-" + to_string(id), restoreHot);
            }
        }catch(mysqlpp::Exception const& e)
        {
            cerr << e.what() << endl;
        }
    }
    return 0;
}
