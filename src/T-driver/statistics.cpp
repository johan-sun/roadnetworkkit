//auto-include {{{
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <boost/date_time.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <mysql++.h>
#include <boost/timer.hpp>
#include "roadmap.h"
#include "time_estimate.h"
#include "bj-road-epsg3785/bj_road_epsg3785.h"
//}}}
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
struct CrossDetail
{
    int crossIndex;
    int count;
    double entropy;
};
struct Phase{
    int in;
    int out;
    bool operator==(Phase const& p)const{
        return in == p.in && out == p.out;
    }
};

int total(vector<pair<Phase,int> > const& v){
    int sum = 0;
    for(auto& p : v){
        sum += p.second;
    }
    return sum;
}
int main(int argc, char *argv[])
{
    RoadMap map;
    typedef unordered_map<int, vector<pair<Phase,int> > > CrossInfo;
    unordered_map<int, vector<pair<Phase,int> > > crossInfo;
    po::options_description desc(argv[0]);
    string shp;
    string type;
    fs::path output;
    string where;
    string user;
    string passwd;
    string host;
    desc.add_options()
        ("user,u", po::value(&user)->default_value("root"), "mysql username")
        ("host,h", po::value(&host)->default_value("localhost"), "mysql host")
        ("passwd,p", po::value(&passwd)->required(), "mysql password")
        ("road,r", po::value(&shp)->required(), "roadmap shp")
        ("sortby", po::value(&type)->required()->default_value("CountXEnt"), "sort type (CountXEnt,Count)")
        ("output,o", po::value(&output)->required(), "output")
        ("where", po::value(&where)->default_value("1"), "the `traj_metadata` WHERE clause to filter data ")
        ("help,h","show help");
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
    if ( type != "CountXEnt"  && type != "Count")
    {
        cerr << "type must be one of (CountXEnt, Count)" << endl;
        return 1;
    }

    if ( !map.load(shp,BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker())){
        cerr << "can not load " << shp << endl;
    }

    if( output.has_parent_path() )
    {
        fs::path ppath = output.parent_path();
        if ( !fs::exists(ppath) )
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
    string line;
    try
    {
        mysqlpp::Connection con("path_restore", host.c_str(), user.c_str(), passwd.c_str());
        mysqlpp::Query q = con.query("SELECT `id` FROM `traj_metadata` WHERE %0");
        mysqlpp::Query dq = con.query("SELECT `cross_id`, `time` FROM `traj_data` WHERE `metadata_id` = %0");
        q.parse();
        dq.parse();
        list<mysqlpp::Row> metaIDs;
        q.storein(metaIDs, where);
        size_t all = metaIDs.size();
        size_t cur = 0;
        boost::timer t;
        while(!metaIDs.empty())
        {
            ++cur;
            if ( t.elapsed() > 0.1 )
            {
                cout << cur << "/" << all <<"\r" << flush;
                t.restart();
            }
            int metaID = metaIDs.front()[0];
            metaIDs.pop_front();
            vector<TimedCrossIndex> timedPath;// = loadTimedPathFromFile(line);
            mysqlpp::StoreQueryResult sr = dq.store(metaID);
            for(auto& r : sr)
            {
                int idx = r[0];
                string s;
                r[1].to_string(s);
                timedPath.push_back({idx, boost::posix_time::time_from_string(s)});
            }
            if ( !timedPath.empty() ){
                vector<TimedCrossIndex>::iterator begin = timedPath.begin();
                vector<TimedCrossIndex>::iterator end = timedPath.end();
                while(begin < end){
                    auto pre = begin;
                    auto cur = find_if(begin ,end, [pre](TimedCrossIndex const& c){
                        return c.first != pre->first;
                    });
                    if ( cur == end ){
                        break;
                    }
                    auto succ = find_if(cur, end, [cur](TimedCrossIndex const& c){
                        return c.first != cur->first;
                    });
                    if ( succ == end ){
                        break;
                    }
                    begin = cur;
                    Phase p;
                    p.in = pre->first;
                    p.out = succ->first;
                    auto fd = boost::find_if(crossInfo[cur->first], [p](pair<Phase, int> const& phPair){ return phPair.first == p; });
                    if ( fd == crossInfo[cur->first].end()){
                        crossInfo[cur->first].push_back({p, 1});
                    }else{
                        fd->second++;
                    }
                }
            }
        }
        cout << "input done!" << endl;
    }catch(exception const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }

    vector<CrossDetail> allCrossInfoDetails;
    allCrossInfoDetails.reserve(crossInfo.size());
    cout << "sorting..." << endl;
    for(auto& eachInfo : crossInfo){
        int s = total(eachInfo.second);
        double e = 0.0;
        for(auto& eachPh : eachInfo.second){
            double p = (double)(eachPh.second) / s;
            e += p * log2(p);
        }
        CrossDetail d;
        d.crossIndex = eachInfo.first;
        d.count = s;
        d.entropy = -e;
        if ( abs(d.entropy) < 1e-6 ){
            d.entropy = 0;
        }
        allCrossInfoDetails.push_back(d);
    }
    if(type == "CountXEnt")
    {
        sort(allCrossInfoDetails.begin(), allCrossInfoDetails.end(), [](CrossDetail const& cd1, CrossDetail const& cd2){
            return cd1.count * cd1.count * cd1.entropy > cd2.count * cd2.count * cd2.entropy;
        });
    }else
    {
        sort(allCrossInfoDetails.begin(),allCrossInfoDetails.end(), [](CrossDetail const& cd1, CrossDetail const& cd2){
                return cd1.count > cd2.count;
        });
    }
    cout << "outputing..." << endl;
    SHPHandle hshp = SHPCreate(output.c_str(), SHPT_POINT);
    DBFHandle hdbf = DBFCreate(output.c_str());
    if ( !hshp || !hdbf )
    {
        cerr << "can not create shp" << endl;
        return 1;
    }
    DBFAddField(hdbf, "RANK", FTInteger, 10, 0);
    DBFAddField(hdbf, "INDEX", FTInteger, 10, 0);
    DBFAddField(hdbf, "COUNT", FTInteger, 10, 0);
    DBFAddField(hdbf, "ENTROPY", FTDouble, 12, 9);
    for(int i = 0; i < allCrossInfoDetails.size(); ++i){
        CrossDetail const& d = allCrossInfoDetails[i];
        Cross const& c= map.cross(d.crossIndex);
        double x = c.geometry.x();
        double y = c.geometry.y();
        SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, 0);
        const int insert = -1;
        int insertedID = SHPWriteObject(hshp, insert, point);
        DBFWriteIntegerAttribute(hdbf, insertedID, 0, i);
        DBFWriteIntegerAttribute(hdbf, insertedID, 1, c.index);
        DBFWriteIntegerAttribute(hdbf, insertedID, 2, d.count);
        DBFWriteDoubleAttribute(hdbf, insertedID, 3, d.entropy);
        SHPDestroyObject(point);
    }
    SHPClose(hshp);
    DBFClose(hdbf);
    cout << "done!" << endl;
    return 0;
}
