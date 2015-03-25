//auto-include {{{
#include <string>
#include <vector>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/date_time.hpp>
#include <boost/range/algorithm.hpp>
#include "roadmap.h"
#include "time_estimate.h"
#include "ivmm/ivmm.h"
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
    string output;
    double d;
    po::options_description desc(argv[0]);
    desc.add_options()
        (",r", po::value(&road)->required(), "road shp")
        (",c", po::value(&crossInfo)->required(), "cross info")
        (",k", po::value(&k)->default_value(1000), "top k")
        (",f", po::value(&f)->required(), "filter")
        (",d", po::value(&d)->default_value(0.95), "remain")
        ("output,o",po::value(&output)->required(), "output directory")
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

    string line;
    ifstream ins(crossInfo);
    unordered_set<int> hotCross;
    unordered_map<pair<int, int>, FilterInfo> timeFilterMap;
    EdgeMap edgeMap;
    if ( ! ins ){
        cerr << "can not open " << argv[1] << endl;
        return 1;
    }
    RoadMap map;
    if ( ! map.load(road) ){
        cerr << "can not load shp " << road << endl;
        return 1;
    }

    fs::path outputTimeDir(output);
    outputTimeDir/="time";
    if ( !fs::exists(outputTimeDir) ){
        boost::system::error_code err;
        fs::create_directories(outputTimeDir, err);
        if ( err ){
            cerr << err.message() << endl;
            return 1;
        }
    }

    int index;
    int count;
    double e;
    while( k-- && ins >> index &&  ins.ignore() && ins >> count && ins.ignore() && ins >> e)
    {
        hotCross.insert(index);
    }
    ins.close();
    cout << "load hot cross" << endl;
    while(getline(cin, line)){
        vector<TimedCrossIndex> timedPath = loadTimedPathFromFile(line);
        typedef vector<TimedCrossIndex>::iterator Iter;


        int cross;
        ptime midTime;
        TimedPathIter it = findNextHotCross(timedPath.begin(), timedPath.end(), hotCross, cross, midTime);
        cout << "loading: " << line << endl;
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
                    Path path = map.shortestPathAStar(cross, cross2);
                    filter.length = path.totalLength();
                    filter.speed = min(weightSpeed(path, map) * 3, 33.3);
                    //filter.speed = 33.3;
                    filter.cost = static_cast<int>(filter.length / filter.speed) * 1000;
                    cout << "from:"<< cross << " to:" << cross2 << " length:" << filter.length << " fspeed:" << filter.speed << " must > " << filter.cost << " ms" << endl;
                    timeFilterMap[edge] = filter;
                }

                int costMill = (midTime2 - midTime).total_milliseconds() ;
                if ( costMill > filter.cost ){
                    edgeMap[{cross, cross2}].push_back({midTime, midTime2});
                }
                else{
                    cout << "cost too less from:"<<cross << " to:" << cross2 << " length:" << filter.length << " fspeed:" << filter.speed << " use:" << costMill <<" ms" << " < " << filter.cost << " ms"<<endl;
                }
                cross = cross2;
                midTime = midTime2;
            }
        }
    }

    fs::path outputShp(output);
    outputShp /= "visualEdge";
    SHPHandle shp = SHPCreate(outputShp.c_str(), SHPT_ARC);
    DBFHandle dbf = DBFCreate(outputShp.c_str());
    DBFAddField(dbf, "SCROSS", FTInteger, 10, 0);
    DBFAddField(dbf, "ECROSS", FTInteger, 10, 0);
    DBFAddField(dbf, "COUNT", FTInteger, 10, 0);
    cout << "writing..." << endl;
    for(auto& each : edgeMap){
        pair<int, int> const& edge = each.first;
        vector<pair<ptime, ptime> > & timePairs = each.second;
        boost::sort(timePairs, [](pair<ptime, ptime> const& a, pair<ptime, ptime> const& b){
                return a.second - a.first < b.second - b.first;
        });
        timePairs.resize(timePairs.size()*d);
        if ( timePairs.size() > f ){
            double x[2] = {map.cross(edge.first).geometry.x(), map.cross(edge.second).geometry.x()};
            double y[2] = {map.cross(edge.first).geometry.y(), map.cross(edge.second).geometry.y()};
            SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
            const int insertAtEnd = -1;
            int id = SHPWriteObject(shp, -1, line);
            DBFWriteIntegerAttribute(dbf, id, 0, edge.first);
            DBFWriteIntegerAttribute(dbf, id, 1, edge.second);
            DBFWriteIntegerAttribute(dbf, id, 2, timePairs.size());
            SHPDestroyObject(line);
            string s;
            fs::ofstream outs(outputTimeDir/(to_string(edge.first).append("-").append(to_string(edge.second).append(".txt"))));
            outs.imbue(locale(outs.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%S%F")));
            for(auto& timePair : timePairs)
            {
                outs << timePair.first << "," << timePair.second << "," << (timePair.second - timePair.first).total_milliseconds() << "\n";
            }
        }
    }
    SHPClose(shp);
    DBFClose(dbf);
    return 0;
}
