#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/algorithm.hpp>
#include <fstream>
#include <boost/date_time/time_facet.hpp>
#include <boost/date_time.hpp>
#include "time_estimate.h"
using namespace std;

struct TimeEstimater : public boost::static_visitor<double>{
    TimeEstimater(RoadMap const& map):map(map){}
    RoadMap const& map;
    double operator()(ARoadSegment const& rs)const{
        double speed = map.roadsegment(rs.roadsegment_index).properties.get<double>("SPEED");
        return rs.length / speed;
    }
    double operator()(PartOfRoad const& pr)const{
        double speed = map.roadsegment(pr.roadsegment_index).properties.get<double>("SPEED");
        return pr.length / speed;
    }
    double operator()(ProjectPoint const&)const{
        return 0.0;
    }
};

using b::posix_time::ptime;
vector<TimedCrossIndex> estimate_time(
    vector<GpsPoint> const& log,
    vector<Path> const& paths, 
    pair<int, int> const& log_range,
    RoadMap const& map){

    vector<TimedCrossIndex> timed_path;
    for(int i = log_range.first; i + 1 < log_range.second; ++i){
        GpsPoint const& prev = log[i];
        GpsPoint const& succ = log[i+1];
        Path const& path = paths[i];
        int esz = path.entities.size();
        if ( path.entities.front().type() == typeid(ProjectPoint) ){
            assert(esz == 1);
            ProjectPoint const& pp = get<ProjectPoint const&>(path.entities.front());
            if ( pp.type == ProjectPoint::OnCross ){
                timed_path.push_back({ pp.index, prev.time + (succ.time - prev.time) });
            }
        }else{
            double ms = ( succ.time - prev.time).total_milliseconds();
            double ems = 0;
            for(auto e : path.entities){
                ems += boost::apply_visitor(TimeEstimater(map), e);
            }
            ems *= 1000;
            double factor = ms / ems;
            double currentMs = 0.0;
            if ( path.entities.front().type() == typeid(PartOfRoad) ){
                PartOfRoad const& pr = get<PartOfRoad const&>(path.entities.front());
                if ( pr.start.type == ProjectPoint::OnCross){
                    assert(pr.end.type == ProjectPoint::OnRoad);
                    assert(esz == 1);
                    timed_path.push_back({pr.start.index, prev.time});
                }else if ( pr.end.type == ProjectPoint::OnCross ){
                    assert(pr.start.type == ProjectPoint::OnRoad);
                    if ( esz == 1 ) timed_path.push_back({pr.end.index, succ.time});
                    else{
                        double speed = map.roadsegment(pr.roadsegment_index).properties.get<double>("SPEED");
                        currentMs += pr.length / speed * 1000 * factor;
                        timed_path.push_back({pr.end.index, prev.time + b::posix_time::millisec(currentMs)});
                    }
                }else{
                    assert(esz == 1);
                }
            }else{
                assert(path.entities.front().type() == typeid(ARoadSegment));
                ARoadSegment const& rs = get<ARoadSegment const&>(path.entities.front());
                timed_path.push_back({rs.start_cross, prev.time});
                if ( esz == 1){
                    timed_path.push_back({rs.end_cross, succ.time});
                }else{
                    currentMs = rs.length / map.roadsegment(rs.roadsegment_index).properties.get<double>("SPEED") * 1000 * factor;
                    timed_path.push_back({rs.end_cross, prev.time + b::posix_time::millisec(currentMs)});
                }
            }

            for(auto it = std::next(path.entities.begin()); it != path.entities.end(); ++it){
                if ( it->type() == typeid(ARoadSegment) ){
                    ARoadSegment const& rs = get<ARoadSegment const&>(*it);
                    if ( std::next(it) == path.entities.end() ){
                        timed_path.push_back({rs.end_cross, succ.time});
                    }else{
                        double l = rs.length;
                        double s = map.roadsegment(rs.roadsegment_index).properties.get<double>("SPEED");
                        currentMs += l / s * 1000 * factor;
                        timed_path.push_back({rs.end_cross, prev.time + b::posix_time::millisec(currentMs)});
                    }                
                }
            }
        }
    }
    timed_path.resize(b::unique<b::return_begin_found>(timed_path).size());
    return timed_path;
}

vector<TimedCrossIndex> load_timed_path_from_file(string const& file){
    ifstream ifs(file);
    vector<TimedCrossIndex> timed_path;
    if ( ifs ){
        ifs.imbue(std::locale(ifs.getloc(), new boost::posix_time::time_input_facet("%Y-%m-%d %H:%M:%S%F")));
        int id;
        ptime time;
        while ( ifs >> id && ifs.ignore() && ifs >> time ){
            timed_path.push_back({id, time});
        }
    }
    return timed_path;
}


vector<TimedCrossIndex> load_timed_path_from_DB(mysqlpp::Connection& con, int metaID)
{
    vector<TimedCrossIndex> traj;
    try
    {
        mysqlpp::Query q = con.query("SELECT `cross_id`, `time` FROM traj_data WHERE `metadata_id` = %0");
        q.parse();
        mysqlpp::StoreQueryResult stored_result = q.store(metaID);
        string time_str;
        for(mysqlpp::Row const& r : stored_result)
        {
            int cid = r[0];
            r[1].to_string(time_str);
            traj.push_back({cid, boost::posix_time::time_from_string(time_str)});
        }
    }catch(exception const& e)
    {
        cerr << "error in " << __FUNCTION__ <<":" << e.what() << endl;
        throw;
    }
    return traj;
}
