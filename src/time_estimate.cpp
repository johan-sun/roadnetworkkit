#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/algorithm.hpp>
#include "time_estimate.h"
using namespace std;

struct TimeEstimater : public boost::static_visitor<double>{
    TimeEstimater(RoadMap const& map):map(map){}
    RoadMap const& map;
    double operator()(ARoadSegment const& rs)const{
        double speed = map.roadsegment(rs.roadsegmentIndex).properties.get<double>("SPEED");
        return rs.length / speed;
    }
    double operator()(PartOfRoad const& pr)const{
        double speed = map.roadsegment(pr.roadsegmentIndex).properties.get<double>("SPEED");
        return pr.length / speed;
    }
    double operator()(ProjectPoint const&)const{
        return 0.0;
    }
};
using b::posix_time::ptime;
vector<TimedCrossIndex> estimateTime(
    vector<GpsPoint> const& log,
    vector<Path> const& paths, 
    pair<int, int> const& logRange,
    RoadMap const& map){

    vector<TimedCrossIndex> timedPath;
    for(int i = logRange.first; i + 1 < logRange.second; ++i){
        GpsPoint const& prev = log[i];
        GpsPoint const& succ = log[i+1];
        Path const& path = paths[i];
        int esz = path.entities.size();
        if ( path.entities.front().type() == typeid(ProjectPoint) ){
            assert(esz == 1);
            ProjectPoint const& pp = get<ProjectPoint const&>(path.entities.front());
            if ( pp.type == ProjectPoint::OnCross ){
                timedPath.push_back({ pp.index, prev.time + (succ.time - prev.time) });
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
                    timedPath.push_back({pr.start.index, prev.time});
                }else if ( pr.end.type == ProjectPoint::OnCross ){
                    assert(pr.start.type == ProjectPoint::OnRoad);
                    if ( esz == 1 ) timedPath.push_back({pr.end.index, succ.time});
                    else{
                        double speed = map.roadsegment(pr.roadsegmentIndex).properties.get<double>("SPEED");
                        currentMs += pr.length / speed * 1000 * factor;
                        timedPath.push_back({pr.end.index, prev.time + b::posix_time::millisec(currentMs)});
                    }
                }else{
                    assert(esz == 1);
                }
            }else{
                assert(path.entities.front().type() == typeid(ARoadSegment));
                ARoadSegment const& rs = get<ARoadSegment const&>(path.entities.front());
                timedPath.push_back({rs.startCross, prev.time});
                if ( esz == 1){
                    timedPath.push_back({rs.endCross, succ.time});
                }else{
                    currentMs = rs.length / map.roadsegment(rs.roadsegmentIndex).properties.get<double>("SPEED") * 1000 * factor;
                    timedPath.push_back({rs.endCross, prev.time + b::posix_time::millisec(currentMs)});
                }
            }

            for(auto it = std::next(path.entities.begin()); it != path.entities.end(); ++it){
                if ( it->type() == typeid(ARoadSegment) ){
                    ARoadSegment const& rs = get<ARoadSegment const&>(*it);
                    if ( std::next(it) == path.entities.end() ){
                        timedPath.push_back({rs.endCross, succ.time});
                    }else{
                        double l = rs.length;
                        double s = map.roadsegment(rs.roadsegmentIndex).properties.get<double>("SPEED");
                        currentMs += l / s * 1000 * factor;
                        timedPath.push_back({rs.endCross, prev.time + b::posix_time::millisec(currentMs)});
                    }                
                }
            }
        }
    }
    timedPath.resize(b::unique<b::return_begin_found>(timedPath).size());
    return timedPath;
}
