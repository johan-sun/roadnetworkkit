#ifndef  COM_H
#define  COM_H
//auto-include {{{
#include  <boost/variant.hpp>
#include <boost/date_time.hpp>
#include    "path-restore/highlevelmap.h"
#include    "roadmap.h"
//}}}
struct TimeCal : public boost::static_visitor<double>
{
    RoadMap const& m;
    TimeCal(RoadMap const& map):m(map){}
    double operator()(ARoadSegment const& ars)const
    {
        RoadSegment const& rs = m.roadsegment(ars.roadsegment_index);
        double speed = rs.properties.get<double>("SPEED");
        return ars.length / speed * 1000;
    }
    double operator()(ProjectPoint const& )const{ return 0;}
    double operator()(PartOfRoad const& pr)const
    {
        RoadSegment const& rs = m.roadsegment( pr.roadsegment_index );
        double s = rs.properties.get<double>("SPEED");
        return pr.length /  s * 1000;
    }
};

VirtualEdgeWeight estimation_cost(RoadMap const& map, Path path);

struct TimedCrossInDay
{
    int cross_id;
    boost::posix_time::time_duration time_of_day;
};

void weight_time_interpolation(RoadMap const& map, Path const& path, 
        boost::posix_time::time_duration const& start, boost::posix_time::time_duration const& end, std::vector<TimedCrossInDay>& out);
int direction(Point const& a, Point const& b);
#endif  /*COM_H*/
