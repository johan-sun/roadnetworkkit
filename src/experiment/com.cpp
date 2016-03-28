//auto-include {{{
#include    "experiment/com.h"
#include  <roadmap.h>
//}}}
using namespace std;
using namespace boost::posix_time;
VirtualEdgeWeight estimation_cost(RoadMap const& map, Path path)
{
    double totalMs = 0;
    for(auto& e : path.entities)
    {
        totalMs += b::apply_visitor(TimeCal(map), e);
    }
    return {totalMs};
}

void weight_time_interpolation(RoadMap const& map, Path const& path, 
        time_duration const& start, time_duration const& end, vector<TimedCrossInDay>& out)
{
    double estMs = 0;
    for(auto& e : path.entities)
    {
        estMs += b::apply_visitor(TimeCal(map), e);
    }
    double actualMs = (end - start).total_milliseconds();
    double k = actualMs / estMs;
    out.back().time_of_day = start;
    for(auto& e : path.entities)
    {
        estMs = b::apply_visitor(TimeCal(map), e);
        ARoadSegment const& ars = b::get<ARoadSegment>(e);
        TimedCrossInDay tc;
        tc.cross_id = ars.end_cross;
        tc.time_of_day = out.back().time_of_day + milliseconds(estMs * k);
        out.push_back(tc);
    }
}

#define DEG(degree) (degree / 180.0 * M_PI)
static double abs(Point const& a)
{
    return sqrt(a.x() * a.x() + a.y() * a.y());
}

int direction(Point const& a, Point const& b)
{
    double cos_ = bg::dot_product(a, b) / (abs(a) * abs(b));
    double angle = acos(cos_);
    if ( angle < DEG(30) )
        return 0;//直线
    double d = a.x() * b.y() - a.y() * b.x();
    if( d < 0 ) return 1;//右转
    return -1;//左转
}
