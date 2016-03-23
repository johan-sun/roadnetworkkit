#ifndef  GPS_H
#define  GPS_H
//auto-include {{{
#include <string>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include "geomerty.h"
//}}}
///\brief GpsPoint
struct GpsPoint{
    GpsPoint()=default;
    GpsPoint(double x, double y, boost::posix_time::ptime time):geometry(x,y),time(time){}
    Point geometry;
    boost::posix_time::ptime time;
};

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(
    GpsPoint, 
    double, 
    boost::geometry::cs::cartesian, 
    geometry.x , 
    geometry.y, 
    geometry.x , 
    geometry.y
);

BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<GpsPoint>);

std::vector<GpsPoint> load_from_file(std::string const&);

#endif  /*GPS_H*/

