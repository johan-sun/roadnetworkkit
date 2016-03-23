#ifndef  UTIL_H
#define  UTIL_H
//auto-include {{{
#include  <boost/geometry.hpp>
#include  <utility>
#include  <string>
#include  <type_traits>
#include    "geomerty.h"
#include "sutil/boost/geometry/distance_projected_point_return_point.hpp"
//}}}

/// \brief 从string读取geometry
template<typename Geometry>
Geometry from_wkt(std::string const& wkt){
    Geometry g;
    boost::geometry::read_wkt(wkt, g);
    return g;
}

inline Box range_box(Point const& center, double r){
    Point a, b;
    a.x(center.x() - r);
    a.y(center.y() - r);
    b.x(center.x() + r);
    b.y(center.y() + r);
    return { a, b };
}
template<typename P, typename T>
Point project_point(P c, T t){
    Point prj;
    bg::convert(bg::distance(c, t, bg::strategy::distance::projected_point_return_point<>()).to_pair().second, prj);
    return prj;
}
#endif  /*UTIL_H*/
