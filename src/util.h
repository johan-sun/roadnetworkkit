#ifndef  UTIL_H
#define  UTIL_H

//auto-include {{{
#include  <boost/geometry.hpp>
#include  <boost/graph/dijkstra_shortest_paths.hpp>
#include  <utility>
#include  <string>
#include  <type_traits>
#include    "geomerty.h"
//}}}

/// \brief 从string读取geometry
template<typename Geometry>
Geometry fromWKT(std::string const& wkt){
    Geometry g;
    boost::geometry::read_wkt(wkt, g);
    return g;
}

Box range_box(Point const& center, double r){
    Point a, b;
    a.x(center.x() - r);
    a.y(center.y() - r);
    b.x(center.x() + r);
    b.y(center.y() + r);
    return { a, b };
}
#endif  /*UTIL_H*/
