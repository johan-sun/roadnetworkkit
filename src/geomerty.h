#ifndef  GEOMERTY_H
#define  GEOMERTY_H
#include  <boost/geometry.hpp>
#include  <boost/geometry/geometries/register/point.hpp>
#include  <boost/geometry/index/rtree.hpp>
#include  <boost/geometry/geometries/point_xy.hpp>
#include  <boost/geometry/geometries/segment.hpp>
#include  <boost/geometry/geometries/linestring.hpp>
#include  <boost/geometry/geometries/box.hpp>
#include  <boost/geometry/geometries/adapted/c_array.hpp>
namespace b = boost;
namespace bg = boost::geometry;

BOOST_GEOMETRY_REGISTER_C_ARRAY_CS( ::bg::cs::cartesian );

/// geometry point
typedef bg::model::d2::point_xy<double> Point;

/// geometry segment
typedef bg::model::segment<Point> Segment;

/// geometry linestring
typedef bg::model::linestring<Point> Linestring;

/// geometry box
typedef bg::model::box<Point> Box;

#endif  /*GEOMERTY_H*/
