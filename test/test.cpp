#define BOOST_TEST_MODULE roadnetworkkit
//auto-include{{{
#include  <boost/test/unit_test.hpp>

// there seems a bug for gcc 4.8 about adl lookup
//put the head before the alg use put get will ok
#include  <boost/graph/dijkstra_shortest_paths.hpp>
#include  <boost/graph/breadth_first_search.hpp>
#include  <boost/graph/properties.hpp>
#include  <boost/timer.hpp>
#include <set>
#include <limits>
#include  <map>
#include <unordered_map>
#include  <iostream>
#include    "roadmap.h"
#include    "bj-road-epsg3785/bj_road_epsg3785.h"
#include    "util.h"
#include    "map_graph_property.hpp"
#include    "sutil/boost/geometry/distance_projected_point_return_point.hpp"
#include    "sutil/boost/graph/dijkstra_shortest_paths_to_dest.hpp"
#include    "sutil/boost/graph/visitor_adaptor.hpp"

//}}}
using namespace std;


BOOST_AUTO_TEST_SUITE(roadnetworkkit)
    Map map;

    BOOST_AUTO_TEST_CASE(basic_use) {
        BOOST_REQUIRE(map.load("../data/map/bj-road-epsg3785", BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker()));
        map.mapCrossProperty<string>("ID");
        map.mapRoadsegmentProperty<string>("ID");
        BOOST_CHECK(map.containCross("ID", "60575200005"));
        BOOST_CHECK(map.containRoadSegment("ID", "60575200004"));
        BOOST_CHECK(bg::equals(map.cross("ID", "59566338038").geometry,
                fromWKT<Point>("POINT(12962877.13440558314323425 4847075.07403475046157837)")));
        BOOST_CHECK(bg::equals(map.roadsegment("ID", "59566308618").geometry, fromWKT<Linestring>("LINESTRING(12963151.44789479300379753 4846751.49003899563103914, 12963139.62576487474143505 4846684.77474731393158436, 12963134.49616274051368237 4846654.46518910489976406, 12963129.40663561969995499 4846627.62786234728991985, 12963122.59388278238475323 4846432.13265211507678032)")));
        map.buildGraph();
    }

    BOOST_AUTO_TEST_CASE(basic_query) {
        Point c = fromWKT<Point>("POINT(12979268.69788035191595554 4861101.51186775788664818)");
        Box range100 = range_box(c, 100);
        Box range150 = range_box(c, 150);
        int correctResult1[] = {85529};
        int correctResult2[] = {85528, 85529};
        set<int> result;
        for (auto &p : map.crossIndex | bgi::adaptors::queried(bgi::within(range100))) {
            result.insert(p.second);
        }
        BOOST_CHECK_EQUAL_COLLECTIONS(begin(result), end(result), begin(correctResult1), end(correctResult1));
        result.clear();
        for (auto &p : map.crossIndex | bgi::adaptors::queried(bgi::within(range150))) {
            result.insert(p.second);
        }
        BOOST_CHECK_EQUAL_COLLECTIONS(begin(result), end(result), begin(correctResult2), end(correctResult2));
    }

    BOOST_AUTO_TEST_CASE(property_map_from_roadsegment) {


        PropertyMapFromRoadProperty<string> idMap(map, "ID");

        auto edge = *b::edges(map.graph).first;
        BOOST_CHECK_EQUAL(map.roadsegment(edge).index, 0);
        string id = map.roadsegment(edge).properties.get<string>("ID");
        BOOST_CHECK_EQUAL(get(idMap, edge), id);

        PropertyMapGen<double> weigthMap(map, [](Map::GraphTraits::edge_descriptor const &e, RoadSegment const &r) {
            return bg::length(r.geometry);
        });
        BOOST_CHECK_CLOSE(get(weigthMap, edge), bg::length(map.roadsegment(0).geometry), 1e-6);
    }

    BOOST_AUTO_TEST_CASE(project_point) {
        Point p1{0, 0};
        Point p2{2, 0};
        Point c{1, 2};
        Point correct{1, 0};
        Segment seg(p1, p2);
        double dis;
        Point prj;
        tie(dis, prj) = bg::distance(c, seg, bg::strategy::distance::projected_point_return_point<>()).pair;
        BOOST_CHECK_EQUAL(dis, bg::distance(c, prj));
        BOOST_CHECK_EQUAL(dis, 2);
        BOOST_CHECK(bg::equals(prj, correct));

        Linestring line;
        Point points[] = {
                {0, 0},
                {1, 0},
                {1, 1},
                {2, 1}
        };
        bg::append(line, points);

        c = {0, 2};
        correct = {1, 1};
        tie(dis, prj) = bg::distance(c, line, bg::strategy::distance::projected_point_return_point<>()).pair;
        BOOST_CHECK_CLOSE(dis, bg::distance(prj, c), 1e-6);
        BOOST_CHECK_CLOSE(dis, std::sqrt(2.0), 1e-6);
        BOOST_CHECK(bg::equals(correct, prj));

        Point poly[] = {
                {0, 0},
                {1, 1},
                {2, 1},
                {2, 2},
                {2, 3},
                {-2, 3},
                {-2, 1},
                {-1, 1},
                {0, 0}
        };
        bg::model::polygon <Point> polygon;
        bg::append(polygon, poly);

        correct = {0, 0};
        c = {0, -2};
        tie(dis, prj) = bg::distance(c, polygon, bg::strategy::distance::projected_point_return_point<>()).pair;
        BOOST_CHECK_EQUAL(2, bg::distance(c, polygon));
        BOOST_CHECK_EQUAL(2, dis);
        BOOST_CHECK(bg::equals(correct, prj));


        tie(dis, prj) = bg::comparable_distance(c, polygon, bg::strategy::distance::projected_point_return_point<>()).pair;
        BOOST_CHECK_EQUAL(4, dis);
        BOOST_CHECK(bg::equals(correct, prj));
    }

    BOOST_AUTO_TEST_CASE( lazy_map_test){
        double inf = numeric_limits<double>::max();
        typedef Map::Graph::vertex_descriptor  Vertex;
        unordered_map<Map::Graph::vertex_descriptor , double> d;
        b::lazy_associative_property_map<unordered_map<Map::Graph::vertex_descriptor , double > >
                distanceMap(d, inf);
        BOOST_CHECK_EQUAL(get(distanceMap, Vertex(0)), inf);
        put(distanceMap, Vertex(0), 1.0);
        BOOST_CHECK_EQUAL(get(distanceMap, Vertex(0)), 1.0);
    }

    BOOST_AUTO_TEST_CASE(shortest) {
        map.visitRoadSegment([](RoadSegment &r) {
            r.properties.put<double>("LENGTH", bg::length(r.geometry));
        });
        PropertyMapFromRoadProperty<double> edgeWeightMap(map, "LENGTH");

        typedef Map::GraphTraits::vertex_descriptor Vertex;
        Vertex start = 73184, end = 73050;

        vector<Vertex> pre(b::num_vertices(map.graph));
        vector<double> d(b::num_vertices(map.graph));

        b::dijkstra_shortest_paths(map.graph, start,
                b::weight_map(edgeWeightMap).distance_map(&d[0]).predecessor_map(&pre[0])
        );
        cout << d[end] << endl;

        unordered_map<Vertex , double> ld;

        double inf = numeric_limits<double>::max();
        b::lazy_associative_property_map<std::unordered_map<Vertex ,double> > distanceMap(ld, inf);
        b::timer timer;
        b::dijkstra_shortest_paths(map.graph, start, b::weight_map(edgeWeightMap).distance_map(distanceMap));
        cout << timer.elapsed() << endl;
        ld.clear();
        timer.restart();
        b::dijkstra_shortest_paths_to_dest(map.graph, start, end,
                b::distance_map(distanceMap).
                weight_map(edgeWeightMap));
        cout << timer.elapsed() << endl;
    }


BOOST_AUTO_TEST_SUITE_END()

#include<boost/test/included/unit_test.hpp>
