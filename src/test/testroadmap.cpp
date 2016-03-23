
#define BOOST_TEST_MODULE roadmaptest
//auto-include {{{
#include <boost/test/unit_test.hpp>
#include <shapefil.h>
#include <boost/range/algorithm.hpp>
#include <vector>
#include <boost/timer.hpp>
#include "roadmap.h"
#include "bj-road-epsg3785/bj_road_epsg3785.h"
#include "gps.h"
//}}}

using namespace std;


BOOST_AUTO_TEST_SUITE(roadmap_test)

    RoadMap bjRoad;

    void drawPath ( char const* out, Path const& path){
        SHPHandle shp = SHPCreate(out, SHPT_ARC);
        DBFHandle dbf = DBFCreate(out);
        DBFAddField(dbf, "LENGTH", FTDouble, 10, 3);
        Linestring line = geometry(path, bjRoad);
        vector<double> x;
        vector<double> y;
        b::transform(line, back_inserter(x), [](Point const& p){ return p.x(); });
        b::transform(line, back_inserter(y), [](Point const& p){ return p.y(); });
        SHPObject* l = SHPCreateSimpleObject(SHPT_ARC, line.size(), &x[0], &y[0], nullptr);
        int id = SHPWriteObject(shp, -1, l);
        DBFWriteDoubleAttribute(dbf, id , 0 , path.total_length());
        SHPDestroyObject(l);
        SHPClose(shp);
        DBFClose(dbf);
    }
    void drawLinestring(char const * out, Linestring const& line){
        SHPHandle shp = SHPCreate(out, SHPT_ARC);
        DBFHandle dbf = DBFCreate(out);
        DBFAddField(dbf, "LENGTH", FTDouble, 10, 3);
        vector<double> x;
        vector<double> y;
        b::transform(line, back_inserter(x), [](Point const& p){ return p.x(); });
        b::transform(line, back_inserter(y), [](Point const& p){ return p.y(); });
        SHPObject* l = SHPCreateSimpleObject(SHPT_ARC, line.size(), &x[0], &y[0], nullptr);
        int id = SHPWriteObject(shp, -1, l);
        DBFWriteDoubleAttribute(dbf, id , 0 , bg::length(line));
        SHPDestroyObject(l);
        SHPClose(shp);
        DBFClose(dbf);
    }

    BOOST_AUTO_TEST_CASE(basic_roadmap){
        BOOST_REQUIRE( bjRoad.load("../data/map/bj-road-epsg3785", BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker()));
        vector<GpsPoint> log = load_from_file("../data/bj-gps-after-prepare/20121101/96894/96894-0.txt");
        RoadSegment const& r = bjRoad.roadsegment(83294);
        ProjectPoint p = make_project_point(log[0], r);
        PartOfRoad pr1 = make_part_of_road(p, r.geometry.front(), r);
        PartOfRoad pr2 = make_part_of_road(p, r.geometry.back(), r);
        Linestring line1 = geometry(pr1, r);
        Linestring line2 = geometry(pr2, r);
        vector<double> x;
        vector<double> y;
        SHPHandle shp = SHPCreate("line", SHPT_ARC);
        DBFHandle dbf = DBFCreate("line");
        DBFAddField(dbf, "type", FTString, 10, 0);
        boost::transform(line1, back_inserter(x), [](Point const&p){ return p.x(); });
        boost::transform(line1, back_inserter(y), [](Point const&p){ return p.y(); });
        SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, line1.size(), &x[0], &y[0], nullptr);
        int id = SHPWriteObject(shp, -1, line);
        DBFWriteStringAttribute(dbf, id, 0, "line1");
        SHPDestroyObject(line);
        x.clear();
        y.clear();
        boost::transform(line2, back_inserter(x), [](Point const&p){ return p.x(); });
        boost::transform(line2, back_inserter(y), [](Point const&p){ return p.y(); });
        line = SHPCreateSimpleObject(SHPT_ARC, line2.size(), &x[0], &y[0], nullptr);
        id = SHPWriteObject(shp, -1, line);
        DBFWriteStringAttribute(dbf, id, 0, "line2");
        SHPDestroyObject(line);
        SHPClose(shp);
        DBFClose(dbf);
    }
    vector<Point> srcPoints{
            {12946444.3, 4848673.4},
            {12946484.83, 4848728.42},
            {12946561.985, 4848818.485}
    };

    BOOST_AUTO_TEST_CASE(neariest_project){
        SHPHandle  shp = SHPCreate("src_point", SHPT_POINT);
        DBFHandle  dbf = DBFCreate("src_point");
        SHPHandle shp2 = SHPCreate("project_point", SHPT_POINT);
        DBFHandle dbf2 = DBFCreate("project_point");
        DBFAddField(dbf, "desc", FTString, 20, 0);
        DBFAddField(dbf2, "desc", FTString, 20, 0);
        for(auto& p : srcPoints){
            double x;
            double y;
            x = p.x();
            y = p.y();
            ProjectPoint pp = bjRoad.nearest_project(p);
            SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
            DBFWriteStringAttribute(dbf,SHPWriteObject(shp, -1, point), 0, "...");
            SHPDestroyObject(point);
            x = pp.geometry.x();
            y = pp.geometry.y();
            point = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
            DBFWriteStringAttribute(dbf2,SHPWriteObject(shp2, -1, point), 0, "...");
            SHPDestroyObject(point);
        }
        DBFClose(dbf);
        SHPClose(shp);
        DBFClose(dbf2);
        SHPClose(shp2);
    }

    BOOST_AUTO_TEST_CASE(shortest_path){
    /*
        boost::timer astar, dijkstra, bglDijstra;
        int a = 63151, b = 62738;
        Path bglPath, astarPath, dijkstraPath;
        bglDijstra.restart();
        bglPath = bjRoad.shortestPathBGLDijkstra(a, b);
        cout << "BGL " << bglDijstra.elapsed() << "s"<<endl;
        astar.restart();
        astarPath = bjRoad.shortestPathAStar(a, b);
        cout << "AStar " << astar.elapsed() << "s"<<endl;
        dijkstra.restart();
        dijkstraPath = bjRoad.shortestPathDijkstra(a, b);
        cout << "dijkstra " << dijkstra.elapsed() << "s" << endl;
        cout << bglPath.total_length() << " " << astarPath.total_length() << " " << dijkstraPath.total_length() << endl;*/
        /*
        double x1,y1;
        double x2,y2;
        while ( cin >> x1 >> y1 >> x2 >> y2 ){
            ProjectPoint p1 = bjRoad.nearest_project({x1, y1});
            ProjectPoint p2 = bjRoad.nearest_project({x2, y2});
            Path path = bjRoad.shortest_path(p1, p2);
            drawPath("SHORTEST_PATH", path);
            cout << path.total_length() << endl;
        }*/
    }

BOOST_AUTO_TEST_SUITE_END()
#include<boost/test/included/unit_test.hpp>
