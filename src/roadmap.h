#include "bj-road-epsg3785/bj_road_epsg3785.h"
#ifndef ROADMAP_H
#define ROADMAP_H
#include <vector>
#include <list>
#include <boost/variant.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include "generalmap.h"
#include "map_graph_property.hpp"
#include "geomerty.h"

struct ProjectPoint{
    enum Type{
        OnCross,
        OnRoad
    };
    Point geometry;
    Type type;
    int index;
    double param;
};

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(ProjectPoint, double, bg::cs::cartesian, geometry.x, geometry.y, geometry.x, geometry.y);

ProjectPoint makeProjectPoint(Point const& point, RoadSegment const& r);
inline ProjectPoint makeProjectPointForCross(Cross const& c){
    ProjectPoint p;
    p.type = ProjectPoint::OnCross;
    p.geometry = c.geometry;
    p.index = c.index;
    p.param = 0;
    return p;
}

template<typename P>
ProjectPoint makeProjectPoint(P const& point, RoadSegment const& r){
    Point p;
    bg::convert(point, p);
    return makeProjectPoint(p, r);
}

struct PartOfRoad{
    double length;
    ProjectPoint start;
    ProjectPoint end;
    int roadsegmentIndex;
};

PartOfRoad makePartOfRoad(Point const& start, Point const& end, RoadSegment const& r);
template<typename P1, typename P2>
PartOfRoad makePartOfRoad(P1 const& start, P2 const& end, RoadSegment const& r){
    Point s;
    Point e;
    bg::convert(start, s);
    bg::convert(end, e);
    return makePartOfRoad(s, e, r);
}
Linestring geometry(PartOfRoad const& pr, RoadSegment const& r);

struct ARoadSegment{
    int startCross;
    int endCross;
    int roadsegmentIndex;
    double length;
};

struct Path{
    std::list<boost::variant<ARoadSegment,PartOfRoad, ProjectPoint> > entities;
    double totalLength()const;
    inline bool valid()const{
        return !entities.empty();
    }
};

class RoadMap : public Map{
public:
    enum ShortestPathStrategy{
        Dijkstra,
        AStar
    };

    RoadMap():edgeWeightMap(*this, "GEO_LENGTH"),shortestPathStrategy(AStar){}

    template<typename Picker = BJRoadEpsg3785IDPicker, typename Checker = BJRoadEpsg3785CrossIDChecker>
    bool load(std::string const& shpFile, Picker picker = Picker(), Checker checker = Checker()){
        bool succ = Map::load(shpFile, picker, checker);
        if ( ! succ ){
            return false;
        }

        buildGraph();
        visitRoadSegment([](RoadSegment& road){
            double d = bg::length(road.geometry);
            road.properties.put("GEO_LENGTH", d);
        });
        return true;
    }

    ///\brief 范围查询路口
    ///\ret 返回路口索引按照距离查询点距离大小排序
    std::vector<int> queryCross(Point const& p, double radious)const;

    ///\brief 范围查询道路
    ///\ret 返回道路索引按照距离查询点距离大小排序
    std::vector<int> queryRoad(Point const& p, double radious)const;

    inline Path shortestPath(int crossA, int crossB)const{
        if ( shortestPathStrategy == Dijkstra ){
            return shortestPathDijkstra(crossA, crossB);
        }else{
            return shortestPathAStar(crossA, crossB);
        }
    }
    Path shortestPath(ProjectPoint const& start, ProjectPoint const& end)const;
    Path shortestPath(ProjectPoint const& start, int end)const;
    Path shortestPath(int start, ProjectPoint const& end)const;

    Path shortestPathAStar(int crossA, int crossB)const;
    Path shortestPathDijkstra(int crossA, int crossB)const;
    Path shortestPathBGLDijkstra(int crossA, int crossB)const;
    ProjectPoint nearestProject(Point const& p)const;

    PropertyMapFromRoadProperty<double> edgeWeightMap;
    ShortestPathStrategy shortestPathStrategy;
};

Linestring geometry(Path const& path, RoadMap const& m);
#endif /* ROADMAP_H */
