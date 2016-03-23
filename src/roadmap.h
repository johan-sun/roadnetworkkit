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

ProjectPoint make_project_point(Point const& point, RoadSegment const& r);
inline ProjectPoint make_project_point_for_cross(Cross const& c){
    ProjectPoint p;
    p.type = ProjectPoint::OnCross;
    p.geometry = c.geometry;
    p.index = c.index;
    p.param = 0;
    return p;
}

template<typename P>
ProjectPoint make_project_point(P const& point, RoadSegment const& r){
    Point p;
    bg::convert(point, p);
    return make_project_point(p, r);
}

struct PartOfRoad{
    double length;
    ProjectPoint start;
    ProjectPoint end;
    int roadsegment_index;
};

PartOfRoad make_part_of_road(Point const& start, Point const& end, RoadSegment const& r);
template<typename P1, typename P2>
PartOfRoad make_part_of_road(P1 const& start, P2 const& end, RoadSegment const& r){
    Point s;
    Point e;
    bg::convert(start, s);
    bg::convert(end, e);
    return make_part_of_road(s, e, r);
}

Linestring geometry(PartOfRoad const& pr, RoadSegment const& r);

struct ARoadSegment{
    int start_cross;
    int end_cross;
    int roadsegment_index;
    double length;
};

struct Path{
    std::list<boost::variant<ARoadSegment,PartOfRoad, ProjectPoint> > entities;
    double total_length()const;
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

    RoadMap():edge_weight_map(*this, "GEO_LENGTH"),shortest_path_strategy(AStar){}

    template<typename Picker = BJRoadEpsg3785IDPicker, typename Checker = BJRoadEpsg3785CrossIDChecker>
    bool load(std::string const& shpFile, Picker picker = Picker(), Checker checker = Checker()){
        bool succ = Map::load(shpFile, picker, checker);
        if ( ! succ ){
            return false;
        }

        build_graph();
        visit_roadsegment([](RoadSegment& road){
            double d = bg::length(road.geometry);
            road.properties.put("GEO_LENGTH", d);
            road.properties.put("ID", road.index);
        });
        visit_cross([](Cross & c){
            c.properties.put("ID", c.index);
        });
        map_cross_property<int>("ID");
        map_cross_property<std::string>("SID");
        map_roadsegment_property<int>("ID");
        map_roadsegment_property<std::string>("SID");
        return true;
    }

    ///\brief 范围查询路口
    ///\ret 返回路口索引按照距离查询点距离大小排序
    std::vector<int> query_cross(Point const& p, double radious)const;

    ///\brief 范围查询道路
    ///\ret 返回道路索引按照距离查询点距离大小排序
    std::vector<int> query_road(Point const& p, double radious)const;

    inline Path shortest_path(int crossA, int crossB)const{
        if ( shortest_path_strategy == Dijkstra ){
            return shortest_path_Dijkstra(crossA, crossB);
        }else{
            return shortest_path_Astar(crossA, crossB);
        }
    }
    Path shortest_path(ProjectPoint const& start, ProjectPoint const& end)const;
    Path shortest_path(ProjectPoint const& start, int end)const;
    Path shortest_path(int start, ProjectPoint const& end)const;

    Path shortest_path_Astar(int crossA, int crossB)const;
    Path shortest_path_Dijkstra(int crossA, int crossB)const;
    ProjectPoint nearest_project(Point const& p)const;

    PropertyMapFromRoadProperty<double> edge_weight_map;
    ShortestPathStrategy shortest_path_strategy;
};

Linestring geometry(Path const& path, RoadMap const& m);
#endif /* ROADMAP_H */
