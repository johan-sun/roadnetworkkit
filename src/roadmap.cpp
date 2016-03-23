#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/adaptors/query.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/pool/pool.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_set>
#include "geomerty.h"
#include "roadmap.h"
#include "util.h"
#include "sutil/boost/graph/dijkstra_shortest_paths_to_dest.hpp"
#include "sutil/boost/graph/lazy_associative_property_map.hpp"
#include  <assert.h>

using namespace std;

double Path::total_length()const{
    if (! valid() ) return numeric_limits<double>::max();
    double d = 0.0;
    struct LengthGet : public boost::static_visitor<double> {
        double operator()(ARoadSegment const& rs)const{
            return rs.length;
        }
        double operator()(PartOfRoad const& pr)const{
            return pr.length;
        }
        double operator()(ProjectPoint const&)const{
            return 0.0;
        }
    };

    for(auto& e : entities){
        d += b::apply_visitor(LengthGet(), e);
    }
    return d;
}
vector<int> RoadMap::query_cross(Point const &p, double radious) const {
    Box box = range_box(p, radious);
    vector<int> ret;
    vector<pair<int, double> > candidate;
    double rad2 = radious * radious;
    for (pair<Point, int > const& pi  : cross_rtree | bgi::adaptors::queried(bgi::covered_by(box)) ){
        double d = bg::comparable_distance(pi.first, p);
        if (d <= rad2){
            candidate.push_back({pi.second, d});
        }
    }

    b::sort(candidate, [](pair<int,double> const& a, pair<int, double> const& b){
        return a.second < b.second;
    });

    for(auto& p : candidate){
        ret.push_back(p.first);
    }
    return ret;
}


vector<int> RoadMap::query_road(Point const& p, double radious)const{
    Box box = range_box(p, radious);
    vector<int> ret;
    vector<pair<int, double> > cand;
    set<int> roadIdx;
    double rad2 = radious * radious;
    for( pair<Box, int> const& pi :
            roadsegment_rtree | bgi::adaptors::queried(bgi::covers(box))){
        roadIdx.insert(pi.second);
    }

    for( pair<Box, int> const& pi :
            roadsegment_rtree | bgi::adaptors::queried(bgi::covered_by(box))){
        roadIdx.insert(pi.second);
    }

    for( pair<Box, int> const& pi :
            roadsegment_rtree | bgi::adaptors::queried(bgi::overlaps(box))){
        roadIdx.insert(pi.second);
    }

    for(int i : roadIdx){
        RoadSegment const& r = roadsegment(i);
        double d = bg::comparable_distance(p, r.geometry);
        if ( d <= rad2){
            cand.push_back({i, d});
        }
    }
    b::sort(cand, [](pair<int,double> const&a, pair<int, double> const& b){
        return a.second < b.second;
    });
    for(auto& p : cand){
        ret.push_back(p.first);
    }
    return ret;
}
/*
Path RoadMap::shortestPathBGLDijkstra(int crossA, int crossB)const{
    Path path;
    if ( crossA == crossB ){
        path.entities.push_back(makeProjectPointForCross(cross(crossA)));
        return path;
    }

    unordered_map<GraphTraits::vertex_descriptor, GraphTraits::vertex_descriptor> pre;
    unordered_map<GraphTraits::vertex_descriptor, double> distance;
    b::dijkstra_shortest_paths_to_dest(graph, crossA, crossB, 
            b::predecessor_map(b::make_lazy_property_map(pre, GraphTraits::null_vertex()))
            .distance_map(b::make_lazy_property_map(distance, numeric_limits<double>::max()))
            .weight_map(edge_weight_map));
    vector<GraphTraits::vertex_descriptor> cross_index;
    GraphTraits::vertex_descriptor p = crossB;
    while ( p != GraphTraits::null_vertex() ){
        cross_index.push_back(p);
        p = pre.count(p) == 0 ? GraphTraits::null_vertex() : pre[p];
    }

    b::reverse(cross_index);
    for(int i = 1; i < cross_index.size(); ++i){
        auto findedEdge = b::edge(cross_index[i-1], cross_index[i], graph);
        assert(findedEdge.second);
        ARoadSegment ars;
        ars.start_cross = cross_index[i-1];
        ars.end_cross = cross_index[i];
        RoadSegment const& r = roadsegment(findedEdge.first);
        ars.roadsegment_index = r.index;
        ars.length = r.properties.get<double>("GEO_LENGTH");
        path.entities.push_back(ars);
    }
    return path;
}*/


ProjectPoint make_project_point(Point const& point, RoadSegment const& r){
    ProjectPoint pp;
    pp.geometry = project_point(point, r.geometry);
    if ( bg::equals(pp.geometry, r.geometry.front()) ){
        pp.type = ProjectPoint::OnCross;
        pp.index = r.start_cross_index;
        pp.param = 0.0;
    }else if ( bg::equals(pp.geometry, r.geometry.back()) ){
        pp.type = ProjectPoint::OnCross;
        pp.index = r.end_cross_index;
        pp.param = bg::length(r.geometry);
    }else {
        pp.type = ProjectPoint::OnRoad;
        pp.index = r.index;
        pp.param = 0.0;
        for(int i = 1; i < r.geometry.size(); ++i){
            bg::model::referring_segment<Point const> segment(r.geometry[i-1], r.geometry[i]);
            if ( bg::distance(segment, pp.geometry) < 1e-6){//projected point on segment
                pp.param += bg::distance(segment.first, pp.geometry);
                break;
            }else{
                pp.param += bg::length(segment);
            }
        }
    }
    return pp;
}

PartOfRoad make_part_of_road(Point const& start, Point const& end, RoadSegment const& r){
    PartOfRoad pr;
    pr.start = make_project_point(start, r);
    pr.end = make_project_point(end, r);
    pr.roadsegment_index = r.index;
    pr.length = abs(pr.start.param - pr.end.param);
    return pr;
}

Linestring geometry(PartOfRoad const& pr, RoadSegment const& r){
    Linestring line;
    double pS = pr.start.param;
    double pE = pr.end.param;
    Point p1 = pr.start.geometry;
    Point p2 = pr.end.geometry;
    bool doSwap = false;
    if ( pE < pS ){
        swap(pS,pE);
        swap(p1, p2);
        doSwap = true;
    }
    bool incP1 = false;
    for(size_t i = 1; i < r.geometry.size(); ++i){
        bg::model::referring_segment<Point const> seg(r.geometry[i-1], r.geometry[i]);
        if ( !incP1 && bg::comparable_distance(p1, seg) < 1e-6){
            line.push_back(p1);
            incP1 = true;
            if ( bg::comparable_distance(p2, seg) < 1e-6){
                line.push_back(p2);
                break;
            }

            if ( !bg::equals(p1, seg.second) ){
                line.push_back(seg.second);
            }
        }

        if ( incP1 ){
            if ( bg::comparable_distance(p2, seg) > 1e-6){
                line.push_back(seg.second);
            }else{
                if ( ! bg::equals(line.back(), p2) ){
                    line.push_back(p2);
                }
                break;
            }
        }
    }
    if ( doSwap ){
        bg::reverse(line);
    }
    return line;
}



struct GeometryAppend : public boost::static_visitor<void> {

    GeometryAppend(RoadMap const& m, Linestring & l):map(m),line(l){}

    void operator()(ProjectPoint  const& p){
        line.push_back(p.geometry);
    }

    void operator()(ARoadSegment const& r){
        RoadSegment const& rs = map.roadsegment(r.roadsegment_index);
        Linestring l = rs.geometry;
        if ( r.start_cross ==  rs.end_cross_index){
            b::reverse(l);
        }
        bg::append(line, l);
    }

    void operator()(PartOfRoad const& pr){
        bg::append(line, geometry(pr, map.roadsegment(pr.roadsegment_index)));
    }

    RoadMap const& map;
    Linestring & line;
};

Linestring geometry(Path const& path, RoadMap const& m){
    Linestring line;
    GeometryAppend appender(m, line);
    for(auto& v : path.entities){
        boost::apply_visitor(appender, v);
    }
    return line;
}

ProjectPoint RoadMap::nearest_project(Point const &p) const {

    int roadIdx = roadsegment_rtree.qbegin(bgi::nearest(p, 1))->second;
    double r = bg::distance(p, roadsegment(roadIdx).geometry);
    vector<int> ret = query_road(p, r);
    int min = ret.empty() ? roadIdx: ret[0];
    return make_project_point(p, roadsegment(min));
}

struct AStarNode{
    double g;
    double h;
    int cross;
    AStarNode(int cross, double g, double h):g(g),h(h),cross(cross){}
    bool operator<(AStarNode const& otr)const{
        return (g+h) > (otr.g + otr.h);
    }
};

template<typename HF>
vector<int>  
shortest_path_impl(
    RoadMap const& map,
    vector<int> const& init, 
    vector<double> const& initG,
    unordered_set<int> const& finishSet,
    HF hf){

    vector<int>  cross_index;
    typedef b::heap::fibonacci_heap<AStarNode> Heap;
    Heap open;
    unordered_set<RoadMap::GraphTraits::vertex_descriptor> close;
    unordered_map<RoadMap::GraphTraits::vertex_descriptor, RoadMap::GraphTraits::vertex_descriptor> pre;
    unordered_map<RoadMap::GraphTraits::vertex_descriptor, Heap::handle_type> savedHandle;
    for(int i = 0; i < init.size(); ++i){
        savedHandle[init[i]] = open.push({init[i], initG[i], 0.0});
        pre[init[i]] = RoadMap::GraphTraits::null_vertex();
    }
    while(!open.empty()){
        AStarNode top = open.top();
        open.pop();
        if ( finishSet.count(top.cross)){
            RoadMap::GraphTraits::vertex_descriptor p = top.cross;
            while ( p != RoadMap::GraphTraits::null_vertex() ){
                cross_index.push_back(p);
                p = pre[p];
            }

            b::reverse(cross_index);
            return cross_index;
        }

        close.insert(top.cross);
        for(auto iterPair = b::out_edges(top.cross, map.graph); iterPair.first != iterPair.second; ++iterPair.first){
            RoadMap::GraphTraits::edge_descriptor edge = *iterPair.first;
            RoadMap::GraphTraits::vertex_descriptor t = b::target(edge, map.graph);
            if ( close.count(t) ){
                continue;
            }

            double g = get(map.edge_weight_map, edge) + top.g;
            double h = hf(t);
            //double h = bg::distance(cross(t), cross(crossB));
            auto it = savedHandle.find(t);
            if ( it == savedHandle.end()){
                savedHandle[t] = open.push({(int)t,g,h});
                pre[t] = top.cross;
            }else{
                AStarNode& node = *(it->second);
                if ( g + h < node.g + node.h ){
                    AStarNode newNode{(int)t, g, h};
                    pre[t] = top.cross;
                    open.update_lazy(it->second, newNode);
                }
            }
        }
    }

    return cross_index;
}

Path RoadMap::shortest_path_Astar(int crossA, int crossB)const{
    Path path;
    if ( crossA == crossB ){
        path.entities.push_back(make_project_point_for_cross(cross(crossA)));
        return path;
    }
    vector<int> cross_index = 
        shortest_path_impl(*this, {crossA}, {0.0}, {crossB}, 
                [&](int cross){
                    return bg::distance(this->cross(cross), this->cross(crossB));
                });
    for(size_t i = 1; i < cross_index.size(); ++i){
        auto findedEdge = b::edge(cross_index[i-1], cross_index[i], graph);
        assert(findedEdge.second);
        ARoadSegment ars;
        ars.start_cross = cross_index[i-1];
        ars.end_cross = cross_index[i];
        ars.roadsegment_index = get(IndexOfEdge, graph, findedEdge.first);
        ars.length = get(edge_weight_map, findedEdge.first);
        path.entities.push_back(ars);
    }
    return path;
}

Path RoadMap::shortest_path_Dijkstra(int crossA, int crossB)const{
    Path path;
    if ( crossA == crossB ){
        path.entities.push_back(make_project_point_for_cross(cross(crossA)));
        return path;
    }

    vector<int> crossIndex = 
        shortest_path_impl(*this, {crossA}, {0.0}, {crossB}, [](int){ return 0.0; });
    for(size_t i = 1; i < crossIndex.size(); ++i){
        auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
        assert(findedEdge.second);
        ARoadSegment ars;
        ars.start_cross = crossIndex[i-1];
        ars.end_cross = crossIndex[i];
        ars.roadsegment_index = get(IndexOfEdge, graph, findedEdge.first);
        ars.length = get(edge_weight_map, findedEdge.first);
        path.entities.push_back(ars);
    }
    return path;

    return path;
}

static bool startCanGoToEndDirectly(ProjectPoint const& start, ProjectPoint const& end, RoadSegment const& sameRoad){
    if (sameRoad.direction == Direction::Bidirection){
        return true;
    }
    if ( sameRoad.direction == Direction::Forward ){
        return start.param <= end.param;
    }else{
        return start.param >= end.param;
    }
}

static pair<int, int> 
nearbyCrossOnSameRoad(ProjectPoint const& p1, ProjectPoint const& p2, RoadSegment const& r){
    if ( p1.param < p2.param ){
        assert(r.direction == Direction::Backward );
        return { r.start_cross_index, r.end_cross_index };
    }

    //p1.param > p2.param
    assert( r.direction == Direction::Forward );
    return { r.end_cross_index, r.start_cross_index };
}

vector<int> possible_start_nearby_cross(ProjectPoint const&, RoadSegment const& r){
    vector<int> c;
    if ( r.direction & Direction::Forward ){
        c.push_back(r.end_cross_index);
    }

    if ( r.direction & Direction::Backward ){
        c.push_back(r.start_cross_index);
    }

    return c;
}

vector<int> possible_end_nearby_cross(ProjectPoint const&, RoadSegment const& r){
    vector<int> c;
    if ( r.direction & Direction::Forward ){
        c.push_back(r.start_cross_index);
    }

    if ( r.direction & Direction::Backward ){
        c.push_back(r.end_cross_index);
    }

    return c;
}

Path RoadMap::shortest_path(ProjectPoint const& start, ProjectPoint const& end)const{
    if ( bg::equals(start,end) ){
        Path p;
        p.entities.push_back(start);
        return p;
    }

    if ( start.type == ProjectPoint::OnCross && end.type == ProjectPoint::OnCross ){//OnCross OnCross
        return shortest_path(start.index, end.index);
    }else if ( start.type == ProjectPoint::OnRoad && end.type == ProjectPoint::OnRoad ){//OnRoad OnRoad
        if (start.index == end.index){//On Same Road
            RoadSegment const& r = roadsegment(start.index);
            if(startCanGoToEndDirectly(start, end, r)){
                Path p;
                p.entities.push_back(make_part_of_road(start, end, r));
                return p;
            }else{
                //中和为一个点
                /*
                int sc, ec;
                tie(sc, ec) = nearbyCrossOnSameRoad(start, end, r);
                Path rp;
                Path p = shortestPath(sc, ec);
                if ( !p.valid() ){
                    return rp;
                }
                rp.entities.push_back(makePartOfRoad(start, cross(sc), r));//head
                rp.entities.splice(rp.entities.end(), p.entities);//body
                rp.entities.push_back(makePartOfRoad(cross(ec), end, r));//tail
                return rp;*/
                Path rp;
                rp.entities.push_back(end);
                return rp;
            }
        }else{//different road
            RoadSegment const& rs = roadsegment(start.index);
            RoadSegment const& re = roadsegment(end.index);
            Path sPath;/*
            double shortest = numeric_limits<double>::max();
            vector<int> start_possible_cross = possibleStartNearbyCross(start, rs);
            vector<int> end_possible_cross = possible_end_nearby_cross(end, re);
            for(int sc : start_possible_cross){
                for(int ec : end_possible_cross){
                    Path rp;
                    Path bodyPath = shortestPath(sc, ec);//ProjectPoint or ARoadSegments
                    if (bodyPath.valid() ){
                        rp.entities.push_back(makePartOfRoad(start, cross(sc), rs));
                        if ( bodyPath.entities.front().type() == typeid(ARoadSegment))
                            rp.entities.splice(rp.entities.end(), bodyPath.entities);
                        rp.entities.push_back(makePartOfRoad(cross(ec), end, re));
                    }
                    if ( rp.totalLength() < sPath.totalLength() ){
                        sPath = std::move(rp);
                    }
                }
            }*/
            vector<int> start_possible_cross = possible_start_nearby_cross(start, rs);
            vector<int> end_possible_cross = possible_end_nearby_cross(end, re);
            vector<PartOfRoad> part_of_road_s;
            vector<PartOfRoad> part_of_road_e;
            vector<double> initG;
            for(auto v : start_possible_cross){
                part_of_road_s.push_back(make_part_of_road(start ,cross(v), roadsegment(start.index)));
                initG.push_back(part_of_road_s.back().length);
            }
            unordered_map<int, double> endG;
            for(auto v : end_possible_cross){
                part_of_road_e.push_back(make_part_of_road(cross(v), end, roadsegment(end.index)));
                endG[v] = part_of_road_e.back().length;
            }

            vector<int> cross_index = 
                shortest_path_impl(*this, start_possible_cross, initG, {end_possible_cross.begin(), end_possible_cross.end()},
                        [&](int v){
                            if ( endG.count(v) ){
                                return endG[v];
                            }
                            if ( this->shortest_path_strategy == RoadMap::Dijkstra ){
                                return 0.0;
                            }else{
                                return bg::distance(this->cross(v), end);
                            }
                        });
            if ( !cross_index.empty() ){
                for(size_t i = 1; i < cross_index.size(); ++i){
                    auto findedEdge = b::edge(cross_index[i-1], cross_index[i], graph);
                    assert(findedEdge.second);
                    ARoadSegment ars;
                    ars.start_cross = cross_index[i-1];
                    ars.end_cross = cross_index[i];
                    ars.roadsegment_index = get(IndexOfEdge, graph, findedEdge.first);
                    ars.length = get(edge_weight_map, findedEdge.first);
                    sPath.entities.push_back(ars);
                }
                sPath.entities.push_front(part_of_road_s[b::find<b::return_begin_found>(start_possible_cross, cross_index.front()).size()]);
                sPath.entities.push_back(part_of_road_e[b::find<b::return_begin_found>(end_possible_cross, cross_index.back()).size()]);
            }
            return sPath;
        }
    }else if ( start.type == ProjectPoint::OnCross ){//OnCross OnRoad
        return shortest_path(start.index, end);
    }else{//OnRoad OnCross
        return shortest_path(start, end.index);
    }
}


Path RoadMap::shortest_path(ProjectPoint const& start, int end)const{
    if ( start.type == ProjectPoint::OnCross ){
        return shortest_path(start.index, end);
    }else{
        RoadSegment const& rs = roadsegment(start.index);
        vector<int> start_possible_cross = possible_start_nearby_cross(start, rs);
        vector<PartOfRoad> part_of_road_s;
        vector<double> initG;
        for(auto v : start_possible_cross){
            part_of_road_s.push_back(make_part_of_road(start, cross(v), roadsegment(start.index)));
            initG.push_back(part_of_road_s.back().length);
        }
        Path sPath;
        vector<int> cross_index = 
                shortest_path_impl(*this, start_possible_cross, initG, {end},
                        [&](int v){
                            if ( this->shortest_path_strategy == RoadMap::Dijkstra ){
                                return 0.0;
                            }else{
                                return bg::distance(this->cross(v), this->cross(end));
                            }
                        });
        if ( !cross_index.empty() ){
            for(size_t i = 1; i < cross_index.size(); ++i){
                auto findedEdge = b::edge(cross_index[i-1], cross_index[i], graph);
                assert(findedEdge.second);
                ARoadSegment ars;
                ars.start_cross = cross_index[i-1];
                ars.end_cross = cross_index[i];
                ars.roadsegment_index = get(IndexOfEdge, graph, findedEdge.first);
                ars.length = get(edge_weight_map, findedEdge.first);
                sPath.entities.push_back(ars);
            }
            sPath.entities.push_front(part_of_road_s[b::find<b::return_begin_found>(start_possible_cross, cross_index.front()).size()]);
        }
        return sPath;
    }
}

Path RoadMap::shortest_path(int start, ProjectPoint const& end)const{
    if ( end.type == ProjectPoint::OnCross ){
        return shortest_path(start, end.index);
    }else{
        RoadSegment const& re = roadsegment(end.index);
        Path sPath;
        /*
        for(int ec : end_possible_cross){
            Path p;
            Path bodyPath = shortestPath(start, ec);
            if ( bodyPath.valid() ){
                if ( bodyPath.entities.front().type() == typeid(ARoadSegment))
                    p.entities.splice(p.entities.end(), bodyPath.entities);
                p.entities.push_back(makePartOfRoad(cross(ec), end, re));
            }
            if ( p.totalLength() < sPath.totalLength() ){
                sPath = std::move(p);
            }
        }*/
        vector<int> end_possible_cross = possible_end_nearby_cross(end, re);
        vector<PartOfRoad> part_of_road_e;
        unordered_map<int, double> endG;
        for(auto v : end_possible_cross){
            part_of_road_e.push_back(make_part_of_road(cross(v), end, roadsegment(end.index)));
            endG[v] = part_of_road_e.back().length;
        }
        vector<int> cross_index = 
            shortest_path_impl(*this, {start}, {0.0}, {end_possible_cross.begin(), end_possible_cross.end()},
                [&](int v){
                    if ( endG.count(v) ){
                        return endG[v];
                    }
                    if ( this->shortest_path_strategy == RoadMap::Dijkstra ){
                        return 0.0;
                    }else{
                        return bg::distance(this->cross(v), end);
                    }
                });
        if ( !cross_index.empty() ){
            for(size_t i = 1; i < cross_index.size(); ++i){
                auto findedEdge = b::edge(cross_index[i-1], cross_index[i], graph);
                assert(findedEdge.second);
                ARoadSegment ars;
                ars.start_cross = cross_index[i-1];
                ars.end_cross = cross_index[i];
                ars.roadsegment_index = get(IndexOfEdge, graph, findedEdge.first);
                ars.length = get(edge_weight_map, findedEdge.first);
                sPath.entities.push_back(ars);
            }
            sPath.entities.push_back(make_part_of_road(cross(cross_index.back()), end, roadsegment(end.index)));
        }
        return sPath;
    }
}
