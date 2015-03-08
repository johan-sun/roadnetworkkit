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

double Path::totalLength()const{
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
        //d += e.apply_visitor(LengthGet());
        d += b::apply_visitor(LengthGet(), e);
    }
    return d;
}
vector<int> RoadMap::queryCross(Point const &p, double radious) const {
    Box box = range_box(p, radious);
    vector<int> ret;
    vector<pair<int, double> > candidate;
    double rad2 = radious * radious;
    for (pair<Point, int > const& pi  : crossIndex | bgi::adaptors::queried(bgi::covered_by(box)) ){
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


vector<int> RoadMap::queryRoad(Point const& p, double radious)const{
    Box box = range_box(p, radious);
    vector<int> ret;
    vector<pair<int, double> > cand;
    set<int> roadIdx;
    double rad2 = radious * radious;
    for( pair<Box, int> const& pi :
            roadsegmentRTree | bgi::adaptors::queried(bgi::covers(box))){
        roadIdx.insert(pi.second);
    }

    for( pair<Box, int> const& pi :
            roadsegmentRTree | bgi::adaptors::queried(bgi::covered_by(box))){
        roadIdx.insert(pi.second);
    }

    for( pair<Box, int> const& pi :
            roadsegmentRTree | bgi::adaptors::queried(bgi::overlaps(box))){
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
            .weight_map(edgeWeightMap));
    vector<GraphTraits::vertex_descriptor> crossIndex;
    GraphTraits::vertex_descriptor p = crossB;
    while ( p != GraphTraits::null_vertex() ){
        crossIndex.push_back(p);
        p = pre.count(p) == 0 ? GraphTraits::null_vertex() : pre[p];
    }

    b::reverse(crossIndex);
    for(int i = 1; i < crossIndex.size(); ++i){
        auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
        assert(findedEdge.second);
        ARoadSegment ars;
        ars.startCross = crossIndex[i-1];
        ars.endCross = crossIndex[i];
        RoadSegment const& r = roadsegment(findedEdge.first);
        ars.roadsegmentIndex = r.index;
        ars.length = r.properties.get<double>("GEO_LENGTH");
        path.entities.push_back(ars);
    }
    return path;
}


ProjectPoint makeProjectPoint(Point const& point, RoadSegment const& r){
    ProjectPoint pp;
    pp.geometry = projectPoint(point, r.geometry);
    if ( bg::equals(pp.geometry, r.geometry.front()) ){
        pp.type = ProjectPoint::OnCross;
        pp.index = r.startCrossIndex;
        pp.param = 0.0;
    }else if ( bg::equals(pp.geometry, r.geometry.back()) ){
        pp.type = ProjectPoint::OnCross;
        pp.index = r.endCrossIndex;
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


PartOfRoad makePartOfRoad(Point const& start, Point const& end, RoadSegment const& r){
    PartOfRoad pr;
    pr.start = makeProjectPoint(start, r);
    pr.end = makeProjectPoint(end, r);
    pr.roadsegmentIndex = r.index;
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
    for(int i = 1; i < r.geometry.size(); ++i){
        bg::model::referring_segment<Point const> seg(r.geometry[i-1], r.geometry[i]);
        double l = bg::length(seg);
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
        RoadSegment const& rs = map.roadsegment(r.roadsegmentIndex);
        Linestring l = rs.geometry;
        if ( r.startCross ==  rs.endCrossIndex){
            b::reverse(l);
        }
        bg::append(line, l);
    }

    void operator()(PartOfRoad const& pr){
        bg::append(line, geometry(pr, map.roadsegment(pr.roadsegmentIndex)));
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

ProjectPoint RoadMap::nearestProject(Point const &p) const {

    int roadIdx = roadsegmentRTree.qbegin(bgi::nearest(p, 1))->second;
    double r = bg::distance(p, roadsegment(roadIdx).geometry);
    vector<int> ret = queryRoad(p, r);
    int min = ret.empty() ? roadIdx: ret[0];
    return makeProjectPoint(p, roadsegment(min));
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
shortestPathImple(
    RoadMap const& map,
    vector<int> const& init, 
    vector<double> const& initG,
    unordered_set<int> const& finishSet,
    HF hf){

    vector<int>  crossIndex;
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
                crossIndex.push_back(p);
                p = pre[p];
            }

            b::reverse(crossIndex);
            return crossIndex;
        }

        close.insert(top.cross);
        for(auto iterPair = b::out_edges(top.cross, map.graph); iterPair.first != iterPair.second; ++iterPair.first){
            RoadMap::GraphTraits::edge_descriptor edge = *iterPair.first;
            RoadMap::GraphTraits::vertex_descriptor t = b::target(edge, map.graph);
            if ( close.count(t) ){
                continue;
            }

            double g = get(map.edgeWeightMap, edge) + top.g;
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

    return crossIndex;
}

Path RoadMap::shortestPathAStar(int crossA, int crossB)const{
    Path path;
    if ( crossA == crossB ){
        path.entities.push_back(makeProjectPointForCross(cross(crossA)));
        return path;
    }
    vector<int> crossIndex = 
        shortestPathImple(*this, {crossA}, {0.0}, {crossB}, 
                [&](int cross){
                    return bg::distance(this->cross(cross), this->cross(crossB));
                });
    for(int i = 1; i < crossIndex.size(); ++i){
        auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
        assert(findedEdge.second);
        ARoadSegment ars;
        ars.startCross = crossIndex[i-1];
        ars.endCross = crossIndex[i];
        ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
        ars.length = get(edgeWeightMap, findedEdge.first);
        path.entities.push_back(ars);
    }
    return path;
    /*
    typedef b::heap::fibonacci_heap<AStarNode> Heap;
    Heap open;
    unordered_set<GraphTraits::vertex_descriptor> close;
    unordered_map<GraphTraits::vertex_descriptor, GraphTraits::vertex_descriptor> pre;
    unordered_map<GraphTraits::vertex_descriptor, Heap::handle_type> savedHandle;
    savedHandle[crossA] = open.push({crossA, 0.0, 0.0});
    pre[crossA] = GraphTraits::null_vertex();
    while(!open.empty()){
        AStarNode top = open.top();
        open.pop();
        if ( top.cross == crossB ){
            vector<int> crossIndex;
            GraphTraits::vertex_descriptor p = crossB;
            while ( p != GraphTraits::null_vertex() ){
                crossIndex.push_back(p);
                p = pre[p];
            }

            b::reverse(crossIndex);
            for(int i = 1; i < crossIndex.size(); ++i){
                auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
                assert(findedEdge.second);
                ARoadSegment ars;
                ars.startCross = crossIndex[i-1];
                ars.endCross = crossIndex[i];
                ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
                ars.length = get(edgeWeightMap, findedEdge.first);
                path.entities.push_back(ars);
            }
            return path;
        }

        close.insert(top.cross);
        for(auto iterPair = b::out_edges(top.cross, graph); iterPair.first != iterPair.second; ++iterPair.first){
            GraphTraits::edge_descriptor edge = *iterPair.first;
            int t = b::target(edge, graph);
            if ( close.count(t) ){
                continue;
            }

            double g = get(edgeWeightMap, edge) + top.g;
            double h = bg::distance(cross(t), cross(crossB));
            auto it = savedHandle.find(t);
            if ( it == savedHandle.end()){
                savedHandle[t] = open.push({t,g,h});
                pre[t] = top.cross;
            }else{
                AStarNode& node = *(it->second);
                if ( g + h < node.g + node.h ){
                    AStarNode newNode{t, g, h};
                    pre[t] = top.cross;
                    open.update_lazy(it->second, newNode);
                }
            }
        }
    }
    return path;*/
}

Path RoadMap::shortestPathDijkstra(int crossA, int crossB)const{
    Path path;
    if ( crossA == crossB ){
        path.entities.push_back(makeProjectPointForCross(cross(crossA)));
        return path;
    }

    vector<int> crossIndex = 
        shortestPathImple(*this, {crossA}, {0.0}, {crossB}, [](int){ return 0.0; });
    for(int i = 1; i < crossIndex.size(); ++i){
        auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
        assert(findedEdge.second);
        ARoadSegment ars;
        ars.startCross = crossIndex[i-1];
        ars.endCross = crossIndex[i];
        ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
        ars.length = get(edgeWeightMap, findedEdge.first);
        path.entities.push_back(ars);
    }
    return path;

    /*
    typedef b::heap::fibonacci_heap<AStarNode> Heap;
    Heap open;
    unordered_set<GraphTraits::vertex_descriptor> close;
    unordered_map<GraphTraits::vertex_descriptor, GraphTraits::vertex_descriptor> pre;
    unordered_map<GraphTraits::vertex_descriptor, Heap::handle_type> savedHandle;
    savedHandle[crossA] = open.push({crossA, 0.0, 0.0});
    pre[crossA] = GraphTraits::null_vertex();
    while(!open.empty()){
        AStarNode top = open.top();
        open.pop();
        if ( top.cross == crossB ){
            vector<int> crossIndex;
            GraphTraits::vertex_descriptor p = crossB;
            while ( p != GraphTraits::null_vertex() ){
                crossIndex.push_back(p);
                p = pre[p];
            }

            b::reverse(crossIndex);
            for(int i = 1; i < crossIndex.size(); ++i){
                auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
                assert(findedEdge.second);
                ARoadSegment ars;
                ars.startCross = crossIndex[i-1];
                ars.endCross = crossIndex[i];
                ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
                ars.length = get(edgeWeightMap, findedEdge.first);
                path.entities.push_back(ars);
            }
            return path;
        }

        close.insert(top.cross);
        for(auto iterPair = b::out_edges(top.cross, graph); iterPair.first != iterPair.second; ++iterPair.first){
            GraphTraits::edge_descriptor edge = *iterPair.first;
            int t = b::target(edge, graph);
            if ( close.count(t) ){
                continue;
            }

            double g = get(edgeWeightMap, edge) + top.g;
            double h = 0.0;
            //double h = bg::distance(cross(t), cross(crossB));
            auto it = savedHandle.find(t);
            if ( it == savedHandle.end()){
                savedHandle[t] = open.push({t,g,h});
                pre[t] = top.cross;
            }else{
                AStarNode& node = *(it->second);
                if ( g + h < node.g + node.h ){
                    AStarNode newNode{t, g, h};
                    pre[t] = top.cross;
                    open.update_lazy(it->second, newNode);
                }
            }
        }
    }*/
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
        return { r.startCrossIndex, r.endCrossIndex };
    }

    //p1.param > p2.param
    assert( r.direction == Direction::Forward );
    return { r.endCrossIndex, r.startCrossIndex };
}

vector<int> possibleStartNearbyCross(ProjectPoint const&, RoadSegment const& r){
    vector<int> c;
    if ( r.direction & Direction::Forward ){
        c.push_back(r.endCrossIndex);
    }

    if ( r.direction & Direction::Backward ){
        c.push_back(r.startCrossIndex);
    }

    return c;
}

vector<int> possibleEndNearbyCross(ProjectPoint const&, RoadSegment const& r){
    vector<int> c;
    if ( r.direction & Direction::Forward ){
        c.push_back(r.startCrossIndex);
    }

    if ( r.direction & Direction::Backward ){
        c.push_back(r.endCrossIndex);
    }

    return c;
}

Path RoadMap::shortestPath(ProjectPoint const& start, ProjectPoint const& end)const{
    if ( bg::equals(start,end) ){
        Path p;
        p.entities.push_back(start);
        return p;
    }

    if ( start.type == ProjectPoint::OnCross && end.type == ProjectPoint::OnCross ){//OnCross OnCross
        return shortestPath(start.index, end.index);
    }else if ( start.type == ProjectPoint::OnRoad && end.type == ProjectPoint::OnRoad ){//OnRoad OnRoad
        if (start.index == end.index){//On Same Road
            RoadSegment const& r = roadsegment(start.index);
            if(startCanGoToEndDirectly(start, end, r)){
                Path p;
                p.entities.push_back(makePartOfRoad(start, end, r));
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
            vector<int> startPossibleCross = possibleStartNearbyCross(start, rs);
            vector<int> endPossibleCross = possibleEndNearbyCross(end, re);
            for(int sc : startPossibleCross){
                for(int ec : endPossibleCross){
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
            vector<int> startPossibleCross = possibleStartNearbyCross(start, rs);
            vector<int> endPossibleCross = possibleEndNearbyCross(end, re);
            vector<PartOfRoad> partOfRoads;
            vector<PartOfRoad> partOfRoade;
            vector<double> initG;
            for(auto v : startPossibleCross){
                partOfRoads.push_back(makePartOfRoad(start ,cross(v), roadsegment(start.index)));
                initG.push_back(partOfRoads.back().length);
            }
            unordered_map<int, double> endG;
            for(auto v : endPossibleCross){
                partOfRoade.push_back(makePartOfRoad(cross(v), end, roadsegment(end.index)));
                endG[v] = partOfRoade.back().length;
            }

            vector<int> crossIndex = 
                shortestPathImple(*this, startPossibleCross, initG, {endPossibleCross.begin(), endPossibleCross.end()},
                        [&](int v){
                            if ( endG.count(v) ){
                                return endG[v];
                            }
                            if ( this->shortestPathStrategy == RoadMap::Dijkstra ){
                                return 0.0;
                            }else{
                                return bg::distance(this->cross(v), end);
                            }
                        });
            if ( !crossIndex.empty() ){
                for(int i = 1; i < crossIndex.size(); ++i){
                    auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
                    assert(findedEdge.second);
                    ARoadSegment ars;
                    ars.startCross = crossIndex[i-1];
                    ars.endCross = crossIndex[i];
                    ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
                    ars.length = get(edgeWeightMap, findedEdge.first);
                    sPath.entities.push_back(ars);
                }
                sPath.entities.push_front(partOfRoads[b::find<b::return_begin_found>(startPossibleCross, crossIndex.front()).size()]);
                sPath.entities.push_back(partOfRoade[b::find<b::return_begin_found>(endPossibleCross, crossIndex.back()).size()]);
            }
            return sPath;
        }
    }else if ( start.type == ProjectPoint::OnCross ){//OnCross OnRoad
        return shortestPath(start.index, end);
    }else{//OnRoad OnCross
        return shortestPath(start, end.index);
    }
}


Path RoadMap::shortestPath(ProjectPoint const& start, int end)const{
    if ( start.type == ProjectPoint::OnCross ){
        return shortestPath(start.index, end);
    }else{
        RoadSegment const& rs = roadsegment(start.index);
        vector<int> startPossibleCross = possibleStartNearbyCross(start, rs);
        vector<PartOfRoad> partOfRoads;
        vector<double> initG;
        for(auto v : startPossibleCross){
            partOfRoads.push_back(makePartOfRoad(start, cross(v), roadsegment(start.index)));
            initG.push_back(partOfRoads.back().length);
        }
        Path sPath;
        /*
        for(int sc : startPossibleCross){
            Path p;
            Path bodyPath = shortestPath(sc, end);
            if ( bodyPath.valid()){
                p.entities.push_back(makePartOfRoad(start, cross(sc), rs));
                if ( bodyPath.entities.front().type() == typeid(ARoadSegment))
                    p.entities.splice(p.entities.end(), bodyPath.entities);
            }
            if ( p.totalLength() < sPath.totalLength() ){
                sPath = std::move(p);
            }
        }*/
        vector<int> crossIndex = 
                shortestPathImple(*this, startPossibleCross, initG, {end},
                        [&](int v){
                            if ( this->shortestPathStrategy == RoadMap::Dijkstra ){
                                return 0.0;
                            }else{
                                return bg::distance(this->cross(v), this->cross(end));
                            }
                        });
        if ( !crossIndex.empty() ){
            for(int i = 1; i < crossIndex.size(); ++i){
                auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
                assert(findedEdge.second);
                ARoadSegment ars;
                ars.startCross = crossIndex[i-1];
                ars.endCross = crossIndex[i];
                ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
                ars.length = get(edgeWeightMap, findedEdge.first);
                sPath.entities.push_back(ars);
            }
            sPath.entities.push_front(partOfRoads[b::find<b::return_begin_found>(startPossibleCross, crossIndex.front()).size()]);
        }
        return sPath;
    }
}

Path RoadMap::shortestPath(int start, ProjectPoint const& end)const{
    if ( end.type == ProjectPoint::OnCross ){
        return shortestPath(start, end.index);
    }else{
        RoadSegment const& re = roadsegment(end.index);
        Path sPath;
        /*
        for(int ec : endPossibleCross){
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
        vector<int> endPossibleCross = possibleEndNearbyCross(end, re);
        vector<PartOfRoad> partOfRoade;
        unordered_map<int, double> endG;
        for(auto v : endPossibleCross){
            partOfRoade.push_back(makePartOfRoad(cross(v), end, roadsegment(end.index)));
            endG[v] = partOfRoade.back().length;
        }
        vector<int> crossIndex = 
            shortestPathImple(*this, {start}, {0.0}, {endPossibleCross.begin(), endPossibleCross.end()},
                [&](int v){
                    if ( endG.count(v) ){
                        return endG[v];
                    }
                    if ( this->shortestPathStrategy == RoadMap::Dijkstra ){
                        return 0.0;
                    }else{
                        return bg::distance(this->cross(v), end);
                    }
                });
        if ( !crossIndex.empty() ){
            for(int i = 1; i < crossIndex.size(); ++i){
                auto findedEdge = b::edge(crossIndex[i-1], crossIndex[i], graph);
                assert(findedEdge.second);
                ARoadSegment ars;
                ars.startCross = crossIndex[i-1];
                ars.endCross = crossIndex[i];
                ars.roadsegmentIndex = get(roadIndexOfEdgeTag, graph, findedEdge.first);
                ars.length = get(edgeWeightMap, findedEdge.first);
                sPath.entities.push_back(ars);
            }
            sPath.entities.push_back(makePartOfRoad(cross(crossIndex.back()), end, roadsegment(end.index)));
        }
        return sPath;
    }
}
