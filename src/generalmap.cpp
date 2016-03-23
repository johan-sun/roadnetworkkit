//auto-include {{{
#include  <boost/range/algorithm.hpp>
#include "generalmap.h"
//}}} 
using namespace std;
void Map::index_(){
    cross_rtree = PointRTree(bgi::dynamic_quadratic(cross_.size()));
    for(auto& c : cross_){
        cross_rtree.insert({c.geometry, c.index});
    }


    roadsegment_rtree = RoadSegmentRTree(bgi::dynamic_quadratic(roadsegment_.size()));
    for(auto& r : roadsegment_){
        Box bx = bg::return_envelope<Box>(r.geometry);
        roadsegment_rtree.insert(make_pair(std::move(bx), r.index));
    }

    predecessor_road_.resize(cross_.size());
    successor_road_.resize(cross_.size());
    for(auto& r : roadsegment_){
        if ( r.direction & Forward ){
            successor_road_[r.start_cross_index].push_back({r.index, r.end_cross_index});
            predecessor_road_[r.end_cross_index].push_back({r.index, r.start_cross_index});
        }
        if ( r.direction & Backward ){
            successor_road_[r.end_cross_index].push_back({r.index, r.start_cross_index});
            predecessor_road_[r.start_cross_index].push_back({r.index, r.end_cross_index});
        }
    }
}

void Map::build_graph(){
    for(int i = 0; i < cross_.size(); ++i){
        b::add_vertex(graph);
    }

    GraphTraits::edge_descriptor edge;
    for(auto& r : roadsegment_){
        if ( r.direction & Forward ){
            road_edge_map_.insert({r.index, b::add_edge(r.start_cross_index, r.end_cross_index, r.index,graph).first});
        }
        if ( r.direction & Backward ){
            road_edge_map_.insert({r.index, b::add_edge(r.end_cross_index, r.start_cross_index, r.index,graph).first});
        }
    }
}

void Map::visit_roadsegment(std::function<void(RoadSegment const&r)> visitor)const{
    b::for_each(roadsegment_, visitor);
}

void Map::visit_roadsegment(std::function<void(RoadSegment &r)> visitor){
    b::for_each(roadsegment_, visitor);
}

void Map::visit_edge(std::function<void(RoadSegment const&r, GraphTraits::edge_descriptor, Graph const& g)> visitor)const{
    for(auto p = b::edges(graph); p.first != p.second; ++p.first){
        GraphTraits::edge_descriptor edge = *p.first;
        int road_idx = b::get(IndexOfEdge, graph, edge);
        visitor(roadsegment_[road_idx], edge, graph);
    }
}

void Map::visit_edge(std::function<void(RoadSegment &r, GraphTraits::edge_descriptor, Graph &)> visitor){
    for(auto p = b::edges(graph); p.first != p.second; ++p.first){
        GraphTraits::edge_descriptor edge = *p.first;
        int road_idx = b::get(IndexOfEdge, graph, edge);
        visitor(roadsegment_[road_idx], edge, graph);
    }
}

void Map::visit_cross(std::function<void(Cross const&)> visitor)const{
    b::for_each(cross_, visitor);
}
void Map::visit_cross(std::function<void(Cross &)> visitor){
    b::for_each(cross_, visitor);
}
