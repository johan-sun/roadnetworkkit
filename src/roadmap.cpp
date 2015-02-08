//auto-include {{{
#include  <boost/range/algorithm.hpp>
#include    "roadmap.h"
//}}} 




void Map::_index(){
    crossIndex = PointRTree(bgi::dynamic_quadratic(_cross.size()));
    for(auto& c : _cross){
        crossIndex.insert({c.geometry, c.index});
    }

    int segmentCount = 0;
    for(auto& r : _roadsegment){
        bg::for_each_segment(r.geometry, 
                [&segmentCount](bg::model::referring_segment<Point> const&){ ++segmentCount; });
    }

    segmentIndex = SegmentRTree(bgi::dynamic_quadratic(segmentCount));
    for(auto& r : _roadsegment){
        bg::for_each_segment(r.geometry, [this, &r](bg::model::referring_segment<Point> const& seg){
            this->segmentIndex.insert({{seg.first, seg.second}, r.index});
        });
    }
    _inRoad.resize(_cross.size());
    _outRoad.resize(_cross.size());
    for(auto& r : _roadsegment){
        if ( r.direction & Forward ){
            _outRoad[r.startCrossIndex].push_back({r.index, r.endCrossIndex});
            _inRoad[r.endCrossIndex].push_back({r.index, r.startCrossIndex});
        }
        if ( r.direction & Backward ){
            _outRoad[r.endCrossIndex].push_back({r.index, r.startCrossIndex});
            _inRoad[r.startCrossIndex].push_back({r.index, r.endCrossIndex});
        }
    }
}

void Map::buildGraph(){
    for(int i = 0; i < _cross.size(); ++i){
        b::add_vertex(graph);
    }

    GraphTraits::edge_descriptor edge;
    for(auto& r : _roadsegment){
        if ( r.direction & Forward ){
            _roadEdgeMap.insert({r.index, b::add_edge(r.startCrossIndex, r.endCrossIndex, r.index,graph).first});
        }
        if ( r.direction & Backward ){
            _roadEdgeMap.insert({r.index, b::add_edge(r.endCrossIndex, r.startCrossIndex, r.index,graph).first});
        }
    }
}

void Map::visitRoadSegment(std::function<void(RoadSegment const&r)> visitor)const{
    b::for_each(_roadsegment, visitor);
}
void Map::visitRoadSegment(std::function<void(RoadSegment &r)> visitor){
    b::for_each(_roadsegment, visitor);
}
void Map::visitEdge(std::function<void(RoadSegment const&r, GraphTraits::edge_descriptor, Graph const& g)> visitor)const{
    for(auto p = b::edges(graph); p.first != p.second; ++p.first){
        GraphTraits::edge_descriptor edge = *p.first;
        int roadIdx = b::get(roadIndexOfEdgeTag, graph, edge);
        visitor(_roadsegment[roadIdx], edge, graph);
    }
}
void Map::visitEdge(std::function<void(RoadSegment &r, GraphTraits::edge_descriptor, Graph &)> visitor){
    for(auto p = b::edges(graph); p.first != p.second; ++p.first){
        GraphTraits::edge_descriptor edge = *p.first;
        int roadIdx = b::get(roadIndexOfEdgeTag, graph, edge);
        visitor(_roadsegment[roadIdx], edge, graph);
    }
}


void Map::visitCross(std::function<void(Cross const&)> visitor)const{
    b::for_each(_cross, visitor);
}
void Map::visitCross(std::function<void(Cross &)> visitor){
    b::for_each(_cross, visitor);
}
