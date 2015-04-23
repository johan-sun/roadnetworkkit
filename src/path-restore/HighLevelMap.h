#ifndef  HIGHLEVELMAP_H
#define  HIGHLEVELMAP_H
//auto-include {{{
#include <string>
#include <memory>
#include <boost/property_map/function_property_map.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range.hpp>
#include <boost/iterator_adaptors.hpp>
#include <unordered_set>
#include    "generalmap.h"
//}}}


class VirtualEdgeWeight
{
public:
    VirtualEdgeWeight()=default;
    VirtualEdgeWeight(double ms);
    VirtualEdgeWeight(std::string const& partten);
    boost::posix_time::time_duration operator()(boost::posix_time::time_duration const& timeOfDay)const;
    boost::posix_time::time_duration operator()(boost::posix_time::ptime const& time)const{
        return operator()(time.time_of_day());
    }
    operator bool ()const{
        return _pimpl.get();
    }
    void displayTo(std::ostream& o)const;
private:
    struct Impl;
    std::shared_ptr<Impl> _pimpl;
};

inline std::ostream& operator<<(std::ostream& o, VirtualEdgeWeight const& e){
    e.displayTo(o);
    return o;
}

inline boost::posix_time::time_duration operator+(VirtualEdgeWeight const& w, boost::posix_time::time_duration const& d)
{
    return d + w(d);
}
inline boost::posix_time::time_duration operator+(boost::posix_time::time_duration const& d, VirtualEdgeWeight const& w)
{
    return d + w(d);
}
inline boost::posix_time::ptime operator+(VirtualEdgeWeight const& w, boost::posix_time::ptime const& t)
{
    return t + w(t);
}

inline boost::posix_time::ptime operator+(boost::posix_time::ptime const& t, VirtualEdgeWeight const& w)
{
    return t + w(t);
}

class HighLevelMap : public Map
{
public:
    struct WeightGetter{
        WeightGetter(HighLevelMap* map):hmp(map){}
        VirtualEdgeWeight operator()(GraphTraits::edge_descriptor const& edge)const{
            return hmp->weight(edge);
        }
        HighLevelMap* hmp;
    };

    HighLevelMap():virtualEdgeWeightProperty(WeightGetter(this)){}

    bool load(std::string const& shp, char const* server, char const* user, char const* pass);

    VirtualEdgeWeight weight(GraphTraits::edge_descriptor const& ed)const{
        return weight(roadsegment(ed).index);
    }

    VirtualEdgeWeight weight(int roadIndex)const{
        return _roadWeight.at(roadIndex);
    }

    class OutIter;
    
    Cross const& addTmpCross(int ID, double x, double y);
    RoadSegment const& addTmpVirtualEdge(int crossStartIndex, int crossEndIndex, VirtualEdgeWeight const& weight);

    OutIter outRoadIncludeTmp(int crossIndex)const;
    boost::iterator_range<OutIter> outRoadRangeIncludeTmp(int crossIndex)const;


    VirtualEdgeWeight weightOrTmpOne(int index)const;
    void clearTmp(){
        _tmpCross.clear();
        _tmpCrossIDIndexMap.clear();
        _tmpRoad.clear();
    }
    Cross const& crossOrTmpOne(int index)const;

    boost::function_property_map<WeightGetter, GraphTraits::edge_descriptor, VirtualEdgeWeight>
        virtualEdgeWeightProperty;

    struct TimedCross : public boost::equality_comparable<TimedCross>
    {
        int crossIndex;
        boost::posix_time::time_duration time;
        bool operator==(TimedCross const& t)const
        {
            return crossIndex == t.crossIndex && time == t.time;
        }
    };

    inline std::vector<TimedCross> 
        shortestPath(int indexA, boost::posix_time::time_duration const& time,  int indexB)const{
            std::unordered_set<int> noDelete;
            return shortestPath(indexA, time, indexB, noDelete);
        }
    std::vector<TimedCross> 
        shortestPath(int indexA, boost::posix_time::time_duration const& time,  int indexB, 
                std::unordered_set<int> const& deleteEdges)const;

private:
    struct TmpRoad{
        RoadSegment road;
        VirtualEdgeWeight weight;
    };
    std::vector<Cross> _tmpCross;
    std::unordered_map<int, int> _tmpCrossIDIndexMap;
    std::vector<TmpRoad> _tmpRoad;
    std::unordered_map<int, std::vector<RoadIndexCrossIndexPair> > _tmpAdjList;
    std::vector<VirtualEdgeWeight> _roadWeight;
};

class HighLevelMap::OutIter : public boost::iterator_facade<OutIter, RoadIndexCrossIndexPair const, boost::forward_traversal_tag>
{
public:
    OutIter():map(0),i(-1),cross(-1){}
private:
    friend class boost::iterator_core_access;
    friend class HighLevelMap;
    OutIter(HighLevelMap const& map, int cross):map(&map), cross(cross), i(0){
        hasTmp = inTmp = false;
        offset = 0;
        if ( map.containCross(cross) )
        {
            offset = map.outRoadOf(cross).size();
        }
        
        if ( map._tmpAdjList.count(cross)  ){
            hasTmp = true;
        }

        if ( offset == 0 ){
            inTmp = true;
        }
        if ( offset == 0 && !hasTmp ){
            invalid_this();
        }
    }
    int cross;
    int i;
    bool hasTmp;
    bool inTmp;
    size_t offset;
    HighLevelMap const* map;

    RoadIndexCrossIndexPair const& dereference()const{
        if ( *this == OutIter() ){
            throw std::out_of_range("iterator dereference out of range");
        }
        if ( inTmp )
        {
            return map->_tmpAdjList.at(cross).at(i-offset);
        }
        return map->outRoadOf(cross).at(i);
    }
    void increment(){
        if ( *this == OutIter() )
            return;
        ++i;
        if ( offset && i == map->outRoadOf(cross).size())
        {
            if(!hasTmp) invalid_this();
            else inTmp = true;
        }
        if( hasTmp && i - offset == map->_tmpAdjList.at(cross).size())
            invalid_this();
    }
    bool equal(OutIter const& otr)const{
        if ( &otr == this ) return true;
        return cross == otr.cross && i == otr.i && map == otr.map;
    }
    void invalid_this(){
        map = nullptr;
        cross = -1;
        i = -1;
    }
};

inline boost::iterator_range<HighLevelMap::OutIter> HighLevelMap::outRoadRangeIncludeTmp(int crossIndex)const
{
    OutIter end;
    return boost::make_iterator_range(outRoadIncludeTmp(crossIndex), end);
}


class KShortestPathGenerator
{
public:
    KShortestPathGenerator(HighLevelMap const& map, int index1, boost::posix_time::time_duration const& t, int index2);
    std::vector<HighLevelMap::TimedCross> const* nextPath();
private:
    struct Impl;
    std::shared_ptr<Impl> _pimpl;
};
#endif  /*HIGHLEVELMAP_H*/
