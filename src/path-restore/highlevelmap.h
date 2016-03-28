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

    HighLevelMap():virtual_edge_weight_property(WeightGetter(this)){}

    bool load(std::string const& shp, char const* server, char const* user, char const* pass);

    VirtualEdgeWeight weight(GraphTraits::edge_descriptor const& ed)const{
        return weight(roadsegment(ed).index);
    }

    VirtualEdgeWeight weight(int roadIndex)const{
        return road_weight_.at(roadIndex);
    }

    class OutIter;
    
    Cross const& add_tmp_cross(int ID, double x, double y);
    RoadSegment const& add_tmp_virtual_edge(int crossStartIndex, int crossEndIndex, VirtualEdgeWeight const& weight);

    OutIter out_road_include_tmp(int cross_index)const;
    boost::iterator_range<OutIter> out_road_range_include_tmp(int cross_index)const;


    VirtualEdgeWeight weight_or_tmp_one(int index)const;
    void clear_tmp(){
        tmp_cross_.clear();
        tmp_cross_id_index_map_.clear();
        tmp_road_.clear();
    }
    bool contain_cross_inc_tmp(int index)const{
        if (contain_cross(index)) return true;
        index -= cross_size();
        return index >= 0 && index < tmp_cross_.size();
    }
    bool contain_cross_inc_tmp_by_id(int id)const{
        if ( contain_cross("ID", id) ) return true;
        return tmp_cross_id_index_map_.count(id);
    }
    Cross const& cross_or_tmp_one(int index)const;
    Cross const& cross_or_tmp_one_by_id(int id)const
    {
        if ( contain_cross("ID", id) ) return cross("ID", id);
        return cross_or_tmp_one(tmp_cross_id_index_map_.at(id));
    }

    boost::function_property_map<WeightGetter, GraphTraits::edge_descriptor, VirtualEdgeWeight>
        virtual_edge_weight_property;

    struct TimedCross : public boost::equality_comparable<TimedCross>
    {
        int cross_index;
        boost::posix_time::time_duration time;
        bool operator==(TimedCross const& t)const
        {
            return cross_index == t.cross_index && time == t.time;
        }
    };

    inline std::vector<TimedCross> 
        shortest_path(int indexA, boost::posix_time::time_duration const& time,  int indexB)const{
            std::unordered_set<int> noDelete;
            return shortest_path(indexA, time, indexB, noDelete, noDelete, nullptr, nullptr);
        }
    std::vector<TimedCross> 
        shortest_path(int indexA, boost::posix_time::time_duration const& time,  int indexB, 
                std::unordered_set<int> const& deleteEdges, std::unordered_set<int> const& deleteCross, 
                std::function<bool(Cross const&)> const& crossIgnore = nullptr,
                std::function<bool(boost::posix_time::time_duration const&)> const& timeIgnore = nullptr)const;
    /*std::vector<TimedCross>
        shortestPathMinTurn(int indexA, boost::posix_time::time_duration const& time, int indexB,
                std::unordered_set<int> const& deleteEdges, std::unordered_set<int> const& deleteCross, 
                std::function<bool(Cross const&)> const& crossIgnore = nullptr,
                std::function<bool(boost::posix_time::time_duration const&)> const& timeIgnore = nullptr)const;*/

private:
    struct TmpRoad{
        RoadSegment road;
        VirtualEdgeWeight weight;
    };
    std::vector<Cross> tmp_cross_;
    std::unordered_map<int, int> tmp_cross_id_index_map_;
    std::vector<TmpRoad> tmp_road_;
    std::unordered_map<int, std::vector<RoadIndexCrossIndexPair> > tmp_adj_list_;
    std::vector<VirtualEdgeWeight> road_weight_;
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
        if ( map.contain_cross(cross) )
        {
            offset = map.successor_road_of(cross).size();
        }
        
        if ( map.tmp_adj_list_.count(cross)  ){
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
            return map->tmp_adj_list_.at(cross).at(i-offset);
        }
        return map->successor_road_of(cross).at(i);
    }
    void increment(){
        if ( *this == OutIter() )
            return;
        ++i;
        if ( offset && i == map->successor_road_of(cross).size())
        {
            if(!hasTmp) invalid_this();
            else inTmp = true;
        }
        if( hasTmp && i - offset == map->tmp_adj_list_.at(cross).size())
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

inline boost::iterator_range<HighLevelMap::OutIter> HighLevelMap::out_road_range_include_tmp(int cross_index)const
{
    OutIter end;
    return boost::make_iterator_range(out_road_include_tmp(cross_index), end);
}


class KShortestPathGenerator
{
public:
    KShortestPathGenerator(HighLevelMap const& map, int index1, boost::posix_time::time_duration const& t, int index2, boost::posix_time::time_duration const& t2terminate, Box const& range);
    std::vector<HighLevelMap::TimedCross> const* nextPath();
private:
    struct Impl;
    std::shared_ptr<Impl> _pimpl;
};
#endif  /*HIGHLEVELMAP_H*/
