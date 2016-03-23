#ifndef  GENERAL_MAP_H
#define  GENERAL_MAP_H
//auto-include {{{
#include  <utility>
#include  <boost/algorithm/string.hpp>
#include  <boost/graph/adjacency_list.hpp>
#include  <boost/property_tree/ptree.hpp>
#include  <boost/any.hpp>
#include  <functional>
#include <string>
#include <vector>
#include <shapefil.h>
#include <type_traits>
#include <unordered_map>
#include    "geomerty.h"
#include    "traits.hpp"
//}}}


namespace bgi = boost::geometry::index;
namespace pt = boost::property_tree;
/// \brief cross in map
struct Cross{
    int index;
    Point geometry;
    pt::ptree properties;///< user stored properties
};

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Cross, double , bg::cs::cartesian, geometry.x, geometry.y, geometry.x, geometry.y);

/// \file
/// \brief direction of road segment
enum  Direction{
    Forward = 1,///< r.s->r.e
    Backward = 2,///< r.e->r.s
    Bidirection = 3,///< both r.s->r.e and r.e->r.s
};

/// \file
/// \brief cross posistion in road segment
enum CrossPosInRoad{
    Front,///< point is r.s
    Back///< point is r.e
};

/// \brief road segment
struct RoadSegment{
    int index;
    int start_cross_index;
    int end_cross_index;
    Direction direction;
    Linestring geometry;
    pt::ptree properties;///< user stored properties
    //std::unordered_map<string, boost::any> properties;
};

/// \brief 通过几何来判断顶点是否添加
/// 
/// Checker通过 cross_is_new 判断路口是否是新的,
/// 若路口是新的则会通过 store_index 存储该点对应的索引
/// 若路口已经添加过,则通过 get_index 获取路口的索引<br/>
/// 默认路口检查器
struct GeometryChecker{
private:
    struct PointHash{
        size_t operator()(Point const& p)const{
            std::hash<double> hasher;
            return hasher(p.x()) ^ ( hasher(p.y()) << 1 );
        }
    };
    struct PointEqual{
        bool operator()(Point const& a, Point const& b)const{
            return a.x() == b.x() && a.y() == b.y();
        }
    };
    std::unordered_map<Point, int, PointHash, PointEqual> saved;
public:

    /// \brief 判断是否是新添加路口
    /// \param[in] pos 路口在路段Linestring的位置
    /// \param[in] p   路口的几何点
    /// \param[in] r   路口所连的路段
    /// \param[in] handle DBFHandle
    /// \param[in] road_index 道路对应的索引
    /// \return  是否是未添加索引的点
    inline bool cross_is_new(CrossPosInRoad /*pos*/, Point const& p, RoadSegment const& /*r*/, DBFHandle /*handle*/, int /*road_index*/){
        return saved.count(p) == 0;
    }

    /// \brief 存储道路索引
    /// \param[in] pos 路口在路段Linestring的位置
    /// \param[in] p   路口的几何点
    /// \param[in] cross_index 存储路口的索引
    /// \param[in] r   路口所连的路段
    /// \param[in] handle DBFHandle
    /// \param[in] road_index 道路对应的索引
    inline void store_index(CrossPosInRoad /*pos*/, Point const& p, int cross_index, RoadSegment const& /*r*/, DBFHandle /*handle*/, int /*road_index*/){
        saved[p] = cross_index;

    }

    /// \brief 获取路口的索引
    /// \param[in] pos 路口在路段Linestring的位置
    /// \param[in] p   路口的几何点
    /// \param[in] r   路口所连的路段
    /// \param[in] handle DBFHandle
    /// \param[in] road_index 道路对应的索引
    /// \return 路口的索引
    inline int get_index(CrossPosInRoad /*pos*/, Point const& p, RoadSegment const& /*r*/, DBFHandle /*handle*/, int /*road_index*/){
        return saved[p];
    }
};

/// \brief 什么也不做的属性选取器
///
/// 什么也不做的属性选取器,总是返回双向道路<br/>
/// 默认属性选取器
struct NoPropertiesPicker{
    /// \brief 选取道路的额外属性
    /// \param[out] properties 存储额外的道路属性
    /// \param[in] handle DBFHandle
    /// \param[in] road_index 道路索引
    /// \return Direction 道路的方向
    inline Direction pick_roadsegment(pt::ptree & /*properties*/, DBFHandle /*handle*/, int /*road_index*/){return Bidirection;}

    /// \brief 选取路口的额外属性
    /// \param[out] properties 存储额外的路口属性
    /// \param[in] pos 路口对应在道路的位置
    /// \param[in] handle DBFHandle
    /// \param[in] road_index 道路的索引
    inline void pick_cross(pt::ptree & /*properties*/, CrossPosInRoad /*pos*/,  DBFHandle /*handle*/, int /*road_index*/){}
};

struct RoadIndexCrossIndexPair : std::pair<int, int>{
    RoadIndexCrossIndexPair()=default;
    RoadIndexCrossIndexPair(int r, int c){
        first = r;
        second = c;
    }
    int road_index()const{
        return this->first;
    }
    int cross_index()const{
        return this->second;
    }
    void set_road_index(int idx){
        this->first = idx;
    }
    void set_cross_index(int idx){
        this->second = idx;
    }
};

template<typename PropertyValueType>
struct PropertyIndexMap{
    typedef std::unordered_map<typename CharSequenceToStringElseNoChange<PropertyValueType>::type, int> type;
};

/// \brief 道路地图
class Map{
public:

    Map()noexcept:roadsegment_rtree(bgi::dynamic_quadratic(1)),cross_rtree(bgi::dynamic_quadratic(1)) {}

    /// \brief 构建graph
    void build_graph();

    /// \brief 载入shp地图
    /// \param[in] shpFile shp的路径(不包括后缀)
    /// \param[in] picker 自定义的属性选取器,默认无属性双向道路
    /// \param[in] checker 自定义的路口检查器,默认使用geometry检查是否已经添加
    /// \retval true 成功载入地图
    /// \retval false 出现io错误
    template<typename Picker = NoPropertiesPicker, typename Checker = GeometryChecker>
    bool load(std::string const& shpFile, Picker picker = Picker(), Checker checker = Checker());

    /// \brief 映射路口属性到索引
    /// \param[in] property_name 要映射的属性名字
    /// \param typename PropertyValueType 属性对应的数据类型

    /// 映射用户自定义的属性到索引,可以用通过属性名查询路口

    /// \see cross contain_cross
    template<typename PropertyValueType>
    void map_cross_property(std::string const& property_name);
    
    /// \brief 映射路段属性到索引
    /// \param[in] property_name 要映射的属性名字
    /// \param typename PropertyValueType 属性对应的数据类型
    ///
    /// 映射用户自定义的属性到索引,可以用通过属性名查询路段
    /// \see roadsegment contain_roadsegment
    template<typename PropertyValueType>
    void map_roadsegment_property(std::string const& property_name);

    /// \brief 通过索引查询路口是否存在
    inline bool contain_cross(int index)const{
        return index >= 0 && index < (int)cross_.size();
    }

    /// \brief 通过属性查询路口是否存在
    /// \param[in] property_name 映射过的属性名
    /// \param[in] value 属性值
    /// \see map_cross_property
    template<typename T>
    inline bool contain_cross(std::string const& property_name, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        auto iter = cross_property_index_.find(property_name) ;
        if ( iter != cross_property_index_.end() ){
            return b::any_cast<MapType const&>(iter->second).count(value);
        }
        return false;
    }

    ///  \brief 通过索引查询路段是否存在
    inline bool contain_roadsegment(int index)const{
        return index >= 0 && index < (int)roadsegment_.size();
    }

    /// \brief 通过映射过的属性查询路段是否存在
    /// \see mapRoadSegmentProperty
    template<typename T>
    inline bool contain_roadsegment(std::string const& property_name, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        auto iter = roadsegment_property_index_.find(property_name) ;
        if ( iter != roadsegment_property_index_.end() ){
            return b::any_cast<MapType const&>(iter->second).count(value);
        }
        return false;
    }


    ///  \brief 获取路口
    inline Cross const& cross(int index)const{
        return cross_.at(index);
    }

    /// \brief 通过属性获得路口
    ///  \see map_cross_property
    template<typename T>
    inline Cross const& cross(std::string const& property_name, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        return cross_.at(b::any_cast<MapType const&>(cross_property_index_.at(property_name)).at(value));
    }

    /// \brief 获得路段
    inline RoadSegment const& roadsegment(int index)const{
        return roadsegment_.at(index);
    }


    /// \brief 根据属性值获取路段
    /// \see mapRoadSegmentProperty
    template<typename T>
    inline RoadSegment const& roadsegment(std::string const& property_name, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        return roadsegment_.at(b::any_cast<MapType const&>(roadsegment_property_index_.at(property_name)).at(value));
    }

    /// 线段索引RTree
    typedef bgi::rtree<std::pair<Box, int> , bgi::dynamic_quadratic> RoadSegmentRTree;
    /// 点索引RTree
    typedef bgi::rtree<std::pair<Point, int> , bgi::dynamic_quadratic> PointRTree;
    
    struct RoadIndexOfEdgeTag{
        typedef b::edge_property_tag kind;
    };
    static RoadIndexOfEdgeTag IndexOfEdge;

    /// graph 内部保存边对应的RoadSegment索引
    typedef b::property<RoadIndexOfEdgeTag, int> RoadIndexProperty;

    /// graph
    typedef b::adjacency_list<b::vecS, b::vecS, b::directedS, b::no_property, RoadIndexProperty> Graph;
    typedef b::graph_traits<Graph> GraphTraits;
    RoadSegmentRTree roadsegment_rtree;///< 路段空间索引
    PointRTree cross_rtree;///< 路口空间索引

    Graph graph;///< graph

    /// \brief 访问所有的路段
    void visit_roadsegment(std::function<void(RoadSegment const&r)> visitor)const;
    /// \brief 访问所有的路段
    void visit_roadsegment(std::function<void(RoadSegment &r)> visitor);
    /// \brief 访问所有的边
    void visit_edge(std::function<void(RoadSegment const&r, GraphTraits::edge_descriptor, Graph const& g)> visitor)const;
    /// \brief 访问所有的边
    void visit_edge(std::function<void(RoadSegment &r, GraphTraits::edge_descriptor, Graph &)> visitor);

    /// \brief 访问所有路欧
    void visit_cross(std::function<void(Cross const&)> visitor)const;
    /// \brief 访问所有路口
    void visit_cross(std::function<void(Cross &)> visitor);

    /// \brief 获得驶入道路
    inline std::vector<RoadIndexCrossIndexPair> const& predecessor_road_of(int cross_index)const{
        return predecessor_road_.at(cross_index);
    }


    /// \brief 获得驶出道路
    inline std::vector<RoadIndexCrossIndexPair> const& successor_road_of(int cross_index)const{
        return successor_road_.at(cross_index);
    }

    /// \brief  获得更具edge获得路段
    inline RoadSegment const& roadsegment( GraphTraits::edge_descriptor const& edge)const{
        int roadIdx = b::get(IndexOfEdge, graph, edge);
        return roadsegment_.at(roadIdx);
    }

    /// \brief 搜索start->end的路段
    inline RoadIndexCrossIndexPair find_road_oneway(int start, int end)const{
        if ( start > 0 || start < (int)cross_.size())
        {
            for ( RoadIndexCrossIndexPair  const& p : successor_road_.at(start) ){
                if ( end == p.cross_index() ){
                    return p;
                }
            }
        }
        RoadIndexCrossIndexPair p;
        p.first = p.second = -1;
        return p;
    }

    /// \brief 查找start->end或者end->start的路段
    inline RoadIndexCrossIndexPair findRoadBidrection(int start, int end)const{
        RoadIndexCrossIndexPair rst = find_road_oneway(start, end);
        if ( rst.road_index() == -1 ){
            return find_road_oneway(end, start);
        }
        return rst;
    }

    size_t road_size()const{
        return roadsegment_.size();
    }
    size_t cross_size()const{
        return cross_.size();
    }
    size_t edge_size()const{
        return b::num_edges(graph);
    }
    virtual ~Map()=default;

private:
    void index_();
    std::unordered_map<std::string, b::any> cross_property_index_;
    std::unordered_map<std::string, b::any> roadsegment_property_index_;
    std::vector<RoadSegment> roadsegment_;
    std::vector<Cross> cross_;
    std::vector<std::vector<RoadIndexCrossIndexPair> >successor_road_;
    std::vector<std::vector<RoadIndexCrossIndexPair> >predecessor_road_;
    std::unordered_multimap<int, GraphTraits::edge_descriptor> road_edge_map_;
};


//template impl
struct ShpFileOpenHelper{
    SHPHandle hShp;
    DBFHandle hDbf;
    ShpFileOpenHelper (std::string const& name){
        std::string shpFile = name + ".shp";
        std::string dbfFile = name + ".dbf";
        hShp = SHPOpen(shpFile.c_str(), "r");
        hDbf = DBFOpen(dbfFile.c_str(), "r");
    }

    ~ShpFileOpenHelper(){
        if ( hShp ) SHPClose(hShp);
        if ( hDbf ) DBFClose(hDbf);
    }

    inline operator void*(){
        return (void*) (hShp && hDbf);
    }
};


template<typename Picker, typename Checker>
bool Map::load(std::string const& shp,  Picker picker, Checker checker){
    ShpFileOpenHelper helper(shp);
    if ( ! helper ){
        return false;
    }

    int n_road = DBFGetRecordCount(helper.hDbf);
    roadsegment_.reserve(n_road);
    for(int i = 0; i < n_road; ++i){
        SHPObject* linestring = SHPReadObject(helper.hShp, i);
        RoadSegment r;
        int n = linestring->nVertices;
        r.geometry.reserve(n);
        for(int i = 0; i < n; ++i){
            r.geometry.push_back({linestring->padfX[i], linestring->padfY[i]});
        }
        SHPDestroyObject(linestring);
        r.direction = picker.pick_roadsegment(r.properties, helper.hDbf, i);

        Point s = r.geometry.front();
        Point e = r.geometry.back();
        unsigned int startIndex, endIndex;
        if ( checker.cross_is_new(Front, s, r, helper.hDbf, i) ){
            Cross front;
            front.geometry = s;
            picker.pick_cross(Front, front.properties, helper.hDbf, i);
            startIndex = front.index = cross_.size();
            cross_.push_back(std::move(front));
            checker.store_index(Front, s, startIndex, r, helper.hDbf, i);
        }else{
            startIndex = checker.get_index(Front, s, r, helper.hDbf, i);
        }

        if ( checker.cross_is_new(Back, e, r, helper.hDbf, i) ){
            Cross back;
            back.geometry = e;
            picker.pick_cross(Back, back.properties, helper.hDbf, i);
            endIndex = back.index = cross_.size();
            cross_.push_back(std::move(back));
            checker.store_index(Back, e, endIndex, r, helper.hDbf, i);
        }else{
            endIndex = checker.get_index(Back, e, r, helper.hDbf, i);
        }

        r.start_cross_index = startIndex;
        r.end_cross_index = endIndex;
        r.index = roadsegment_.size();
        roadsegment_.push_back(std::move(r));
    }

    index_();
    return true;
}

template<typename PropertyValueType>
void Map::map_cross_property(std::string const& property_name){
    typedef typename PropertyIndexMap<PropertyValueType>::type MapType;
    MapType pimap;
    for(auto& c : cross_){
        pimap[c.properties.get<PropertyValueType>(property_name)] = c.index;
    }
    cross_property_index_.insert({property_name, b::any(std::move(pimap))});
}

template<typename PropertyValueType>
void Map::map_roadsegment_property(std::string const& property_name){
    typename PropertyIndexMap<PropertyValueType>::type pimap;
    for(auto& r : roadsegment_){
        pimap[r.properties.get<PropertyValueType>(property_name)] = r.index;
    }
    roadsegment_property_index_.insert({property_name, boost::any(std::move(pimap))});
}

#endif  /*GENERAL_MAP*/
