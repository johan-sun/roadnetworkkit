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
    int startCrossIndex;
    int endCrossIndex;
    Direction direction;
    Linestring geometry;
    pt::ptree properties;///< user stored properties
};

/// \brief 通过几何来判断顶点是否添加
/// 
/// Checker通过 crossIsNew 判断路口是否是新的,
/// 若路口是新的则会通过 storeIndex 存储该点对应的索引
/// 若路口已经添加过,则通过 getIndex 获取路口的索引<br/>
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
    std::unordered_map<Point, int, PointHash, PointEqual> savedPoint;
public:

    /// \brief 判断是否是新添加路口
    /// \param[in] pos 路口在路段Linestring的位置
    /// \param[in] p   路口的几何点
    /// \param[in] r   路口所连的路段
    /// \param[in] handle DBFHandle
    /// \param[in] roadIndex 道路对应的索引
    /// \return  是否是未添加索引的点
    inline bool crossIsNew(CrossPosInRoad /*pos*/, Point const& p, RoadSegment const& /*r*/, DBFHandle /*handle*/, int /*roadIndex*/){
        return savedPoint.count(p) == 0;
    }

    /// \brief 存储道路索引
    /// \param[in] pos 路口在路段Linestring的位置
    /// \param[in] p   路口的几何点
    /// \param[in] crossIndex 存储路口的索引
    /// \param[in] r   路口所连的路段
    /// \param[in] handle DBFHandle
    /// \param[in] roadIndex 道路对应的索引
    inline void storeIndex(CrossPosInRoad /*pos*/, Point const& p, int crossIndex, RoadSegment const& /*r*/, DBFHandle /*handle*/, int /*roadIndex*/){
        savedPoint[p] = crossIndex;

    }

    /// \brief 获取路口的索引
    /// \param[in] pos 路口在路段Linestring的位置
    /// \param[in] p   路口的几何点
    /// \param[in] r   路口所连的路段
    /// \param[in] handle DBFHandle
    /// \param[in] roadIndex 道路对应的索引
    /// \return 路口的索引
    inline int getIndex(CrossPosInRoad /*pos*/, Point const& p, RoadSegment const& /*r*/, DBFHandle /*handle*/, int /*roadIndex*/){
        return savedPoint[p];
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
    /// \param[in] roadIndex 道路索引
    /// \return Direction 道路的方向
    inline Direction pickRoadsegment(pt::ptree & /*properties*/, DBFHandle /*handle*/, int /*roadIndex*/){return Bidirection;}

    /// \brief 选取路口的额外属性
    /// \param[out] properties 存储额外的路口属性
    /// \param[in] pos 路口对应在道路的位置
    /// \param[in] handle DBFHandle
    /// \param[in] roadIndex 道路的索引
    inline void pickCross(pt::ptree & /*properties*/, CrossPosInRoad /*pos*/,  DBFHandle /*handle*/, int /*roadIndex*/){}
};

struct RoadIndexCrossIndexPair : std::pair<int, int>{
    RoadIndexCrossIndexPair()=default;
    RoadIndexCrossIndexPair(int r, int c){
        first = r;
        second = c;
    }
    int roadIndex()const{
        return this->first;
    }
    int crossIndex()const{
        return this->second;
    }
    void setRoadIndex(int idx){
        this->first = idx;
    }
    void setCrossIndex(int idx){
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

    Map()noexcept:roadsegmentRTree(bgi::dynamic_quadratic(1)),crossIndex(bgi::dynamic_quadratic(1)) {}

    /// \brief 构建graph
    void buildGraph();

    /// \brief 载入shp地图
    /// \param[in] shpFile shp的路径(不包括后缀)
    /// \param[in] picker 自定义的属性选取器,默认无属性双向道路
    /// \param[in] checker 自定义的路口检查器,默认使用geometry检查是否已经添加
    /// \retval true 成功载入地图
    /// \retval false 出现io错误
    template<typename Picker = NoPropertiesPicker, typename Checker = GeometryChecker>
    bool load(std::string const& shpFile, Picker picker = Picker(), Checker checker = Checker());

    /// \brief 映射路口属性到索引
    /// \param[in] propertyName 要映射的属性名字
    /// \param typename PropertyValueType 属性对应的数据类型

    /// 映射用户自定义的属性到索引,可以用通过属性名查询路口

    /// \see cross containCross
    template<typename PropertyValueType>
    void mapCrossProperty(std::string const& propertyName);
    
    /// \brief 映射路段属性到索引
    /// \param[in] propertyName 要映射的属性名字
    /// \param typename PropertyValueType 属性对应的数据类型
    ///
    /// 映射用户自定义的属性到索引,可以用通过属性名查询路段
    /// \see roadsegment containRoadSegment
    template<typename PropertyValueType>
    void mapRoadsegmentProperty(std::string const& propertyName);

    /// \brief 通过索引查询路口是否存在
    inline bool containCross(int index)const{
        return index >= 0 && index < (int)_cross.size();
    }

    /// \brief 通过属性查询路口是否存在
    /// \param[in] propertyName 映射过的属性名
    /// \param[in] value 属性值
    /// \see mapCrossProperty
    template<typename T>
    inline bool containCross(std::string const& propertyName, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        auto iter = _crossPropertyIndexMap.find(propertyName) ;
        if ( iter != _crossPropertyIndexMap.end() ){
            return b::any_cast<MapType const&>(iter->second).count(value);
        }
        return false;
    }

    ///  \brief 通过索引查询路段是否存在
    inline bool containRoadSegment(int index)const{
        return index >= 0 && index < (int)_roadsegment.size();
    }

    /// \brief 通过映射过的属性查询路段是否存在
    /// \see mapRoadSegmentProperty
    template<typename T>
    inline bool containRoadSegment(std::string const& propertyName, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        auto iter = _roadsegmentPropertyIndexMap.find(propertyName) ;
        if ( iter != _roadsegmentPropertyIndexMap.end() ){
            return b::any_cast<MapType const&>(iter->second).count(value);
        }
        return false;
    }


    ///  \brief 获取路口
    inline Cross const& cross(int index)const{
        return _cross.at(index);
    }

    /// \brief 通过属性获得路口
    ///  \see mapCrossProperty
    template<typename T>
    inline Cross const& cross(std::string const& propertyName, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        return _cross.at(b::any_cast<MapType const&>(_crossPropertyIndexMap.at(propertyName)).at(value));
    }

    /// \brief 获得路段
    inline RoadSegment const& roadsegment(int index)const{
        return _roadsegment.at(index);
    }


    /// \brief 根据属性值获取路段
    /// \see mapRoadSegmentProperty
    template<typename T>
    inline RoadSegment const& roadsegment(std::string const& propertyName, T const& value)const{
        typedef typename PropertyIndexMap<T>::type MapType;
        return _roadsegment.at(b::any_cast<MapType const&>(_roadsegmentPropertyIndexMap.at(propertyName)).at(value));
    }

    /// 线段索引RTree
    typedef bgi::rtree<std::pair<Box, int> , bgi::dynamic_quadratic> RoadSegmentRTree;
    /// 点索引RTree
    typedef bgi::rtree<std::pair<Point, int> , bgi::dynamic_quadratic> PointRTree;
    
    struct RoadIndexOfEdgeTag{
        typedef b::edge_property_tag kind;
    };
    static RoadIndexOfEdgeTag roadIndexOfEdgeTag;

    /// graph 内部保存边对应的RoadSegment索引
    typedef b::property<RoadIndexOfEdgeTag, int> RoadIndexProperty;

    /// graph
    typedef b::adjacency_list<b::vecS, b::vecS, b::directedS, b::no_property, RoadIndexProperty> Graph;
    typedef b::graph_traits<Graph> GraphTraits;
    RoadSegmentRTree roadsegmentRTree;///< 路段空间索引
    PointRTree crossIndex;///< 路口空间索引

    Graph graph;///< graph

    /// \brief 访问所有的路段
    void visitRoadSegment(std::function<void(RoadSegment const&r)> visitor)const;
    /// \brief 访问所有的路段
    void visitRoadSegment(std::function<void(RoadSegment &r)> visitor);
    /// \brief 访问所有的边
    void visitEdge(std::function<void(RoadSegment const&r, GraphTraits::edge_descriptor, Graph const& g)> visitor)const;
    /// \brief 访问所有的边
    void visitEdge(std::function<void(RoadSegment &r, GraphTraits::edge_descriptor, Graph &)> visitor);

    /// \brief 访问所有路欧
    void visitCross(std::function<void(Cross const&)> visitor)const;
    /// \brief 访问所有路口
    void visitCross(std::function<void(Cross &)> visitor);

    /// \brief 获得驶入道路
    inline std::vector<RoadIndexCrossIndexPair> const& inRoadOf(int crossIndex)const{
        return _inRoad.at(crossIndex);
    }


    /// \brief 获得驶出道路
    inline std::vector<RoadIndexCrossIndexPair> const& outRoadOf(int crossIndex)const{
        return _outRoad.at(crossIndex);
    }

    /// \brief  获得路段
    inline RoadSegment const& roadsegment( GraphTraits::edge_descriptor const& edge)const{
        int roadIdx = b::get(roadIndexOfEdgeTag, graph, edge);
        return _roadsegment.at(roadIdx);
    }

    inline RoadIndexCrossIndexPair findRoadForward(int start, int end)const{
        if ( start > 0 || start < (int)_cross.size())
        {
            for ( RoadIndexCrossIndexPair  const& p : _outRoad.at(start) ){
                if ( end == p.crossIndex() ){
                    return p;
                }
            }
        }
        RoadIndexCrossIndexPair p;
        p.first = p.second = -1;
        return p;
    }

    inline RoadIndexCrossIndexPair findRoadBidrection(int start, int end)const{
        RoadIndexCrossIndexPair rst = findRoadForward(start, end);
        if ( rst.roadIndex() == -1 ){
            return findRoadForward(end, start);
        }
        return rst;
    }
    virtual ~Map()=default;

private:
    void _index();
    std::unordered_map<std::string, b::any> _crossPropertyIndexMap;
    std::unordered_map<std::string, b::any> _roadsegmentPropertyIndexMap;
    std::vector<RoadSegment> _roadsegment;
    std::vector<Cross> _cross;
    std::vector<std::vector<RoadIndexCrossIndexPair> >_outRoad;
    std::vector<std::vector<RoadIndexCrossIndexPair> >_inRoad;
    std::unordered_multimap<int, GraphTraits::edge_descriptor> _roadEdgeMap;
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

    int roadSegmentCount = DBFGetRecordCount(helper.hDbf);
    _roadsegment.reserve(roadSegmentCount);
    for(int i = 0; i < roadSegmentCount; ++i){
        SHPObject* linestring = SHPReadObject(helper.hShp, i);
        RoadSegment r;
        int n = linestring->nVertices;
        r.geometry.reserve(n);
        for(int i = 0; i < n; ++i){
            r.geometry.push_back({linestring->padfX[i], linestring->padfY[i]});
        }
        SHPDestroyObject(linestring);
        r.direction = picker.pickRoadsegment(r.properties, helper.hDbf, i);

        Point s = r.geometry.front();
        Point e = r.geometry.back();
        unsigned int startIndex, endIndex;
        if ( checker.crossIsNew(Front, s, r, helper.hDbf, i) ){
            Cross front;
            front.geometry = s;
            picker.pickCross(Front, front.properties, helper.hDbf, i);
            startIndex = front.index = _cross.size();
            _cross.push_back(std::move(front));
            checker.storeIndex(Front, s, startIndex, r, helper.hDbf, i);
        }else{
            startIndex = checker.getIndex(Front, s, r, helper.hDbf, i);
        }

        if ( checker.crossIsNew(Back, e, r, helper.hDbf, i) ){
            Cross back;
            back.geometry = e;
            picker.pickCross(Back, back.properties, helper.hDbf, i);
            endIndex = back.index = _cross.size();
            _cross.push_back(std::move(back));
            checker.storeIndex(Back, e, endIndex, r, helper.hDbf, i);
        }else{
            endIndex = checker.getIndex(Back, e, r, helper.hDbf, i);
        }

        r.startCrossIndex = startIndex;
        r.endCrossIndex = endIndex;
        r.index = _roadsegment.size();
        _roadsegment.push_back(std::move(r));
    }

    _index();
    return true;
}

template<typename PropertyValueType>
void Map::mapCrossProperty(std::string const& propertyName){
    typedef typename PropertyIndexMap<PropertyValueType>::type MapType;
    MapType pimap;
    for(auto& c : _cross){
        pimap[c.properties.get<PropertyValueType>(propertyName)] = c.index;
    }
    _crossPropertyIndexMap.insert({propertyName, b::any(std::move(pimap))});
}

template<typename PropertyValueType>
void Map::mapRoadsegmentProperty(std::string const& propertyName){
    typename PropertyIndexMap<PropertyValueType>::type pimap;
    for(auto& r : _roadsegment){
        pimap[r.properties.get<PropertyValueType>(propertyName)] = r.index;
    }
    _roadsegmentPropertyIndexMap.insert({propertyName, boost::any(std::move(pimap))});
}

#endif  /*GENERAL_MAP*/
