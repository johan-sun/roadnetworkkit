#ifndef  MAP_GRAPH_PROPERTY_HPP
#define  MAP_GRAPH_PROPERTY_HPP
//auto-include {{{
#include  <boost/property_map/property_map.hpp>
#include  <string>
#include  <type_traits>
#include  <iostream>
#include    "generalmap.h"
//}}}

///\breif 地图属性的edge property map
///
///根据地图属性包装一个property map
///可以用于BGL
template<typename PropertyValue>
class PropertyMapFromRoadProperty{
public:
    typedef Map::GraphTraits::edge_descriptor key_type;
    typedef typename CharSequenceToStringElseNoChange<PropertyValue>::type value_type;
    typedef boost::readable_property_map_tag category;
    ///\breif 构建proerpty map
    ///
    ///\param map 被包装的map
    ///\param name 地图属性的字段
    PropertyMapFromRoadProperty(Map const& map, std::string const& name):_map(map),_name(name){}

    value_type get(key_type const& key)const{
        RoadSegment const& seg = _map.roadsegment(key);
        return seg.properties.get<value_type>(_name);
    }
private:
    Map const& _map;
    std::string _name;
};

template<typename T>
typename PropertyMapFromRoadProperty<T>::value_type
get(PropertyMapFromRoadProperty<T> const& map, typename PropertyMapFromRoadProperty<T>::key_type const& key){
    return map.get(key);
}

template<typename PropertyValue>
class PropertyMapGen{
public:
    typedef Map::GraphTraits::edge_descriptor key_type;
    typedef typename CharSequenceToStringElseNoChange<PropertyValue>::type value_type;
    typedef boost::readable_property_map_tag category;
    template<typename Fun>
    PropertyMapGen(Map const& map, Fun fun):_map(map), _fun(fun){}

    value_type get(key_type const& key)const{
        RoadSegment const& seg = _map.roadsegment(key);
        return _fun(key, seg);
    }
private:
    Map const& _map;
    std::function<value_type(key_type const&, RoadSegment const&)> _fun;
};


template<typename T>
typename PropertyMapGen<T>::value_type
get(PropertyMapGen<T> const& map, typename PropertyMapGen<T>::key_type const& k){
    return map.get(k);
}

namespace boost{
template<typename T>
struct property_traits<PropertyMapFromRoadProperty<T> >{
    typedef typename PropertyMapFromRoadProperty<T>::key_type key_type;
    typedef typename PropertyMapFromRoadProperty<T>::value_type value_type;
    typedef typename PropertyMapFromRoadProperty<T>::category category;
};

template<typename T>
struct property_traits<PropertyMapGen<T> >{
    typedef typename PropertyMapGen<T>::key_type key_type;
    typedef typename PropertyMapGen<T>::value_type value_type;
    typedef typename PropertyMapGen<T>::category category;

};
}
#endif  /*MAP_GRAPH_PROPERTY_HPP*/
