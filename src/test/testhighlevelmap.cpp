#define BOOST_TEST_MODULE testhighlevelmap
//auto-include {{{
#include  <boost/test/unit_test.hpp>
#include <iostream>
#include  <shapefil.h>
#include <fstream>
#include    "path-restore/highlevelmap.h"
//}}}
using namespace std;

BOOST_AUTO_TEST_SUITE(hightlevelmap)
void drawpPath(HighLevelMap const& map, vector<HighLevelMap::TimedCross> const& path, string const& fil)
{
    SHPHandle shp = SHPCreate(fil.c_str(),SHPT_ARC);
    DBFHandle dbf = DBFCreate(fil.c_str());
    DBFAddField(dbf, "INDEX", FTInteger, 10, 0);
    DBFAddField(dbf, "COST", FTString, 20, 0);
    for(size_t i = 1; i < path.size(); ++i)
    {
        auto& p1 = path[i-1];
        auto& p2 = path[i];
        ostringstream os;
        Cross const& c1 = map.cross(p1.cross_index);
        Cross const& c2 = map.cross(p2.cross_index);
        os << (p2.time - p1.time);
        double x[2] = { c1.geometry.x(), c2.geometry.x() };
        double y[2] = { c1.geometry.y(), c2.geometry.y() };
        SHPObject* arc = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, 0);
        const int insert = -1;
        int id = SHPWriteObject(shp, insert , arc );
        DBFWriteStringAttribute(dbf, id, 0, os.str().c_str());
        SHPDestroyObject(arc);
    }
    DBFClose(dbf);
    SHPClose(shp);
}
BOOST_AUTO_TEST_CASE(basic)
{
    HighLevelMap hmap;
    BOOST_REQUIRE( hmap.load("../build/virtualEdgeTop4000Weekdays", "localhost", "root", "19900708") );
    BOOST_CHECK_EQUAL(hmap.cross("ID", 91911).index, hmap.roadsegment(0).start_cross_index);
    BOOST_CHECK_EQUAL(hmap.cross("ID", 91909).index, hmap.roadsegment(0).end_cross_index);

    int newCrossIndex = hmap.add_tmp_cross(53020, 0, 0).index;
    BOOST_CHECK_EQUAL( newCrossIndex, hmap.cross_size() );
    BOOST_CHECK_EQUAL(hmap.cross_or_tmp_one(newCrossIndex).properties.get<int>("ID"), 53020);

    VirtualEdgeWeight w(1000);
    int newEdgeIndex = hmap.add_tmp_virtual_edge(newCrossIndex, hmap.cross("ID", 53019).index, w).index;
    BOOST_CHECK_EQUAL(newEdgeIndex, hmap.road_size());
    ///*BOOST_NOEXCEPT_EXPR(*/ cout << hmap.weightOrTmpOne(newEdgeIndex) << endl; /*);*/
    auto iter = hmap.out_road_include_tmp(newCrossIndex);
    HighLevelMap::OutIter end;
    bool outIterIsNotEnd = iter != end;
    BOOST_CHECK(outIterIsNotEnd);
    BOOST_CHECK_EQUAL(iter->cross_index(),hmap.cross("ID", 53019).index);
    BOOST_CHECK_EQUAL(iter->road_index() , newEdgeIndex);
    ++iter;
    bool outIterIsEnd = iter == end;
    BOOST_CHECK(outIterIsEnd);

    Cross const& c = hmap.add_tmp_cross(1111111, 0, 0);
    BOOST_CHECK_EQUAL(c.properties.get<int>("ID"), 1111111);
    BOOST_CHECK(!hmap.contain_cross(c.index));
    BOOST_CHECK(hmap.contain_cross_inc_tmp(c.index));
    BOOST_CHECK(!hmap.contain_cross("ID", 1111111));
    BOOST_CHECK(hmap.contain_cross_inc_tmp_by_id(1111111));
}

BOOST_AUTO_TEST_SUITE_END()

#include<boost/test/included/unit_test.hpp>
