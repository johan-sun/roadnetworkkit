#define BOOST_TEST_MODULE testhighlevelmap
//auto-include {{{
#include  <boost/test/unit_test.hpp>
#include <iostream>
#include    "path-restore/HighLevelMap.h"
//}}}
using namespace std;
BOOST_AUTO_TEST_SUITE(hightlevelmap)


BOOST_AUTO_TEST_CASE(basic)
{
    HighLevelMap hmap;
    BOOST_REQUIRE( hmap.load("../build/virtualEdgeTop4000Weekdays", "localhost", "root", "19900708") );
    BOOST_CHECK_EQUAL(hmap.cross("ID", 91911).index, hmap.roadsegment(0).startCrossIndex);
    BOOST_CHECK_EQUAL(hmap.cross("ID", 91909).index, hmap.roadsegment(0).endCrossIndex);

    int newCrossIndex = hmap.addTmpCross(53020, 0, 0).index;
    BOOST_CHECK_EQUAL( newCrossIndex, hmap.crossSize() );
    BOOST_CHECK_EQUAL(hmap.crossOrTmpOne(newCrossIndex).properties.get<int>("ID"), 53020);

    VirtualEdgeWeight w(1000);
    int newEdgeIndex = hmap.addTmpVirtualEdge(newCrossIndex, hmap.cross("ID", 53019).index, w).index;
    BOOST_CHECK_EQUAL(newEdgeIndex, hmap.roadSize());
    /*BOOST_NOEXCEPT_EXPR(*/ cout << hmap.weightOrTmpOne(newEdgeIndex) << endl; /*);*/
    auto iter = hmap.outRoadIncludeTmp(newCrossIndex);
    HighLevelMap::OutIter end;
    bool outIterIsNotEnd = iter != end;
    BOOST_CHECK(outIterIsNotEnd);
    BOOST_CHECK_EQUAL(iter->crossIndex(),hmap.cross("ID", 53019).index);
    BOOST_CHECK_EQUAL(iter->roadIndex() , newEdgeIndex);
    ++iter;
    bool outIterIsEnd = iter == end;
    BOOST_CHECK(outIterIsEnd);
}

BOOST_AUTO_TEST_SUITE_END()

#include<boost/test/included/unit_test.hpp>
