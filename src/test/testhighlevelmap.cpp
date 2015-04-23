#define BOOST_TEST_MODULE testhighlevelmap
//auto-include {{{
#include  <boost/test/unit_test.hpp>
#include <iostream>
#include  <shapefil.h>
#include <fstream>
#include    "path-restore/HighLevelMap.h"
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
        Cross const& c1 = map.cross(p1.crossIndex);
        Cross const& c2 = map.cross(p2.crossIndex);
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
    //BOOST_CHECK_EQUAL(hmap.cross("ID", 91911).index, hmap.roadsegment(0).startCrossIndex);
    //BOOST_CHECK_EQUAL(hmap.cross("ID", 91909).index, hmap.roadsegment(0).endCrossIndex);

    //int newCrossIndex = hmap.addTmpCross(53020, 0, 0).index;
    //BOOST_CHECK_EQUAL( newCrossIndex, hmap.crossSize() );
    //BOOST_CHECK_EQUAL(hmap.crossOrTmpOne(newCrossIndex).properties.get<int>("ID"), 53020);

    //VirtualEdgeWeight w(1000);
    //int newEdgeIndex = hmap.addTmpVirtualEdge(newCrossIndex, hmap.cross("ID", 53019).index, w).index;
    //BOOST_CHECK_EQUAL(newEdgeIndex, hmap.roadSize());
    ///*BOOST_NOEXCEPT_EXPR(*/ cout << hmap.weightOrTmpOne(newEdgeIndex) << endl; /*);*/
    //auto iter = hmap.outRoadIncludeTmp(newCrossIndex);
    //HighLevelMap::OutIter end;
    //bool outIterIsNotEnd = iter != end;
    //BOOST_CHECK(outIterIsNotEnd);
    //BOOST_CHECK_EQUAL(iter->crossIndex(),hmap.cross("ID", 53019).index);
    //BOOST_CHECK_EQUAL(iter->roadIndex() , newEdgeIndex);
    //++iter;
    //bool outIterIsEnd = iter == end;
    //BOOST_CHECK(outIterIsEnd);
    //hmap.clearTmp();

    //int s ,e ;
    //boost::posix_time::time_duration t;
    //int i = 0;
    //while(cin >> s >> t >> e)
    //{
    //    ++i;
    //    auto p = hmap.shortestPath(hmap.cross("ID",s).index, t, hmap.cross("ID",e).index);
    //    cout << p.back().time << ": spend " << p.back().time - p.front().time << endl;
    //    drawpPath(hmap, p, "TIMED-SHORTEST-PATH-" + to_string(i));
    //}
    int s, e;
    boost::posix_time::time_duration t;
    cin >> s >> t >> e;
    KShortestPathGenerator g(hmap, hmap.cross("ID", s).index, t, hmap.cross("ID", e).index);
    int i  = 0;
    while(cin.get() != EOF)
    {
        ++i;
        cout << "next path" << endl;
        auto path = g.nextPath();
        if ( path == nullptr )
            break;
        cout << path->back().time - path->front().time << endl;
        drawpPath(hmap, *path, "TIMED-KSHORTEST-PATH-" + to_string(i));
        ofstream o("P-" + to_string(i));
        for(auto& p : *path)
        {
            o << p.crossIndex << "," << p.time << endl;
        }
        o.close();
    }

}

BOOST_AUTO_TEST_SUITE_END()

#include<boost/test/included/unit_test.hpp>
