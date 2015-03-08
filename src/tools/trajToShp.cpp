#include  <shapefil.h>
#include  <roadmap.h>
#include <fstream>
#include <string>
#include    "time_estimate.h"
#include    "bj-road-epsg3785/bj_road_epsg3785.h"
#include    "sutil/boost/date_time/date_time_format.hpp"
using namespace std;
void drawTimedCross(char const* output, vector<TimedCrossIndex> const& timedPath, RoadMap const& bjRoad){
    SHPHandle shp = SHPCreate(output, SHPT_POINT);
    DBFHandle dbf = DBFCreate(output);
    DBFAddField(dbf, "INDEX", FTInteger, 10, 0);
    DBFAddField(dbf, "TIME", FTString, 40, 0);
    for(TimedCrossIndex const& idx : timedPath){
        double x = bjRoad.cross(idx.first).geometry.x();
        double y = bjRoad.cross(idx.first).geometry.y();
        SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
        int id = SHPWriteObject(shp, -1, point);
        DBFWriteIntegerAttribute(dbf, id, 0, idx.first);
        DBFWriteStringAttribute(dbf, id , 1, to_format_string(idx.second, "%H:%M:%S%F").c_str());
        SHPDestroyObject(point);
    }
    SHPClose(shp);
    DBFClose(dbf);
}

int main(int argc, char *argv[])
{
    RoadMap bjRoad;
    if (!bjRoad.load(argv[1], BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker()) ){
        return -1;
    }

    vector<TimedCrossIndex> timedPath;
    fstream in(argv[2]);
    if ( ! in  ){
        return -1;
    }

    int i;
    string s;
    while( in >> i && in.ignore() && getline(in, s) ){
        TimedCrossIndex ci;
        ci.first = i;
        ci.second = from_format_string<boost::posix_time::ptime>(s, "%Y-%m-%d %H:%M:%S%F");
        timedPath.push_back(ci);
    }
    drawTimedCross("TIMED_TRAJ", timedPath, bjRoad);
    return 0;
}
