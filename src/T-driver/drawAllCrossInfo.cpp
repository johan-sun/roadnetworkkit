//auto-include {{{
#include <shapefil.h>
#include <iostream>
#include "roadmap.h"
#include "bj-road-epsg3785/bj_road_epsg3785.h"
//}}}
using namespace std;
int main(int argc, char *argv[])
{
    RoadMap map;
    if ( ! map.load(argv[1], BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker()) ){
        cerr << "can not load " << argv[1] << endl;
        return 1;
    }

    int cindex;
    int count;
    double entropy;
    SHPHandle shp = SHPCreate("crossInfo", SHPT_POINT);
    DBFHandle dbf = DBFCreate("crossInfo");
    DBFAddField(dbf, "RANK", FTInteger, 10, 0);
    DBFAddField(dbf, "INDEX", FTInteger, 10, 0);
    DBFAddField(dbf, "COUNT", FTInteger, 10, 0);
    DBFAddField(dbf, "ENTROPY", FTDouble, 12, 9);

    int rank = 0;
    while(cin>>cindex && cin.ignore() && cin >> count && cin.ignore() && cin >> entropy)
    {
        double x = map.cross(cindex).geometry.x();
        double y = map.cross(cindex).geometry.y();
        SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
        int id = SHPWriteObject(shp, -1, point);
        DBFWriteIntegerAttribute(dbf, id, 0, rank++);
        DBFWriteIntegerAttribute(dbf, id, 1, cindex);
        DBFWriteIntegerAttribute(dbf, id, 2, count);
        DBFWriteDoubleAttribute(dbf, id, 3, entropy);
        SHPDestroyObject(point);
    }
    SHPClose(shp);
    DBFClose(dbf);
    return 0;
}
