#define BOOST_TEST_MODULE roadnetworkkit_ivmm

#include <boost/test/unit_test.hpp>

#include <shapefil.h>
#include <boost/range/algorithm.hpp>
#include <vector>
#include <fstream>
#include "roadmap.h"
#include "gps.h"
#include "bj-road-epsg3785/bj_road_epsg3785.h"
#include "ivmm/ivmm.h"
#include "util.h"
#include "time_estimate.h"
#include "sutil/boost/date_time/date_time_format.hpp"
using namespace std;

BOOST_AUTO_TEST_SUITE(ivmmtest)

    RoadMap bjRoad;

    void drawCandidatesAndN(char const* output, IVMM::VVector<Candidate> const& cand, IVMM::VVector<double> const& n){
        SHPHandle shp = SHPCreate(output, SHPT_POINT);
        DBFHandle dbf = DBFCreate(output);
        DBFAddField(dbf, "GPSID", FTInteger, 10, 0);
        DBFAddField(dbf, "CID", FTInteger, 10, 0);
        DBFAddField(dbf, "N", FTDouble, 12, 6);
        DBFAddField(dbf, "VOTE", FTInteger, 10, 0);
        DBFAddField(dbf, "FVALUE", FTDouble, 12, 6);

        for(int i = 0; i < cand.size(); ++i){
            for(int j = 0; j < cand[i].size(); ++j){
                double x = cand[i][j].point.geometry.x();
                double y = cand[i][j].point.geometry.y();
                SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
                int id = SHPWriteObject(shp, -1, point);
                DBFWriteIntegerAttribute(dbf, id, 0, i);
                DBFWriteIntegerAttribute(dbf, id, 1, j);
                DBFWriteDoubleAttribute(dbf, id, 2, n[i][j]);
                DBFWriteIntegerAttribute(dbf, id, 3, cand[i][j].vote);
                DBFWriteDoubleAttribute(dbf, id, 4, cand[i][j].fvalue);
            }
        }

        SHPClose(shp);
        DBFClose(dbf);
    }

    void drawPaths(char const* output, IVMM::VVVector<Path> const& paths,IVMM::VVVector<Detail> const& details, IVMM::VVector<double> const& n){
        SHPHandle shp = SHPCreate(output, SHPT_ARC);
        DBFHandle dbf = DBFCreate(output);
        DBFAddField(dbf, "ID", FTInteger, 10, 0);
        DBFAddField(dbf, "FROMGPS", FTInteger, 10,0);
        DBFAddField(dbf, "FROMCAND", FTInteger, 10, 0);
        DBFAddField(dbf, "TOGPS", FTInteger, 10, 0);
        DBFAddField(dbf, "TOCAND", FTInteger, 10, 0);
        DBFAddField(dbf, "LENGTH", FTDouble, 10, 3);
        DBFAddField(dbf, "GPS_DISTANCE", FTDouble, 10, 3);
        DBFAddField(dbf, "V", FTDouble, 12,6);
        DBFAddField(dbf, "GPS_TIME_INTERVAL", FTInteger, 10, 0);
        DBFAddField(dbf, "AVERAGE_SPEED", FTDouble, 12, 6);
        DBFAddField(dbf, "WEIGHT_SPEED", FTDouble, 12,6);
        DBFAddField(dbf, "FT", FTDouble, 12,6);
        DBFAddField(dbf, "FSFT", FTDouble, 12,6);
        DBFAddField(dbf, "N", FTDouble, 12, 6);

        int ID = 0;
        for(int g = 0 ; g < paths.size(); ++g){
            for(int c1 = 0; c1 < paths[g].size(); ++c1){
                for(int c2 = 0; c2 < paths[g][c1].size(); ++c2){
                    Detail const& detail = details[g][c1][c2];
                    Path const& path = paths[g][c1][c2];
                    Linestring line = geometry(path, bjRoad);
                    double x[line.size()];
                    double y[line.size()];
                    b::transform(line, x, [](Point const& p){ return p.x();});
                    b::transform(line, y, [](Point const& p){ return p.y();});
                    SHPObject* l = SHPCreateSimpleObject(SHPT_ARC, line.size() , x, y, nullptr);
                    int id = SHPWriteObject(shp, -1, l);
                    DBFWriteIntegerAttribute(dbf, id, 0, ID++);
                    DBFWriteIntegerAttribute(dbf, id, 1, g);
                    DBFWriteIntegerAttribute(dbf, id, 2, c1);
                    DBFWriteIntegerAttribute(dbf, id, 3, g+1);
                    DBFWriteIntegerAttribute(dbf, id, 4, c2);
                    DBFWriteDoubleAttribute(dbf, id, 5, detail.pathLength);
                    DBFWriteDoubleAttribute(dbf, id, 6, detail.twoGpsDistance);
                    DBFWriteDoubleAttribute(dbf, id, 7, detail.v);
                    DBFWriteIntegerAttribute(dbf, id, 8, detail.timeInteval);
                    DBFWriteDoubleAttribute(dbf, id, 9, detail.avgSpeed);
                    DBFWriteDoubleAttribute(dbf, id, 10, detail.weightSpeed);
                    DBFWriteDoubleAttribute(dbf, id, 11, detail.ft);
                    DBFWriteDoubleAttribute(dbf, id, 12, detail.v * detail.ft * n[g+1][c2]);
                    DBFWriteDoubleAttribute(dbf, id, 13, n[g+1][c2]);
                    SHPDestroyObject(l);
                }
            }
        }
        DBFClose(dbf);
        SHPClose(shp);
    }

    void drawPaths(char const* output,vector<int> const& finalCand, IVMM::VVVector<Path> const& paths,IVMM::VVVector<Detail> const& details, IVMM::VVector<double> const& n){
        SHPHandle shp = SHPCreate(output, SHPT_ARC);
        DBFHandle dbf = DBFCreate(output);
        DBFAddField(dbf, "ID", FTInteger, 10, 0);
        DBFAddField(dbf, "FROMGPS", FTInteger, 10,0);
        DBFAddField(dbf, "FROMCAND", FTInteger, 10, 0);
        DBFAddField(dbf, "TOGPS", FTInteger, 10, 0);
        DBFAddField(dbf, "TOCAND", FTInteger, 10, 0);
        DBFAddField(dbf, "LENGTH", FTDouble, 10, 3);
        DBFAddField(dbf, "GPS_DISTANCE", FTDouble, 10, 3);
        DBFAddField(dbf, "V", FTDouble, 12,6);
        DBFAddField(dbf, "GPS_TIME_INTERVAL", FTInteger, 10, 0);
        DBFAddField(dbf, "AVERAGE_SPEED", FTDouble, 12, 6);
        DBFAddField(dbf, "WEIGHT_SPEED", FTDouble, 12,6);
        DBFAddField(dbf, "FT", FTDouble, 12,6);
        DBFAddField(dbf, "FSFT", FTDouble, 12,6);
        DBFAddField(dbf, "N", FTDouble, 12, 6);

        int ID = 0;
        for(int g = 0 ; g < paths.size(); ++g){
            for(int c1 = 0; c1 < paths[g].size(); ++c1){
                for(int c2 = 0; c2 < paths[g][c1].size(); ++c2){
                    if ( c1 != finalCand[g] || c2 != finalCand[g+1])
                        continue;
                    Detail const& detail = details[g][c1][c2];
                    Path const& path = paths[g][c1][c2];
                    Linestring line = geometry(path, bjRoad);
                    double x[line.size()];
                    double y[line.size()];
                    b::transform(line, x, [](Point const& p){ return p.x();});
                    b::transform(line, y, [](Point const& p){ return p.y();});
                    SHPObject* l = SHPCreateSimpleObject(SHPT_ARC, line.size() , x, y, nullptr);
                    int id = SHPWriteObject(shp, -1, l);
                    DBFWriteIntegerAttribute(dbf, id, 0, ID++);
                    DBFWriteIntegerAttribute(dbf, id, 1, g);
                    DBFWriteIntegerAttribute(dbf, id, 2, c1);
                    DBFWriteIntegerAttribute(dbf, id, 3, g+1);
                    DBFWriteIntegerAttribute(dbf, id, 4, c2);
                    DBFWriteDoubleAttribute(dbf, id, 5, detail.pathLength);
                    DBFWriteDoubleAttribute(dbf, id, 6, detail.twoGpsDistance);
                    DBFWriteDoubleAttribute(dbf, id, 7, detail.v);
                    DBFWriteIntegerAttribute(dbf, id, 8, detail.timeInteval);
                    DBFWriteDoubleAttribute(dbf, id, 9, detail.avgSpeed);
                    DBFWriteDoubleAttribute(dbf, id, 10, detail.weightSpeed);
                    DBFWriteDoubleAttribute(dbf, id, 11, detail.ft);
                    DBFWriteDoubleAttribute(dbf, id, 12, detail.v * detail.ft * n[g+1][c2]);
                    DBFWriteDoubleAttribute(dbf, id, 13, n[g+1][c2]);
                    SHPDestroyObject(l);
                }
            }
        }
        DBFClose(dbf);
        SHPClose(shp);
    }

    void drawTimedCross(char const* output, vector<TimedCrossIndex> const& timedPath){
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

    void drawSegmentCost(char const* output, vector<TimedCrossIndex> const& timedPath){
        SHPHandle shp = SHPCreate(output, SHPT_ARC);
        DBFHandle dbf = DBFCreate(output);
        DBFAddField(dbf, "COST", FTInteger, 10,0);
        for(int i = 1; i < timedPath.size(); ++i){
            int ms = (timedPath[i].second - timedPath[i-1].second).total_milliseconds();
            double x[2];
            double y[2];
            x[0] = bjRoad.cross(timedPath[i-1].first).geometry.x();
            y[0] = bjRoad.cross(timedPath[i-1].first).geometry.y();
            x[1] = bjRoad.cross(timedPath[i].first).geometry.x();
            y[1] = bjRoad.cross(timedPath[i].first).geometry.y();
            SHPObject * line = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
            int id = SHPWriteObject(shp, -1, line);
            DBFWriteIntegerAttribute(dbf, id, 0, ms);
            SHPDestroyObject(line);
        }
        SHPClose(shp);
        DBFClose(dbf);
    }

    BOOST_AUTO_TEST_CASE(ivmm) {
        BOOST_REQUIRE(bjRoad.load("../data/map/bj-road-epsg3785", BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker()));
        IVMMParam param;
        param.candidateLimit = 5;
        param.candidateQueryRadious = 100;
        param.projectDistMean = 5;
        param.projectDistStddev = 20;
        param.window = 50;
        param.beta = 5000;
        IVMM ivmm(&bjRoad, param);

        //vector<GpsPoint> log = loadFromFile("../data/bj-gps-after-prepare/20121101/96894/96894-0.txt");
        //vector<GpsPoint> log = loadFromFile("../data/bj-gps-after-prepare/20121101/21004/21004-0.txt");
        //vector<GpsPoint> log = loadFromFile("../data/bj-gps-after-prepare/20121101/102734/102734-0.txt");
        vector<GpsPoint> log = loadFromFile("../data/bj-gps-after-prepare/20121101/100064/100064-4.txt");

        //IVMM::VVector<Candidate> candidates;
        //IVMM::VVector<double> n;
        //IVMM::VVVector<Detail> details;
        //IVMM::VVVector<Path> paths;
        //vector<int> finalCand;
        //cout << ivmm.mapMatch(log, n, details, paths, candidates, finalCand) << endl;
        //drawCandidatesAndN("CN", candidates, n);
        //drawPaths("PATH", paths, details, n);
        //drawPaths("FINAL_PATH_IVMM", finalCand, paths, details, n);
        //vector<Path> finalPath;
        //for(int i = 0; i < paths.size(); ++i){
        //    int srcBest = finalCand[i];
        //    int destBest = finalCand[i+1];
        //    finalPath.push_back(paths[i][srcBest][destBest]);
        //}
        //paths.clear();
        //vector<TimedCrossIndex> timedPath = estimateTime(log, finalPath, bjRoad);
        //drawTimedCross("TIMED_CROSS", timedPath);
        //drawSegmentCost("SEGMENT_COST", timedPath);

        vector<pair<int, int> > ranges;
        auto result = ivmm.mapMatchSafe(log, ranges);
        cout << ranges.size() << endl;
        for(auto& r : ranges){
            cout << r.second - r.first << endl;
        }
        vector<TimedCrossIndex> timedPath = estimateTime(log, result, ranges[0], bjRoad);
        if ( !timedPath.empty()){
            drawTimedCross("TIMED_CROSS_0", timedPath);
            drawSegmentCost("SEGMENT_COST_0", timedPath);
        }
        timedPath = estimateTime(log, result, ranges[1], bjRoad);
        if ( !timedPath.empty()){
            drawTimedCross("TIMED_CROSS_1", timedPath);
            drawSegmentCost("SEGMENT_COST_1", timedPath);
        }
    }

BOOST_AUTO_TEST_SUITE_END()

#include<boost/test/included/unit_test.hpp>
