#ifndef  BJ_ROAD_EPSG3785_H
#define  BJ_ROAD_EPSG3785_H
#include  <string>
#include    "generalmap.h"
struct BJRoadEpsg3785CrossIDChecker{
    const static int FrontCrossIDField = 18;
    const static int BackCrossIDField = 19;
    bool cross_is_new(CrossPosInRoad pos, Point const& /*p*/, RoadSegment const&/*r*/ , DBFHandle handle, int roadIndex){
        int Field = pos == Front ? FrontCrossIDField : BackCrossIDField;
        std::string id = DBFReadStringAttribute(handle, roadIndex, Field);
        return IDIndexMap.count(id) == 0;
    }

    void store_index(CrossPosInRoad pos, Point const& /*p*/, int crossIndex, RoadSegment const& /*r*/, DBFHandle handle, int roadIndex){
        int Field = pos == Front ? FrontCrossIDField : BackCrossIDField;
        std::string id = DBFReadStringAttribute(handle, roadIndex, Field);
        IDIndexMap[id] = crossIndex;
    }

    int get_index(CrossPosInRoad pos, Point const& /*p*/, RoadSegment const& /*r*/, DBFHandle handle, int roadIndex){
        int Field = pos == Front ? FrontCrossIDField : BackCrossIDField;
        std::string id = DBFReadStringAttribute(handle, roadIndex, Field);
        return IDIndexMap.at(id);
    }

    std::unordered_map<std::string, int> IDIndexMap;
};

struct BJRoadEpsg3785IDPicker{
    const static int FrontCrossIDField = 18;
    const static int BackCrossIDField = 19;
    const static int IDField = 2;
    const static int DirectionField = 6;
    const static int PathClassField = 20;
    Direction pick_roadsegment(pt::ptree & properties, DBFHandle handle, int roadIndex){
        std::string ID = DBFReadStringAttribute(handle, roadIndex, IDField);
        int direction = atoi(DBFReadStringAttribute(handle, roadIndex, DirectionField));
        int path_class = atoi(DBFReadStringAttribute(handle, roadIndex, PathClassField));
        double speed = 30 / 3.6;
        switch(path_class){
            case 4:
                speed = 70 / 3.6;
                break;
            case 3:
                speed = 50 / 3.6;
                break;
            case 2:
                speed = 40 / 3.6;
        }
        properties.put("SPEED", speed);
        properties.put("SID", ID);
        enum Direction dir = Bidirection;
        if ( direction == 2 ){
            dir = Forward;
        }else if ( direction == 3 ){
            dir = Backward;
        }
        return dir;
    }
    void pick_cross(CrossPosInRoad pos, pt::ptree& properties, DBFHandle handle, int roadIndex){
        int Field = pos == Front ? FrontCrossIDField : BackCrossIDField;
        std::string id = DBFReadStringAttribute(handle, roadIndex, Field);
        properties.put("SID", id);
    }
};

#endif  /*BJ_ROAD_EPSG3785_H*/
