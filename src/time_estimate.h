#ifndef  TIME_ESTIMATE_H
#define  TIME_ESTIMATE_H
//auto-include {{{
#include <vector>
#include <utility>
#include "gps.h"
#include "roadmap.h"
//}}}

typedef std::pair<int, boost::posix_time::ptime> TimedCrossIndex;


std::vector<TimedCrossIndex> estimateTime(
    std::vector<GpsPoint> const& log,
    std::vector<Path> const& paths,
    std::pair<int, int> const& logRange,
    RoadMap const& map);

inline std::vector<TimedCrossIndex> estimateTime(
    std::vector<GpsPoint> const& log,
    std::vector<Path> const& paths,
    RoadMap const& map){
    return estimateTime(log, paths, {0, log.size()}, map);
}





#endif  /*TIME_ESTIMATE_H*/
