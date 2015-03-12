#ifndef  TIME_ESTIMATE_H
#define  TIME_ESTIMATE_H
//auto-include {{{
#include <vector>
#include <utility>
#include <string>
#include "gps.h"
#include "roadmap.h"
//}}}

/// TimedCrossIndex
typedef std::pair<int, boost::posix_time::ptime> TimedCrossIndex;

///\brief 估计时间
///\param log
///\param paths ivmm 生成的path向量
///\param logRange 只估计[first, second)区间的GPS数据
///\param 对应的道路地图
std::vector<TimedCrossIndex> estimateTime(
    std::vector<GpsPoint> const& log,
    std::vector<Path> const& paths,
    std::pair<int, int> const& logRange,
    RoadMap const& map);

///\brief 估计时间,使用全部GPS点的版本
inline std::vector<TimedCrossIndex> estimateTime(
    std::vector<GpsPoint> const& log,
    std::vector<Path> const& paths,
    RoadMap const& map){
    return estimateTime(log, paths, {0, log.size()}, map);
}

std::vector<TimedCrossIndex> loadTimedPathFromFile(std::string const& file);



#endif  /*TIME_ESTIMATE_H*/
