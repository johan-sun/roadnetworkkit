#ifndef  TIME_ESTIMATE_H
#define  TIME_ESTIMATE_H
//auto-include {{{
#include <vector>
#include <utility>
#include <string>
#include <mysql++.h>
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
std::vector<TimedCrossIndex> estimate_time(
    std::vector<GpsPoint> const& log,
    std::vector<Path> const& paths,
    std::pair<int, int> const& logRange,
    RoadMap const& map);

///\brief 估计时间,使用全部GPS点的版本
inline std::vector<TimedCrossIndex> estimate_time(
    std::vector<GpsPoint> const& log,
    std::vector<Path> const& paths,
    RoadMap const& map){
    return estimate_time(log, paths, {0, log.size()}, map);
}

std::vector<TimedCrossIndex> load_timed_path_from_file(std::string const& file);
std::vector<TimedCrossIndex> load_timed_path_from_DB(mysqlpp::Connection& con, int metaID);


#endif  /*TIME_ESTIMATE_H*/
