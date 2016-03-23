#ifndef  IVMM_H
#define  IVMM_H
//auto-include {{{
#include <string>
#include <vector>
#include <utility>
#include "roadmap.h"
#include  <boost/optional.hpp>
//}}}


class GpsPoint;

///\brief 候选点
struct Candidate{
    ProjectPoint point;///< 投影点
    int vote;///< 投票
    double fvalue;///< ivmm论文中的favlue
};

///\brief IVMM算法参数
struct IVMMParam{
    double candidate_query_radious;
    int candidate_limit;
    double project_dist_mean;
    double project_dist_stddev;
    double beta;
    int window;
    static boost::optional<IVMMParam> load_config(std::string const& filename);
};

///\brief 计算静态矩阵m[i][j]对应值的具体参数
struct Detail{
    double weight_speed;///< 两候选点之间最短路径的道路加权速度
    double avg_speed;///< 平均估计速度
    double two_gps_distance;///< 两个GPS点之间的直线距离
    double path_length;///< 最短路径长度
    int time_inteval;///< 两个GPS的时间间隔

    double v;///< 论文中的v
    double ft;///< 论文中的ft
};

///\brief IVMM
class IVMM{
public:
    template<typename V> using VVector = std::vector<std::vector<V> >;
    template<typename V> using VVVector = std::vector<std::vector<std::vector<V> > >;
    IVMM(RoadMap const* map, IVMMParam const& p):map_(map),param(p){}

    VVector<Candidate>
        candidates(
                std::vector<GpsPoint> const& log)const;

    VVector<double>
        normal(
            std::vector<GpsPoint> const& log,
            VVector<Candidate> const& candidates)const;

    VVVector<Path>
        paths(VVector<Candidate> const& candidates)const;

    VVVector<Detail>
        detail(std::vector<GpsPoint> const& log, VVVector<Path> const& paths)const;

    double find_sequence(
            std::vector<int>& seq,
            VVector<double> const& n,
            VVVector<Detail> const& vft,
            std::vector<GpsPoint> const& log,
            std::vector<double> const& w,
            VVector<Candidate> const& candidates,
            int focus_on_gps,
            int must_pass_cand,
            int gps_begin,
            int gps_end)const;

    bool st_match(std::vector<GpsPoint> const& log,
            VVector<double> &n,
            VVVector<Detail>& details,
            VVVector<Path>& paths,
            VVector<Candidate>& candidates,
            std::vector<int> & finalCand
        )const;

    bool map_match(std::vector<GpsPoint> const& log,
            VVector<double> &n,
            VVVector<Detail>& details,
            VVVector<Path>& paths,
            VVector<Candidate>& candidates,
            std::vector<int>& finalCand)const;

    std::vector<Path> map_match(std::vector<GpsPoint> const& log)const;

    void initW(std::vector<double>& w,
            std::vector<GpsPoint> const& log,
            int begin, int end, int focusOnGps)const;

    void draw_paths_to_shp(
            char const* output,
            std::vector<int> const& finalCand, 
            IVMM::VVVector<Path> const& paths,
            IVMM::VVVector<Detail> const& details, 
            IVMM::VVector<double> const& n)const;

    ///\brief ivmm 地图匹配
    ///\param[in] log GpsLog
    ///\param[out] ranges 有效的路径区间
    ///\ret 匹配路径，匹配路径中间可能断代，用ranges指出
    std::vector<Path> 
        map_match_s(std::vector<GpsPoint> const& log, std::vector<std::pair<int, int> >& ranges)const;
    IVMMParam param;
    inline RoadMap const& map()const{
        return *map_;
    }

private:
    RoadMap const* map_;
};

double weight_speed(Path const& path, RoadMap const& map);

#endif  /*IVMM_H*/
