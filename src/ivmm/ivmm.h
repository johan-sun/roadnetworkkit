#ifndef  IVMM_H
#define  IVMM_H
//auto-include {{{
#include <string>
#include <vector>
#include <utility>
#include "roadmap.h"
//}}}


class GpsPoint;

struct Candidate{
    ProjectPoint point;
    int vote;
    double fvalue;
};

struct IVMMParam{
    double candidateQueryRadious;
    int candidateLimit;
    double projectDistMean;
    double projectDistStddev;
    double beta;
    int window;
};

struct Detail{
    double weightSpeed;
    double avgSpeed;
    double twoGpsDistance;
    double pathLength;
    int timeInteval;

    double v;
    double ft;
};
class IVMM{
public:
    template<typename V> using VVector = std::vector<std::vector<V> >;
    template<typename V> using VVVector = std::vector<std::vector<std::vector<V> > >;
    IVMM(RoadMap const* map, IVMMParam const& p):_map(map),param(p){}

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

    double findSequence(
            std::vector<int>& seq,
            VVector<double> const& n,
            VVVector<Detail> const& vft,
            std::vector<GpsPoint> const& log,
            std::vector<double> const& w,
            VVector<Candidate> const& candidates,
            int focusOnGps,
            int mustPassCand,
            int gpsBegin,
            int gpsEnd)const;

    bool stMatch(std::vector<GpsPoint> const& log,
            VVector<double> &n,
            VVVector<Detail>& details,
            VVVector<Path>& paths,
            VVector<Candidate>& candidates,
            std::vector<int> & finalCand
        )const;

    bool mapMatch(std::vector<GpsPoint> const& log,
            VVector<double> &n,
            VVVector<Detail>& details,
            VVVector<Path>& paths,
            VVector<Candidate>& candidates,
            std::vector<int>& finalCand)const;

    std::vector<Path> mapMatch(std::vector<GpsPoint> const& log)const;

    void initW(std::vector<double>& w,
            std::vector<GpsPoint> const& log,
            int begin, int end, int focusOnGps)const;

    void drawPathsToShp(
            char const* output,
            std::vector<int> const& finalCand, 
            IVMM::VVVector<Path> const& paths,
            IVMM::VVVector<Detail> const& details, 
            IVMM::VVector<double> const& n)const;

    std::vector<Path> 
        mapMatchSafe(std::vector<GpsPoint> const& log, std::vector<std::pair<int, int> >& ranges)const;
    IVMMParam param;
    inline RoadMap const& map()const{
        return *_map;
    }

private:
    RoadMap const* _map;
};

#endif  /*IVMM_H*/
