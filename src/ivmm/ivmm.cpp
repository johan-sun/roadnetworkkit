//auto-include {{{
#include <shapefil.h>
#include <boost/range/algorithm.hpp>
#include <boost/phoenix.hpp>
#include  <boost/property_tree/ptree.hpp>
#include  <boost/property_tree/ini_parser.hpp>
#include "ivmm/ivmm.h"
#include "gps.h"
#include "util.h"
//}}}
using namespace std;
namespace lambda = b::phoenix::arg_names;
namespace pt = b::property_tree;

static double oo = numeric_limits<double>::infinity();

inline static double normal(GpsPoint const& gps, Candidate const& cp, double mean, double stddev){
    static double sqrt_2_pi = sqrt( 2 * M_PI);
    double dist = bg::distance(gps, cp.point.geometry) - mean;
    double dist_sqr = dist * dist;
    double i = - dist_sqr / ( 2 * stddev * stddev);
    double c = 1.0 /  ( sqrt_2_pi * stddev);
    double n = c * exp(i);
    if ( n < 1e-8 ) n = 1e-8;
    return n;
}

double weight_speed(Path const& path, RoadMap const& map){
    struct SpeedGetter : public b::static_visitor<double>{
        RoadMap const& map;
        SpeedGetter(RoadMap const& map):map(map){}

        double operator()(ARoadSegment const& ars)const{
            RoadSegment const& rs = map.roadsegment(ars.roadsegment_index);
            return rs.properties.get<double>("SPEED");
        }

        double operator()(PartOfRoad const& pr)const{
            RoadSegment const& rs = map.roadsegment(pr.roadsegment_index);
            return rs.properties.get<double>("SPEED");
        }

        double operator()(ProjectPoint const& )const{
            return 0;
        }
    };

    struct LengthGetter : public b::static_visitor<double>{
        double operator()(ARoadSegment const& ars)const{
            return ars.length;
        }

        double operator()(PartOfRoad const& pr)const{
            return pr.length;
        }

        double operator()(ProjectPoint const& )const{
            return 0;
        }
    };


    double d = path.total_length();
    double weightSpeed = 0;
    for(auto& e : path.entities){
        weightSpeed += b::apply_visitor(LengthGetter(), e) / d * b::apply_visitor(SpeedGetter(map), e);
    }
    return weightSpeed;
}

static void initPre(
        IVMM::VVector<int> &pre,
        IVMM::VVector<Candidate> const& candidates,int begin,int end){
    int nCand = end - begin;
    pre.resize(nCand);
    for(int i = begin; i != end; ++i){
        pre[i-begin].resize(candidates[i].size(), -1);
    }
}

IVMM::VVector<Candidate> IVMM::candidates(std::vector<GpsPoint> const &log) const{
    VVector<Candidate> candidates;
    candidates.resize(log.size());
    for(size_t i = 0; i < log.size(); ++i){
        double r = param.candidate_query_radious;
        vector<int> roadIdx = map_->query_road(log[i].geometry, r);
        while (roadIdx.empty()){
            r *= 1.1;
            roadIdx = map_->query_road(log[i].geometry, r);
        }

        for(int ri : roadIdx){
            RoadSegment const& rs = map_->roadsegment(ri);
            Candidate c;
            c.vote = 0;
            c.fvalue = 0.0;
            c.point = make_project_point(log[i].geometry, rs);//projectPoint(log[i], r.geometry);
            vector<Candidate> & can = candidates[i];
            if ( static_cast<int>(can.size()) < param.candidate_limit &&
                    ( can.empty() || !bg::equals(c.point.geometry, can.back().point.geometry))){
                can.push_back(std::move(c));
            }
        }
    }
    return candidates;
}

template<typename V, typename C>
static void initVV(IVMM::VVector<V>& vv, IVMM::VVector<C> const& c){
    vv.resize(c.size());
    for(int i = 0; i < c.size(); ++i){
        vv[i].resize(c[i].size());
    }
}

template<typename V, typename P>
static void initVVV(IVMM::VVVector<V>& vvv, IVMM::VVector<P> const& c){
    vvv.resize(c.size() - 1);
    for(int i = 1; i < c.size(); ++i){
        vector<P> const& srcCand = c[i-1];
        vector<P> const& destCand = c[i];
        vvv[i-1].resize(srcCand.size());
        for(auto& e : vvv[i-1]){
            e.resize(destCand.size());
        }
    }
}

template<typename V, typename P>
static void initVVV(IVMM::VVVector<V> & vvv, IVMM::VVVector<P> const& p){
    vvv.resize(p.size());
    for(int i= 0; i < p.size(); ++i){
        vvv[i].resize(p[i].size());
        for(int j = 0; j < p[i].size(); ++j){
            vvv[i][j].resize(p[i][j].size());
        }
    }
}

IVMM::VVector<double> IVMM::normal(std::vector<GpsPoint> const &log, IVMM::VVector<Candidate> const &candidates) const {
    VVector<double> n;
    initVV(n, candidates);
    for(size_t i = 0; i < candidates.size(); ++i){
        for(size_t j = 0; j < candidates[i].size(); ++j){
            n[i][j] = ::normal(log[i], candidates[i][j], param.project_dist_mean, param.project_dist_stddev);
        }
    }

    return n;
}

IVMM::VVVector<Path> IVMM::paths(IVMM::VVector<Candidate> const &candidates) const {
    VVVector<Path> paths;
    initVVV(paths, candidates);
    for(size_t i = 1; i < candidates.size(); ++i){
        int srcGps = i - 1;
        int destGps = i;
        for(size_t srcCand = 0; srcCand < candidates[srcGps].size(); ++srcCand){
            for(size_t destCand = 0; destCand < candidates[destGps].size(); ++destCand){
                paths[srcGps][srcCand][destCand] =
                        map_->shortest_path(candidates[srcGps][srcCand].point,
                                candidates[destGps][destCand].point);
            }
        }
    }
    return paths;
}

double IVMM::find_sequence(
        vector<int> &seq,
        IVMM::VVector<double> const &n,
        IVMM::VVVector<Detail> const& vft,
        std::vector<GpsPoint> const &log,
        std::vector<double> const& w,
        IVMM::VVector<Candidate> const &candidates,
        int focusOnGps, int mustPassCand, int gpsBegin, int gpsEnd) const {

    vector<double> f(candidates[gpsBegin].size());
    VVector<int> pre;
    initPre(pre, candidates, gpsBegin, gpsEnd);

    //init
    for(int i = 0; i < candidates[gpsBegin].size(); ++i){
        f[i] = w[gpsBegin] * n[gpsBegin][i];
    }

    if ( gpsBegin == focusOnGps){
        for(int i = 0; i < candidates[focusOnGps].size(); ++i){
            if ( i != mustPassCand ){
                f[i] = -oo;
            }
        }
    }

    for(int destGps = gpsBegin + 1; destGps < gpsEnd; ++destGps){
        vector<double> newF(candidates[destGps].size());
        int srcGps = destGps - 1;
        for(int destCand = 0; destCand < candidates[destGps].size(); ++destCand){
            //find max pre candidate for destCand
            double maxF = -oo;
            int preCandIdx = -1;
            for(int srcCand = 0; srcCand < f.size(); ++srcCand){
                Detail const detail = vft[srcGps][srcCand][destCand];
                double fst = w[srcGps] *
                    n[destGps][destCand] *
                        detail.v * detail.ft;
                if ( f[srcCand] + fst > maxF ){
                    maxF = f[srcCand] + fst;
                    preCandIdx = srcCand;
                }
            }
            newF[destCand] = maxF;
            pre[destGps - gpsBegin][destCand] = preCandIdx;
        }
        f = std::move(newF);
        if ( destGps == focusOnGps ){
            for(int i = 0; i < f.size(); ++i){
                if ( i != mustPassCand ){
                    f[i] = -oo;
                }
            }
        }
        if (b::all(f, lambda::arg1 == -oo) ){
            return -1;
        }
    }

    int idx = b::max_element<b::return_begin_found>(f).size();
    double fvalue = f[idx];
    int nGps = gpsEnd - gpsBegin;
    seq.resize(nGps);

    while(nGps--) {
        seq[nGps] = idx;//const_cast<Candidate *>(&candidates[nGps][idx]);
        idx = pre[nGps][idx];
    }

    return fvalue;
}

static pair<int, int> window(int i, int w, int sz){
    int begin, end;
    if ( w <= 0){
        begin = 0;
        end = sz;
    }else{
        begin = max(0, i - w);
        end = min(i + w, sz);
    }
    return { begin, end};
}

bool IVMM::map_match(vector<GpsPoint> const &log,
        VVector<double> & n,
        VVVector<Detail> &details,
        VVVector<Path>& paths,
        VVector<Candidate>& candidates,
        std::vector<int>& finalCand)const {
    candidates = this->candidates(log);
    paths = this->paths(candidates);
    n = normal(log, candidates);
    details = this->detail(log, paths);

    vector<double> w(log.size() - 1, 1.0);

    int sz = (int) log.size();
    for(int i = 0; i < log.size(); ++i){
        int begin, end;
        tie(begin, end) = window(i, param.window, sz);
        //here weight for gps i
        initW(w, log, begin, end, i);
        int badConnection = 0;
        for(int k = 0; k < candidates[i].size(); ++k){
            vector<int> sequence;
            double fvalue = find_sequence(sequence, n, details, log,w ,candidates, i, k, begin, end);
            if ( fvalue < 0){
                ++badConnection;
            }else{
                //for(Candidate* pc : sequence) pc->vote++;
                for(int b = 0; b < sequence.size(); ++b){
                    candidates[b+begin][sequence[b]].vote++;
                }
            }
            candidates[i][k].fvalue = fvalue;
        }
        if ( badConnection == candidates[i].size() ){
            return false;
        }
    }

    finalCand.resize(candidates.size());
    for(int i = 0; i < candidates.size(); ++i){
        finalCand[i] = b::max_element<b::return_begin_found>(candidates[i],
                [](Candidate const& a, Candidate const& b){
                    return make_pair(a.vote,a.fvalue) < make_pair( b.vote, b.fvalue);
                }).size();
    }
    return true;
}

IVMM::VVVector<Detail> IVMM::detail(std::vector<GpsPoint> const &log, IVMM::VVVector<Path> const &paths) const {
    VVVector<Detail> details;
    initVVV(details, paths);
    for(int srcGps = 0; srcGps < paths.size(); ++srcGps){
        int destGps = srcGps + 1;
        double distanceOfTowGps = bg::distance(log[srcGps], log[destGps]);
        int timeInterval = (log[destGps].time - log[srcGps].time).total_seconds();
        for(int srcCand = 0; srcCand < paths[srcGps].size(); ++srcCand){
            for(int destCand = 0; destCand < paths[srcGps][srcCand].size(); ++destCand){
                Detail & detail = details[srcGps][srcCand][destCand];
                Path const& path = paths[srcGps][srcCand][destCand];
                double pathLength = path.total_length();
                double d = distanceOfTowGps + 1;
                double l = pathLength + 1;
                detail.v = path.valid()? min((d/l)*(d/l), sqrt(l/d)) : -oo;

                double weightSpeed = pathLength == 0? 0 : ::weight_speed(path, *map_);
                double avgSpeed = pathLength / timeInterval;

                if  ( weightSpeed == 0 ) {
                    detail.ft = 1;
                }else{
                    detail.ft = weightSpeed / (abs(avgSpeed - weightSpeed) + weightSpeed);
                }

                //=============
                detail.weight_speed = weightSpeed;
                detail.avg_speed = avgSpeed;
                detail.path_length = pathLength;
                detail.time_inteval = timeInterval;
                detail.two_gps_distance = distanceOfTowGps;
            }
        }
    }
    return details;
}

void IVMM::initW(std::vector<double> &w, std::vector<GpsPoint> const &log, int begin, int end, int focusOnGps) const {
    for(int m = begin; m < end - 1; ++m){
        int otr = m;
        if ( focusOnGps <= m ) otr++;
        double d = bg::distance(log[focusOnGps], log[otr]);
        w[m] = exp(-d*d/(param.beta*param.beta));
    }
}

bool IVMM::st_match(
        std::vector<GpsPoint> const &log,
        IVMM::VVector<double> &n,
        IVMM::VVVector<Detail> &details,
        IVMM::VVVector<Path> &paths,
        IVMM::VVector<Candidate> &candidates,
        std::vector<int> &finalCand) const {

    candidates = this->candidates(log);
    paths = this->paths(candidates);
    n = normal(log, candidates);
    details = this->detail(log, paths);
    vector<double> w(log.size() - 1, 1.0);
    if (find_sequence(finalCand, n, details, log, w, candidates, -1, -1, 0, log.size()) < 0){
        return false;
    }
    return true;
}

vector<Path> IVMM::map_match(std::vector<GpsPoint> const& log)const{
    vector<Path> finalPath;
    VVector<double> n;
    VVVector<Detail> details;
    VVVector<Path> paths;
    VVector<Candidate> candidates;
    vector<int> finalCand;
    if ( ! map_match(log, n, details, paths, candidates, finalCand) ){
        return finalPath;
    }
    for(int i = 0; i < paths.size(); ++i){
        int srcBest = finalCand[i];
        int destBest = finalCand[i+1];
        finalPath.push_back(std::move(paths[i][srcBest][destBest]));
    }
    return finalPath;
}

void IVMM::draw_paths_to_shp(
        char const* output,
        vector<int> const& finalCand, 
        IVMM::VVVector<Path> const& paths,
        IVMM::VVVector<Detail> const& details, 
        IVMM::VVector<double> const& n)const{
    RoadMap const& bjRoad = *map_;
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
                DBFWriteDoubleAttribute(dbf, id, 5, detail.path_length);
                DBFWriteDoubleAttribute(dbf, id, 6, detail.two_gps_distance);
                DBFWriteDoubleAttribute(dbf, id, 7, detail.v);
                DBFWriteIntegerAttribute(dbf, id, 8, detail.time_inteval);
                DBFWriteDoubleAttribute(dbf, id, 9, detail.avg_speed);
                DBFWriteDoubleAttribute(dbf, id, 10, detail.weight_speed);
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

vector<Path> IVMM::map_match_s(vector<GpsPoint> const& log, vector<pair<int, int> >& ranges)const{
    vector<Path> paths = map_match(log);
    if ( paths.empty() ){
        return paths;
    }
    auto pathBegin = find_if(paths.begin(), paths.end(), [](Path const& p){ return p.valid(); });
    auto gpsBegin = log.begin() + distance(paths.begin(), pathBegin);
    while( pathBegin < paths.end() ){
        auto firstInvalidPath = find_if(pathBegin, paths.end(), [](Path const& p){ return !p.valid(); });
        auto invalidPathBeginGps = gpsBegin + distance(pathBegin, firstInvalidPath);
        ranges.emplace_back(distance(log.begin(), gpsBegin), distance(log.begin(), invalidPathBeginGps+1));
        pathBegin = find_if(firstInvalidPath, paths.end(), [](Path const& p){ return p.valid(); });
        gpsBegin = log.begin() + distance(paths.begin(), pathBegin);
    }
    return paths;
}

boost::optional<IVMMParam> IVMMParam::load_config(std::string const& filename) {
    pt::ptree pt;
    IVMMParam param;
    try{
        pt::read_ini(filename, pt);
        param.project_dist_mean = pt.get<double>("IVMM.projectDistMean");
        param.project_dist_stddev = pt.get<double>("IVMM.projectDistStddev");
        param.candidate_query_radious = pt.get<double>("IVMM.candidateQueryRadious");
        param.candidate_limit = pt.get<int>("IVMM.candidateLimit");
        param.beta = pt.get<double>("IVMM.beta");
        param.window = pt.get<int>("IVMM.window");
    }catch(std::exception const& e){
        std::cerr << e.what() << endl;
        return boost::none;
    }
    return param;
}
