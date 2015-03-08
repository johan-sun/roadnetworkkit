#include "sutil/boost/date_time/date_time_format.hpp"
#include "gps.h"
using namespace std;
vector<GpsPoint> loadFromFile(std::string const & name){
    vector<GpsPoint> ret;
    ifstream ifs(name);
    tm t;
    double x;
    double y;
    if ( ifs){
        while ( ifs.ignore(1024, ',') &&
                ifs >> get_time(&t, "%Y-%m-%d %H:%M:%S") &&
                ifs.ignore() &&
                ifs.ignore(1024,',') &&
                ifs.ignore(1024, ',') &&
                ifs >> x && ifs.ignore() &&
                ifs >> y){
            GpsPoint p(x, y, boost::posix_time::ptime_from_tm(t));
            ret.push_back(p);
        }
    }
    return ret;
}
