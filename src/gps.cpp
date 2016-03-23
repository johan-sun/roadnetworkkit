#include <string>
#include  <boost/date_time.hpp>
#include "gps.h"
using namespace std;

vector<GpsPoint> load_from_file(std::string const & name){
    vector<GpsPoint> ret;
    ifstream ifs(name);
    ifs.imbue(locale(ifs.getloc(), new boost::posix_time::time_input_facet("%Y-%m-%d %H:%M:%S")));
    double x;
    double y;
    boost::posix_time::ptime ptime;
    if ( ifs){
        while ( ifs.ignore(1024, ',') &&
                ifs >> ptime &&
                ifs.ignore(),
                ifs.ignore(1024,',') &&
                ifs.ignore(1024, ',') &&
                ifs >> x && ifs.ignore() &&
                ifs >> y){
            GpsPoint p(x, y, ptime);
            ret.push_back(p);
        }
    }
    return ret;
}
