//auto-include {{{
#include  "com.h"
#include  <iostream>
#include  <roadmap.h>
#include  <mysql++.h>
#include  <time_estimate.h>
#include  <path-restore/highlevelmap.h>
#include  <boost/timer.hpp>
//}}}


using namespace std;

vector<Point> to_points(RoadMap const& map, vector<TimedCrossIndex> const& traj) {
    vector<Point> points;
    for (auto pair : traj) {
        int idx = pair.first;
        points.push_back(map.cross(idx).geometry);

    }
    return points;
}

int main(int argc, char *argv[])
{
    mysqlpp::Connection con;
    mysqlpp::NoExceptions noexception(con);
    if (! con.connect("path_restore", "localhost", "root", "19900708")) {
        cerr << "can not connect databases" << endl;
        return 1;
    }

    mysqlpp::Query q = con.query("SELECT `id` FROM `traj_metadata`");
    mysqlpp::StoreQueryResult result = q.store();

    RoadMap map;
    if(!map.load("../data/map/bj-road-epsg3785") ) {
        cerr << "can not load map" << endl;
        return 1;
    }

    int left = 0;
    int right = 0;
    int straight = 0;

    size_t cnt = 0;
    size_t all = result.size();
    boost::timer timer;
    for ( auto const& r : result ) {
        if ( timer.elapsed() > 0.1 ) {
            timer.restart();
            clog << "\r" << cnt << "/" << all;
            clog.flush();
        }
        ++cnt;
        int id = r[0];
        vector<Point> points = to_points(map, load_timed_path_from_DB(con, id));
        for ( size_t i = 2; i < points.size(); ++i ) {
            Point a = points[i-2];
            Point b = points[i-1];
            Point c = points[i];
            if ( bg::equals(a, c) || bg::equals(b, c) || bg::equals(a, b)) continue;//地图匹配导致的折返
            bg::subtract_point(c, b);
            bg::subtract_point(b, a);
            switch( direction(a, b) ) {
                case 1:
                    right += 1;
                    break;
                case -1:
                    left += 1;
                default: straight += 1;
            }
        }
    }

    cout << "left:" << left << " right:" << right << " straight:"<< straight << endl;
    return 0;
}
