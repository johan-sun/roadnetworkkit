//auto-include {{{
#include  <gps.h>
#include  <ivmm/ivmm.h>
#include  <vector>
#include  <roadmap.h>
#include  <boost/timer.hpp>
#include  <boost/range.hpp>
#include  <boost/range/adaptors.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/range/algorithm_ext.hpp>
#include  <climits>
//}}}
using namespace std;


size_t LCS(vector<int> const& traj, vector<int> const& restore)
{
    vector<size_t> index;
    index.reserve(restore.size());
    unordered_map<int, vector<int> > index_map;
    //save index for restore
    for(size_t i = 0; i < restore.size(); ++i)
        index_map[restore[i]].push_back(i);


    for(int idxA = 0; idxA < traj.size(); ++idxA)
    {
        int ID = traj[idxA];
        for(int idxB : index_map[ID] | b::adaptors::reversed)
            index.push_back(idxB);
    }

    vector<size_t> l;
    for(auto i : index) {
        auto iter = lower_bound(l.begin(), l.end(), i);
        if ( iter == l.end() ) {
            l.push_back(i);
        }else {
            *iter = i;
        }
    }

    return l.size();
}

double eval(vector<int> const& traj, vector<int> const& restore)
{   
    return LCS(traj, restore) * 2.0 / (traj.size() + restore.size());
}

vector<int> map_to_crosses(Path const& p) {
    vector<int> r;
    struct CrossAppender : boost::static_visitor<void> {
        vector<int>* v;
        CrossAppender(vector<int>* v): v(v) {}

        void operator()(ARoadSegment const& r) {
            if (v->empty() || v->back() != r.start_cross){
                v->push_back(r.start_cross);
            }

            if (v->empty() || v->back() != r.end_cross) {
                v->push_back(r.end_cross);
            }
        }

        void operator()(PartOfRoad const& r) {
            this->operator()(r.start);
            this->operator()(r.end);
        }

        void operator()(ProjectPoint const& p) {
            if (p.type == ProjectPoint::OnCross) {
                if ( v->empty() || v->back() != p.index )
                    v->push_back(p.index);
            } 
        }
    };
    CrossAppender appender(&r);
    for ( auto & e : p.entities ) {
        boost::apply_visitor(appender, e);
    }

    return r;
}

int main(int argc, char *argv[])
{
    string config = "default.ini";
    int n = 500;
    int all = 1;


    if ( argc < 3 ) {
        cout << argv[0] << " <resize to n>  <test of gps>"<< endl;
        return 1;
    }
    
    n = atoi(argv[1]);
    all = atoi(argv[2]);

    auto param = IVMMParam::load_config(config);
    if (!param ) {
        cerr << "can not load ivmm config file" << endl;
        return 1;
    }

    RoadMap map;
    if (!map.load("../data/map/bj-road-epsg3785") ){
        cerr << "can not load map" << endl;
        return 1;
    }

    vector<vector<GpsPoint>> all_test_data;

    string line;
    cout << "Input test gps log path" << endl;
    while (getline(cin, line)) {
        auto gps = load_from_file(line);
        if ( gps.size() > n  ) {
            gps.resize(n);
            all_test_data.push_back(std::move(gps));
            if ( all_test_data.size() >= all ) break;
        }
    }

    cout << "Load all " << all_test_data.size() << " gps log" << endl;
    boost::timer timer;
    cout << "Each log is " << n << " points" << endl;

    unordered_map<int, vector<vector<Path> > > paths;


    int window[] = { 50, 100, 200, -1};
    int test_window[] = {50,100,200};
    all = atoi(argv[2]);
    //for ( int w : window  ) {
    //    IVMMParam p = *param;
    //    p.window = w;
    //    IVMM ivmm(&map, p);
    //    vector<pair<int,int>> range;
    //    timer.restart();
    //    for (auto& log : all_test_data ) {
    //        auto path = ivmm.map_match_s(log, range);
    //        paths[w].push_back(std::move(path));
    //    }
    //    cout << "Window = " << w << " average spend " << timer.elapsed() / all << " seconds" << endl;
    //}
    unordered_map<int, double> lcs_sum;
    unordered_map<int, size_t> lcs_cnt;
    int i = 0;
    for (auto& log : all_test_data) {
        ++i;
        cout << i << "/" << all << endl;
        IVMMParam p = *param;
        unordered_map<int, vector<int>> path_for_window;
        for ( int w : window ) {
            p.window = w;
            IVMM ivmm(&map, p);
            vector<pair<int, int>> range;
            auto path = ivmm.map_match_s(log, range);
            vector<int> crosses;
            for (auto& p : path) {
                boost::push_back(crosses, map_to_crosses(p));
            }
            path_for_window[w] = std::move(crosses);
        }

        for (int w : test_window) {
            double r = eval(path_for_window[-1], path_for_window[w]);
            cout << " lcs rate = " << r << " window = " << w << endl;
            if (!::isnan(r)) {
                lcs_sum[w] += r;
                lcs_cnt[w]++;
            }
        }
    }

    for (int w : test_window ) {
        cout << "==========" << endl;
        cout << "avg lcs rate = " << lcs_sum[w] / lcs_cnt[w] << " window = " << w << endl;
    }

    return 0;
}
