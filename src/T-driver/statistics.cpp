//auto-include {{{
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <boost/range/algorithm.hpp>
#include "roadmap.h"
#include "time_estimate.h"
#include "bj-road-epsg3785/bj_road_epsg3785.h"
//}}}
using namespace std;

struct Phase{
    int in;
    int out;
    bool operator==(Phase const& p)const{
        return in == p.in && out == p.out;
    }
};
int main(int argc, char *argv[])
{
    RoadMap map;
    unordered_map<int, vector<pair<Phase,int> > > crossInfo;

    if (argc < 2){
        cerr << "usage <roadmap>"<<endl;
    }
    if ( !map.load(argv[1],BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker())){
        cerr << "can not load " << argv[1] << endl;
    }
    string line;
    while(getline(cin ,line)){
        vector<TimedCrossIndex> timedPath = loadTimedPathFromFile(line);
        if ( !timedPath.empty() ){
            vector<TimedCrossIndex>::iterator begin = timedPath.begin();
            vector<TimedCrossIndex>::iterator end = timedPath.end();
            while(begin < end){
                auto pre = begin;
                auto cur = find_if(begin ,end, [pre](TimedCrossIndex const& c){
                    return c.first != pre->first;
                });
                if ( cur == end ){
                    break;
                }
                auto succ = find_if(cur, end, [cur](TimedCrossIndex const& c){
                    return c.first != cur->first;
                 });
                if ( succ == end ){
                    break;
                }
                begin = cur;
                Phase p;
                p.in = pre->first;
                p.out = succ->first;
                auto fd = boost::find_if(crossInfo[cur->first], [p](pair<Phase, int> const& phPair){ return phPair.first == p; });
                if ( fd == crossInfo[cur->first].end()){
                    crossInfo[cur->first].push_back({p, 1});
                }else{
                    fd->second++;
                }
            }
        }
    }

    for(auto& eachPair : crossInfo){
        int crossIndex = eachPair.first;
        vector<pair<Phase, int> > const& phasCounter = eachPair.second;
        cout << crossIndex << ":";
        for(auto& eachPh : phasCounter){
            cout << " (" << eachPh.first.in << " ," << eachPh.first.out << ") "<<eachPh.second;
        }
        cout << endl;
    }
    return 0;
}
