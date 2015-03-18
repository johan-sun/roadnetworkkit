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
struct CrossDetail
{
    int crossIndex;
    int count;
    double entropy;
};
struct Phase{
    int in;
    int out;
    bool operator==(Phase const& p)const{
        return in == p.in && out == p.out;
    }
};

int total(vector<pair<Phase,int> > const& v){
    int sum = 0;
    for(auto& p : v){
        sum += p.second;
    }
    return sum;
}
int main(int argc, char *argv[])
{
    RoadMap map;
    typedef unordered_map<int, vector<pair<Phase,int> > > CrossInfo;
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

    vector<CrossDetail> allCrossInfoDetails;
    allCrossInfoDetails.reserve(crossInfo.size());
    for(auto& eachInfo : crossInfo){
        int s = total(eachInfo.second);
        double e = 0.0;
        for(auto& eachPh : eachInfo.second){
            double p = (double)(eachPh.second) / s;
            e += p * log2(p);
        }
        CrossDetail d;
        d.crossIndex = eachInfo.first;
        d.count = s;
        d.entropy = -e;
        if ( abs(d.entropy) < 1e-6 ){
            d.entropy = 0;
        }
        allCrossInfoDetails.push_back(d);
    }
    
    sort(allCrossInfoDetails.begin(), allCrossInfoDetails.end(), [](CrossDetail const& cd1, CrossDetail const& cd2){
        return cd1.count * cd1.count * cd1.entropy > cd2.count * cd2.count * cd2.entropy;
    });
    for(auto& detail : allCrossInfoDetails){
        cout << detail.crossIndex << ":" << detail.count << ","<< setprecision(16) << detail.entropy << endl;
    }
    return 0;
}
