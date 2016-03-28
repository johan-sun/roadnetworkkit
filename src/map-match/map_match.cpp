//auto-include {{{
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <shapefil.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include <ctime>
#include <atomic>
#include <boost/timer.hpp>
#include "ivmm/ivmm.h"
#include "bj-road-epsg3785/bj_road_epsg3785.h"
#include "gps.h"
#include "time_estimate.h"
#include "sutil/boost/date_time/date_time_format.hpp"
//}}}
using namespace std;

namespace po = boost::program_options;
namespace pt = boost::property_tree;
namespace fs = boost::filesystem;
namespace lf = boost::lockfree;
namespace b = boost;

bool generateDefaultConfigFile(string const& name){
    IVMMParam param;
    pt::ptree pt;
    pt.put("IVMM.projectDistMean", 5.0);
    pt.put("IVMM.projectDistStddev", 20.0);
    pt.put("IVMM.candidateQueryRadious", 100.0);
    pt.put("IVMM.candidateLimit", 5);
    pt.put("IVMM.beta", 5000.0);
    pt.put("IVMM.window", 50);
    try{
        pt::write_ini(name, pt);
    }catch(std::exception const& e){
        cerr << "write default config file " << name << " fail.";
        cerr << e.what() << endl;
        return false;
    }
    return true;
}

bool readIVMMParam(string const& name, IVMMParam& param){
    pt::ptree pt;
    pt::read_ini(name, pt);
    try{
        param.project_dist_mean = pt.get<double>("IVMM.projectDistMean");
        param.project_dist_stddev = pt.get<double>("IVMM.projectDistStddev");
        param.candidate_query_radious = pt.get<double>("IVMM.candidateQueryRadious");
        param.candidate_limit = pt.get<int>("IVMM.candidateLimit");
        param.beta = pt.get<double>("IVMM.beta");
        param.window = pt.get<int>("IVMM.window");
    }catch(std::exception const& e){
        cerr << e.what();
        return false;
    }
    return true;
}

struct Input{
    long line;
    fs::path input;
    fs::path traj_output;
    vector<GpsPoint> log;
};
struct Output{
    enum Type{
        GpsTooLessOrLoadFail,
        MapMatchFail,
        Finished
    };
    Type type;
    Input* pinput;
    double cost;
    vector<vector<TimedCrossIndex> > timed_path;
};


bool readLine(string& line, long& inputLine){
    while (getline(cin, line) ){
        ++inputLine;
        b::trim(line);
        if ( line.empty() || line[0] == '#'){
            continue;
        }
        return true;
    }
    return false;
}
void doOutput(Output const* po, RoadMap const& map){
    if ( !po->pinput->traj_output.empty() ) {
        fs::path dir = po->pinput->traj_output.parent_path();
        if (!dir.empty() && !fs::exists(dir)) {
            try {
                fs::create_directories(dir);
            } catch (fs::filesystem_error const &err) {
                cerr << "[" << po->pinput->line << "]" << " error:" << err.what() << endl;
            }
        }
        if( po->timed_path.size() == 1){
            fs::ofstream os(po->pinput->traj_output);
            if (!os) {
                cerr << "[" << po->pinput->line << "]" << " error:" << "can not open " << po->pinput->traj_output << endl;
            } else {
                for (TimedCrossIndex const &idx : po->timed_path[0]) {
                    os << idx.first << "," << to_format_string(idx.second, "%Y-%m-%d %H:%M:%S%F") << "\n";
                }
            }
        }else{
            fs::path new_name;
            for(int i = 0; i < po->timed_path.size(); ++i){
                fs::path const& f = po->pinput->traj_output.filename();
                new_name = dir/(f.stem().string()+"-"+to_string(i)+f.extension().string());
                fs::ofstream os(new_name);
                if (!os) {
                    cerr << "[" << po->pinput->line << "]" << " error:" << "can not open " << new_name << endl;
                } else {
                    for (TimedCrossIndex const &idx : po->timed_path[i]) {
                        os << idx.first << "," << to_format_string(idx.second, "%Y-%m-%d %H:%M:%S%F") << "\n";
                    }
                }
            }
        }
    }
}

void writer(lf::queue<Output*>& outputQueue, 
        RoadMap const& bjRoadMap, 
        atomic_bool const& map_match_done){
    auto doWith = [&bjRoadMap](Output * poutput){
        if ( poutput->type == Output::GpsTooLessOrLoadFail)
            cerr << b::posix_time::second_clock::local_time() << " [" << poutput->pinput->line << "]" << " gps too less or load fail:" << poutput->pinput->input << endl;
        else if ( poutput->type == Output::MapMatchFail )
            cerr << b::posix_time::second_clock::local_time() << " [" << poutput->pinput->line << "]" << " map match fail:" << poutput->pinput->input << endl;
        else{
            doOutput(poutput, bjRoadMap);
            cout << b::posix_time::second_clock::local_time() << " [" << poutput->pinput->line << "]" << " finished in "<< poutput->cost << "s:" << poutput->pinput->input << endl;
        }
        delete poutput->pinput;
        delete poutput;
    };

    while (! map_match_done ){
        Output * poutput; 
        if(outputQueue.pop(poutput) ){
            doWith(poutput);
        }else{
            b::this_thread::sleep(b::posix_time::millisec(5000));
        }
    }
    Output* poutput;
    while ( outputQueue.pop(poutput)){
        doWith(poutput);
    }
}

void reader(
        lf::queue<Input*> &inputQueue,
        lf::queue<Output*> &outputQueue){
    string input, traj_output;//, shpOutput;
    long line = 0;
    cout << "first line is input path of gps log, second line is output path" << endl;
    while(readLine(input, line)){
        int inputLine = line;
        if ( !readLine(traj_output, line) ) break;
        /*
        if (withShp){
            if ( !getline(cin, shpOutput) ) break;
            ++line;
        }*/

        Input* pinput = new Input;
        pinput->line = inputLine;
        pinput->log = load_from_file(input);
        pinput->input = fs::path(input);
        pinput->traj_output = fs::path(traj_output);
        //pinput->shpOutput = fs::path(shpOutput);
        if ( pinput->log.size() > 5 ){
            while(!inputQueue.push(pinput)){
                b::this_thread::sleep(b::posix_time::millisec(50));
            }
        }else{
            Output* poutput = new Output;
            poutput->pinput = pinput;
            poutput->type = Output::GpsTooLessOrLoadFail;
            while(!outputQueue.push(poutput)){
                b::this_thread::sleep(b::posix_time::millisec(100));
            }
        }
    }
}

void working(lf::queue<Input*> & inputQueue, 
        lf::queue<Output*> & outputQueue, 
        IVMM const& ivmm,
        atomic_bool const& read_done){
    auto doWith = [&](Input* pinput){
        b::timer timer;
        Output* poutput = new Output;
        poutput->pinput = pinput;
        vector<pair<int, int> > ranges;
        vector<Path> paths = ivmm.map_match_s(pinput->log, ranges);
        if ( paths.empty()){
            poutput->type = Output::MapMatchFail;
        }else{
            for(auto& range : ranges){
                vector<TimedCrossIndex> timed_path = estimate_time(pinput->log, paths, range, ivmm.map());
                if (! timed_path.empty()){
                    poutput->timed_path.push_back(std::move(timed_path));
                }
            }
            poutput->type = Output::Finished;
            poutput->cost = timer.elapsed();
        }
        
        while(!outputQueue.push(poutput)){
            b::this_thread::sleep(b::posix_time::millisec(100));
        }
    };
    while ( ! read_done ){
        Input* pinput;
        if ( inputQueue.pop(pinput) ){
            doWith(pinput);
        }else{
            b::this_thread::sleep(b::posix_time::millisec(50));
        }
    }

    Input* pinput;
    while ( inputQueue.pop(pinput) ){
        doWith(pinput);
    }
}

int main(int argc, char *argv[])
{
    string configFile;
    string roadShp;
    int j;
    po::options_description desc;
    desc.add_options()
        ("config,c", po::value(&configFile)->required()->default_value("default.ini"), "ivmm paramater config file" )
        ("road,r", po::value(&roadShp)->required(), "road shp file")
        ("debug","use singal thread")
        (",j", po::value(&j)->default_value(1), "use extra thread to run map match")
        ("help,h", "show help");
    po::variables_map vm;
    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }catch(std::exception const& e){
        if ( !vm.count("help") ) cerr << e.what() << endl;
        cout << desc << endl;
        return 0;
    }

    try{
        vm.notify();
    }catch(std::exception const& e){
        if ( !vm.count("help") ) cerr << e.what() << endl;
        cout << desc << endl;
        return 0;
    }
    if ( vm.count("help") ){
        cout << desc << endl;
        return 0;
    }

    if (!fs::exists(configFile)){
        cout << configFile << " doest not exists, generate a default one" << endl;
        if (!generateDefaultConfigFile(configFile)){
            return -1;
        }
    }

    IVMMParam param;
    if ( ! readIVMMParam(configFile, param) ){
        cerr  << "can not parser " << configFile << endl;
        return -1;
    }

    RoadMap bjRoadMap;
    if ( ! bjRoadMap.load(roadShp, BJRoadEpsg3785IDPicker(), BJRoadEpsg3785CrossIDChecker())){
        cerr << "can not load road " << roadShp << endl;
        return -1;
    }

    cout.imbue(locale(cout.getloc(), new b::posix_time::time_facet("%Y-%m-%d %H:%M:%S")));
    cerr.imbue(locale(cout.getloc(), new b::posix_time::time_facet("%Y-%m-%d %H:%M:%S")));
    cout << b::posix_time::second_clock::local_time() << " SHP:" << roadShp << endl;
    cout << "IVMM.projectDistMean = " << param.project_dist_mean << endl;
    cout << "IVMM.projectDistStddev = " << param.project_dist_stddev << endl;
    cout << "IVMM.candidateQueryRadious = " << param.candidate_query_radious << endl;
    cout << "IVMM.candidateLimit = " << param.candidate_limit << endl;
    cout << "IVMM.beta = " << param.beta << endl;
    cout << "IVMM.window = " << param.window << endl;
    IVMM ivmm(&bjRoadMap, param);
    lf::queue<Input*> inputQueue(10000);
    lf::queue<Output*> outputQueue(100);
    atomic_bool read_done(false);
    atomic_bool map_match_done(false);
    if ( vm.count("debug")){
        reader(inputQueue, outputQueue);
        read_done = true;
        working(inputQueue,outputQueue, ivmm,read_done);
        map_match_done = true;
        writer(outputQueue, bjRoadMap, map_match_done);
    }else {
        b::thread read_thread(b::bind(reader,
                b::ref(inputQueue),
                b::ref(outputQueue)));
        b::thread_group working_threads;
        for (int i = 0; i < j; ++i) {
            working_threads.create_thread(
                    b::bind(working,
                            b::ref(inputQueue),
                            b::ref(outputQueue),
                            b::cref(ivmm),
                            b::cref(read_done)));
        }
        b::thread write_thread(
                b::bind(writer,
                        b::ref(outputQueue),
                        b::cref(bjRoadMap),
                        b::cref(map_match_done)));
        read_thread.join();
        read_done = true;
        working_threads.join_all();
        map_match_done = true;
        write_thread.join();
    }
    return 0;
}
