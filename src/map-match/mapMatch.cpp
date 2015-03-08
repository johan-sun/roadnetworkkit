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
        param.projectDistMean = pt.get<double>("IVMM.projectDistMean");
        param.projectDistStddev = pt.get<double>("IVMM.projectDistStddev");
        param.candidateQueryRadious = pt.get<double>("IVMM.candidateQueryRadious");
        param.candidateLimit = pt.get<int>("IVMM.candidateLimit");
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
    fs::path trajOutput;
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
    vector<vector<TimedCrossIndex> > timedPath;
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
    if ( !po->pinput->trajOutput.empty() ) {
        fs::path dir = po->pinput->trajOutput.parent_path();
        if (!dir.empty() && !fs::exists(dir)) {
            try {
                fs::create_directories(dir);
            } catch (fs::filesystem_error const &err) {
                cerr << "[" << po->pinput->line << "]" << " error:" << err.what() << endl;
            }
        }
        if( po->timedPath.size() == 1){
            fs::ofstream os(po->pinput->trajOutput);
            if (!os) {
                cerr << "[" << po->pinput->line << "]" << " error:" << "can not open " << po->pinput->trajOutput << endl;
            } else {
                for (TimedCrossIndex const &idx : po->timedPath[0]) {
                    os << idx.first << "," << to_format_string(idx.second, "%Y-%m-%d %H:%M:%S%F") << "\n";
                }
            }
        }else{
            fs::path newName;
            for(int i = 0; i < po->timedPath.size(); ++i){
                fs::path const& f = po->pinput->trajOutput.filename();
                newName = dir/(f.stem().string()+"-"+to_string(i)+f.extension().string());
                fs::ofstream os(newName);
                if (!os) {
                    cerr << "[" << po->pinput->line << "]" << " error:" << "can not open " << newName << endl;
                } else {
                    for (TimedCrossIndex const &idx : po->timedPath[i]) {
                        os << idx.first << "," << to_format_string(idx.second, "%Y-%m-%d %H:%M:%S%F") << "\n";
                    }
                }
            }
        }
    }
    
    /*
    if ( !po->pinput->shpOutput.empty() ){
        fs::path dir = po->pinput->shpOutput.parent_path();
        if (!dir.empty() && !fs::exists(dir)){
            try{
                fs::create_directories(dir);
            }catch(fs::filesystem_error const& err){
                cerr << "["<< po->pinput->line << "] error:" << err.what() << "\n";
            }
        }

        if ( po->timedPath.size() > 2 ){
            SHPHandle shp = SHPCreate(po->pinput->shpOutput.c_str(), SHPT_ARC);
            if ( ! shp ) { 
                cerr << "[" << po->pinput->line << "] error:" << "can not create shp " << po->pinput->shpOutput << "\n";
                return;
            }
            DBFHandle dbf = DBFHandle(po->pinput->shpOutput.c_str());
            if ( ! dbf  ) {
                cerr << "[" << po->pinput->line << "] error:" << "can not create dbf " << po->pinput->shpOutput << "\n";
                return;
            }
            DBFAddField(dbf, "START", FTInteger, 10, 0);
            DBFAddField(dbf, "END", FTInteger, 10, 0);
            DBFAddField(dbf, "STIME", FTString, 40,0);
            DBFAddField(dbf, "ETIME", FTString, 40,0);
            DBFAddField(dbf, "COSTs", FTDouble, 12, 6);
            double x[2];
            double y[2];
            for(int i = 0; i + 1 < po->timedPath.size(); ++i){
                Cross const& s = map.cross(po->timedPath[i].first);
                Cross const& e = map.cross(po->timedPath[i+1].first);
                x[0] = s.geometry.x();
                y[0] = s.geometry.y();
                x[1] = e.geometry.x();
                y[1] = e.geometry.y();
                SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
                int id = SHPWriteObject(shp, -1, line);
                DBFWriteIntegerAttribute(dbf, id, 0, s.index);
                DBFWriteIntegerAttribute(dbf, id, 1, e.index);
                DBFWriteStringAttribute(dbf, id, 2, to_format_string(po->timedPath[i].second, "%Y-%m-%d %H:%M:%S%F").c_str());
                DBFWriteStringAttribute(dbf, id, 3, to_format_string(po->timedPath[i+1].second, "%Y-%m-%d %H:%M:%S%F").c_str());
                DBFWriteDoubleAttribute(dbf, id, 4, (po->timedPath[i+1].second-po->timedPath[i].second).total_milliseconds()/1000.0);
                SHPDestroyObject(line);
            }
            SHPClose(shp);
            DBFClose(dbf);
        }
    }*/
}

void writer(lf::queue<Output*>& outputQueue, 
        RoadMap const& bjRoadMap, 
        atomic_bool const& mapMatchDone){
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

    while (! mapMatchDone ){
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
    string input, trajOutput;//, shpOutput;
    long line = 0;
    while(readLine(input, line)){
        int inputLine = line;
        if ( !readLine(trajOutput, line) ) break;
        /*
        if (withShp){
            if ( !getline(cin, shpOutput) ) break;
            ++line;
        }*/

        Input* pinput = new Input;
        pinput->line = inputLine;
        pinput->log = loadFromFile(input);
        pinput->input = fs::path(input);
        pinput->trajOutput = fs::path(trajOutput);
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
        atomic_bool const& readDone){
    auto doWith = [&](Input* pinput){
        b::timer timer;
        Output* poutput = new Output;
        poutput->pinput = pinput;
        vector<pair<int, int> > ranges;
        vector<Path> paths = ivmm.mapMatchSafe(pinput->log, ranges);
        if ( paths.empty()){
            poutput->type = Output::MapMatchFail;
        }else{
            for(auto& range : ranges){
                vector<TimedCrossIndex> timedPath = estimateTime(pinput->log, paths, range, ivmm.map());
                if (! timedPath.empty()){
                    poutput->timedPath.push_back(std::move(timedPath));
                }
            }
            poutput->type = Output::Finished;
            poutput->cost = timer.elapsed();
        }
        /*
        if ( paths.empty() ){
            poutput->type = Output::MapMatchFail;
        }else{
            vector<TimedCrossIndex> timedPath = estimateTime(pinput->log, paths, ivmm.map());
            poutput->cost = timer.elapsed();
            poutput->timedPath = std::move(timedPath);
            poutput->type = Output::Finished;
        }*/
        while(!outputQueue.push(poutput)){
            b::this_thread::sleep(b::posix_time::millisec(100));
        }
    };
    while ( ! readDone ){
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

    time_t t = time(NULL);
    cout.imbue(locale(cout.getloc(), new b::posix_time::time_facet("%Y-%m-%d %H:%M:%S")));
    cout << b::posix_time::second_clock::local_time() << " SHP:" << roadShp << endl;
    cout << "IVMM.projectDistMean = " << param.projectDistMean << endl;
    cout << "IVMM.projectDistStddev = " << param.projectDistStddev << endl;
    cout << "IVMM.candidateQueryRadious = " << param.candidateQueryRadious << endl;
    cout << "IVMM.candidateLimit = " << param.candidateLimit << endl;
    cout << "IVMM.beta = " << param.beta << endl;
    cout << "IVMM.window = " << param.window << endl;
    IVMM ivmm(&bjRoadMap, param);
    lf::queue<Input*> inputQueue(10000);
    lf::queue<Output*> outputQueue(100);
    atomic_bool readDone(false);
    atomic_bool mapMatchDone(false);
    if ( vm.count("debug")){
        reader(inputQueue, outputQueue);
        readDone = true;
        working(inputQueue,outputQueue, ivmm,readDone);
        mapMatchDone = true;
        writer(outputQueue, bjRoadMap, mapMatchDone);
    }else {
        b::thread readerThread(b::bind(reader,
                b::ref(inputQueue),
                b::ref(outputQueue)));
        b::thread_group workingThreads;
        for (int i = 0; i < j; ++i) {
            workingThreads.create_thread(
                    b::bind(working,
                            b::ref(inputQueue),
                            b::ref(outputQueue),
                            b::cref(ivmm),
                            b::cref(readDone)));
        }
        b::thread writerThread(
                b::bind(writer,
                        b::ref(outputQueue),
                        b::cref(bjRoadMap),
                        b::cref(mapMatchDone)));
        readerThread.join();
        readDone = true;
        workingThreads.join_all();
        mapMatchDone = true;
        writerThread.join();
    }
    return 0;
}
