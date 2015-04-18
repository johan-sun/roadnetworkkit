//auto-include {{{
#include <utility>
#include <vector>
#include <fstream>
#include <istream>
#include <functional>
#include <boost/range/algorithm.hpp>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <unordered_map>
#include <mysql++.h>
#include  <mysql++/cmdline.h>
#include <boost/timer.hpp>
#include <iostream>
#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include "sutil/key_visitor.hpp"
//}}}
using namespace boost::posix_time;
using namespace std;
struct Record
{
    unsigned long long id;
    ptime time;
    unsigned int cost;
    double hourInDay()const noexcept{
        return time.time_of_day().total_seconds() / 3600.0;
    }
    long costMilles()const noexcept{
        return cost;
    }
    double costSec()const noexcept{
        return cost / 1000.0;
    }
    int vclass;
    int eclass;
};

typedef vector<Record> Records;
Records load(mysqlpp::UseQueryResult & ur)
{
    Records records;
    Record r;
    try
    {//"SELECT `id`, `enter`, `cost` FROM `virtual_edge_data` WHERE `metadataID`"
        while(mysqlpp::Row row = ur.fetch_row())
        {
            r.id = row[0];
            r.time = boost::posix_time::time_from_string(row[1].c_str());
            r.cost = row[2];
            records.push_back(r);
        }
    }catch(std::exception const& e)
    {
        cerr << "exception in " << __FUNCTION__ << " :" << e.what() << endl;
        throw;
    }
    return records;
}

double var(Records::iterator first, Records::iterator last)
{
    double sum = 0.0;
    size_t dis = distance(first, last);
    for(auto it = first; it != last; ++it) sum += it->costSec();
    double mean = sum / dis;
    double varSum = 0.0;
    for(auto it = first; it != last; ++it) {
        double s = it->costSec() - mean;
        varSum += s*s;
    }
    return varSum / dis;
}

double wav(Records::iterator first, Records::iterator split, Records::iterator last)
{
    size_t L = distance(first, last);
    size_t L1 = distance(first, split);
    size_t L2 = distance(split, last);
    return var(first, split) * L1 / L + var(split, last) * L2 / L;
}
int vcluster_aux(Records::iterator first, Records::iterator last,double t, int initClassID)
{
    for(auto it = first; it != last; ++it) it->vclass = initClassID;
    size_t L = distance(first, last);
    if  ( L < 2 ){
        return initClassID+1;
    }

    double v = var(first, last);
    double minWAV = numeric_limits<double>::max();
    auto minSplit = last;
    for(auto sp = next(first); sp != last; ++sp)
    {
       double w = wav(first, sp, last);
       if ( minWAV > w ){
           minWAV = w;
           minSplit = sp;
       }
    }
    double delta = v - minWAV;
    if ( delta > t ){
        //accept the split
        int nextInitClass = vcluster_aux(first, minSplit, t, initClassID);
        return vcluster_aux(minSplit, last, t, nextInitClass);
    }else
    {
        //reject the split
        return initClassID + 1;
    }
}

int vcluster(Records& r)
{
    boost::sort(r, binary_of<less>(&Record::costSec));
    double v = var(r.begin(), r.end());
    int initClassID = 0;
    return vcluster_aux(r.begin(), r.end(), sqrt(v), initClassID);
}

double ent(Records::iterator first, Records::iterator last, size_t& kclass)
{
    size_t N = distance(first, last);
    unordered_map<int, int> vclassCount;
    for(auto it = first; it != last; ++it)
    {
        vclassCount[it->vclass]++;
    }
    double e = 0.0;
    kclass = vclassCount.size();
    for(auto& pair : vclassCount)
    {
        size_t count = pair.second;
        double p = static_cast<double>(count) / N;
        e += - p * log2(p);
    }
    return e;
}

double mdlp(size_t N, size_t K, double Ent, size_t K1, double Ent1, size_t K2, double Ent2)
{
    return  
        (log2(static_cast<double>(N-1)) + log2(pow(3.0, K) - 2) - ( K * Ent - K1 * Ent1 - K2 * Ent2 )) / N;
}

double wae(Records::iterator first, Records::iterator split, Records::iterator last, size_t& K1, double& Ent1, size_t& K2, double& Ent2)
{
    size_t L = distance(first, last);
    size_t L1 = distance(first, split);
    size_t L2 = distance(split, last);
    Ent1 = ent(first, split, K1);
    Ent2 = ent(split, last, K2);
    return (Ent1 * L1 + Ent2 * L2) / L;
}

int ecluster_aux(Records::iterator first, Records::iterator last, int initClassID)
{
    for(auto it = first; it != last; ++it) it->eclass = initClassID;
    size_t L = distance(first, last);
    if ( L < 2 ){
        return initClassID+1;
    }
    size_t KClass;
    double Ent = ent(first, last, KClass);
    double minWAE = numeric_limits<double>::max();
    double Ent1, Ent2;
    size_t K1Class, K2Class;
    auto split = last;
    for(auto it = next(first); it != last; ++it)
    {
        size_t k1, k2;
        double e1, e2;
        double WAE = wae(first, it, last, k1,e1, k2,e2);
        if ( minWAE > WAE )
        {
            minWAE = WAE;
            Ent1 = e1;Ent2 = e2;
            K1Class = k1;K2Class = k2;
            split = it;
        }
    }

    double Gain = Ent - minWAE;
    double T= mdlp(L, KClass, Ent, K1Class, Ent1, K2Class, Ent2) ;
    if ( Gain < T)
    {
        return initClassID+1;
    }
    int nextClassID = ecluster_aux(first, split, initClassID);
    return ecluster_aux(split, last, nextClassID);
}

int ecluster(Records& r)
{
    boost::sort(r, binary_of<less>(&Record::hourInDay));
    return ecluster_aux(r.begin(), r.end(), 0);
}
struct Pattern
{
    boost::posix_time::time_duration begin;
    boost::posix_time::time_duration end;
    double averageCostMs;
};
struct QData
{
    int metaID;
    Records records;
    vector<Pattern> patterns;
};

void calulatePatern(struct QData& data)
{
    auto begin = data.records.begin();
    Pattern p;
    while( begin != data.records.end() )
    {
        auto end = find_if(begin, data.records.end(), [begin](Record const& arg){ return arg.eclass != begin->eclass; });
        unsigned long long totalMs = 
            accumulate(begin, end, static_cast<unsigned long long>(0), [](unsigned long long init,  Record const& arg){
                return init + arg.cost;
                });
        p.begin = begin->time.time_of_day();
        p.end = end != data.records.end() ? end->time.time_of_day() :boost::posix_time::time_duration(24,0,0);
        p.averageCostMs = static_cast<double>(totalMs) / distance(begin, end) / 1000.0;
        data.patterns.push_back(p);
        begin = end;
    }
}
boost::lockfree::queue<QData*> inputs(10000);
boost::lockfree::queue<QData*> outputs(10000);
boost::atomic_bool inputDone(false);
boost::atomic_bool procDone(false);
const int InputQProductTime = 32;
const int InputQConsumeTime = 1024;
const int OutputQProductTime = 1024;
const int OutputQConsumeTime = 32;
const int MAXTime = 1024;
boost::mutex mutex;
void VECluster()
{
    QData* pdata = 0;
    int ms = InputQProductTime;
    while(!inputDone)
    {
        while(inputs.pop(pdata))
        {
            ms = InputQProductTime;
            vcluster(pdata->records);
            ecluster(pdata->records);
            calulatePatern(*pdata);
            int oms = OutputQConsumeTime;
            while(!outputs.push(pdata))
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(oms));
                oms = min(oms << 1, MAXTime);
            }
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
        ms = min(ms << 1, MAXTime);
    }
    while( inputs.pop(pdata))
    {
        vcluster(pdata->records);
        ecluster(pdata->records);
        calulatePatern(*pdata);
        int oms = OutputQConsumeTime;
        while(!outputs.push(pdata))
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(oms));
            oms = min(MAXTime, oms << 1);
        }
    }
}

void updateMysql(mysqlpp::Connection& con)
{
    boost::timer timer, roundTimer;
    mysqlpp::Query veupdater = con.query("UPDATE `virtual_edge_data` SET `vclass` = %0, `eclass` = %1 WHERE `id` = %2");
    mysqlpp::Query pupdater = con.query("UPDATE `virtual_edge_metadata` SET `pattern` = %0Q WHERE `id` = %1");
    veupdater.parse();
    pupdater.parse();
    QData* pdata = 0;
    size_t cnt = 0;
    int ms = OutputQProductTime;
    auto proc = [&]()
    {
        boost::lock_guard<boost::mutex> g(mutex);
        for(auto& p : pdata->records){
            veupdater.execute(p.vclass, p.eclass, mysqlpp::sql_bigint_unsigned(p.id));
        }
        ostringstream o;
        for(auto& p : pdata->patterns)
        {
            o << p.begin << "-" << p.end << ":" << p.averageCostMs << endl;
        }
        pupdater.execute(o.str(), pdata->metaID);
        delete pdata;
        pdata = 0;
    };
    while(!procDone)
    {
        while(outputs.pop(pdata))
        {
            ms = OutputQProductTime;
            proc();
            ++cnt;
            if ( roundTimer.elapsed() > 0.5 )
            {
                cout << "\r" << cnt << " avg speed:" << cnt / timer.elapsed() << "/s" << flush;
                roundTimer.restart();
            }
        }
        boost::this_thread::sleep(boost::posix_time::millisec(ms));
        ms = min(ms << 1, MAXTime);
    }

    while(outputs.pop(pdata))
    {        
        proc();
        if ( roundTimer.elapsed() > 0.5 )
        {
            cout << "\r" << cnt << " avg speed:" << cnt / timer.elapsed() << "s" << flush;
            roundTimer.restart();
        }
    }
}
int main(int argc, char *argv[])
{
    auto* f = new boost::posix_time::time_facet;
    f->set_iso_extended_format();
    cout.imbue(locale(cout.getloc(), f));
    try
    {
        mysqlpp::examples::CommandLine cmd(argc, argv);
        mysqlpp::Connection con("path_restore", cmd.server(), cmd.user(), cmd.pass());
        mysqlpp::Transaction tan(con);
        mysqlpp::Query q = con.query("SELECT `id` FROM `virtual_edge_metadata`");
        mysqlpp::Query uq = con.query("SELECT `id`, `enter`, `cost` FROM `virtual_edge_data` WHERE `metadata_id` = %0");
        mysqlpp::Query update = con.query("UPDATE `virtual_edge_data` SET `vclass` = %0, `eclass` = %1 WHERE `id` = %2");
        uq.parse();
        update.parse();
        mysqlpp::StoreQueryResult metaIDs = q.store();
        boost::thread_group procGroud;
        boost::thread output(&updateMysql, boost::ref(con));
        for(size_t i = 0; i < boost::thread::hardware_concurrency(); ++i)
        {
            procGroud.create_thread(&VECluster);
        }
        for(auto& row: metaIDs)
        {
            QData* pd = new QData;
            unsigned int metaID = row[0];
            pd->metaID = metaID;
            {boost::lock_guard<boost::mutex> g(mutex);
                mysqlpp::UseQueryResult ur = uq.use(metaID);
                pd->records = load(ur);
            }
            int ms = InputQConsumeTime;
            while(!inputs.push(pd))
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
                ms = min(ms << 1, MAXTime);
            }
        }
        inputDone = true;
        procGroud.join_all();
        procDone = true;
        output.join();
        tan.commit();
    }catch(mysqlpp::Exception const& e)
    {
        cerr << "error:" << e.what() << endl;
    }
    return 0;
}
