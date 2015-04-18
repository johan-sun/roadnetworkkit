//auto-include {{{
#include  <mysql++.h>
#include  <cmdline.h>
#include  <shapefil.h>
#include  <iostream>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry.hpp>
//}}}
using namespace std;
namespace fs = boost::filesystem;
int main(int argc, char *argv[])
{
    mysqlpp::examples::CommandLine cmd(argc, argv);
    auto args = cmd.extra_args();
    boost::regex idpicker("type=(\\d+)");
    boost::regex outputPicker("output=([\\w.\\-/~_+*]+)");
    boost::smatch match;
    fs::path output;
    int type = -1;
    for(string& str : args)
    {
        if ( boost::regex_match(str, match, idpicker) )
        {
            type = stoi(match[1]);
        }
        if ( boost::regex_match(str, match, outputPicker) )
        {
            output = match[1];
        }
    }
    if ( type == -1 || output == "" )
    {
        cerr << "need type and output" << endl;
        cout << "usage type=<int> output=<path>" << endl;
        return 1;
    }
    
    fs::path parentPath = output.parent_path();
    if (parentPath != "" && ! fs::exists(parentPath ))
    {
        boost::system::error_code err;
        fs::create_directories(parentPath, err);
        if ( err ){
            cerr << err.message() << endl;
            return 1;
        }
    }

    SHPHandle shp = SHPCreate(output.c_str(), SHPT_ARC);
    DBFHandle dbf = DBFCreate(output.c_str());
    DBFAddField(dbf, "START", FTInteger, 10, 0);
    DBFAddField(dbf, "END", FTInteger, 10, 0);
    DBFAddField(dbf, "DBID", FTInteger,  10, 0);

    try
    {
        mysqlpp::Connection con("path_restore", cmd.server(), cmd.user(), cmd.pass());
        mysqlpp::Query query = con.query("SELECT start, end, ASTEXT(cs.geometry) as startGEO, ASTEXT(ce.geometry) as endGEO, ve.id "
                "FROM virtual_edge_metadata as ve, `cross` as cs, `cross` as ce "
                "WHERE ve.start = cs.id and ve.end = ce.id and ve.type = %0");
        query.parse();
        mysqlpp::UseQueryResult use = query.use(type);
        while(mysqlpp::Row row = use.fetch_row())
        {
            int start = row[0];
            int end = row[1];
            string startGEO;
            string endGEO;
            row[2].to_string(startGEO);
            row[3].to_string(endGEO);
            int dbid = row[4];
            boost::geometry::model::d2::point_xy<double> s ,e ;
            boost::geometry::read_wkt(startGEO, s);
            boost::geometry::read_wkt(endGEO, e);
            double x[2] = {s.x(), e.x()};
            double y[2] = {s.y(), e.y()};
            SHPObject* arc = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, 0);
            const int insert = -1;
            int id = SHPWriteObject(shp, insert, arc);
            DBFWriteIntegerAttribute(dbf, id, 0, start);
            DBFWriteIntegerAttribute(dbf, id, 1, end);
            DBFWriteIntegerAttribute(dbf, id, 2, dbid);
            SHPDestroyObject(arc);
        }
    }catch(mysqlpp::BadQuery const& e)
    {
        cerr << "bad query:" << e.what() << endl;
    }
    catch(mysqlpp::ConnectionFailed const& e)
    {
        cerr << "connect to db fail:" << e.what() << endl;
    }
    catch(mysqlpp::Exception const& e)
    {
        cerr <<  e.what() << endl;
    }
    SHPClose(shp);
    DBFClose(dbf);
    return 0;
}
