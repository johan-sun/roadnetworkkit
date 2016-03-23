cmake_minimum_required(VERSION 2.8)
project (roadnetworkkit)

#find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework program_options filesystem system thread date_time regex)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
include_directories(src /usr/include/mysql /usr/include/mysql++)

add_library(roadnetworkkit STATIC
    src/generalmap.cpp
    src/roadmap.cpp
    src/gps.cpp
    src/ivmm/ivmm.cpp
    src/time_estimate.cpp
    src/path-restore/highlevelmap.cpp
)

target_link_libraries(roadnetworkkit
    shp
    mysqlclient
    mysqlpp
    ${Boost_DATE_TIME_LIBRARY}
)

add_executable(test_
    src/test/test.cpp
)

target_link_libraries(test_
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(testivmm
    src/test/testivmm.cpp
)

target_link_libraries(testivmm
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(testroadmap
    src/test/testroadmap.cpp
)

target_link_libraries(testroadmap
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
)


add_executable(map_match
    src/map-match/map_match.cpp
)

target_link_libraries(map_match
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)


add_executable(timed_path_draw
    src/tools/trajToShp.cpp
)

target_link_libraries(timed_path_draw
    roadnetworkkit
)

add_executable(t-driver-statistics
    src/T-driver/statistics.cpp
)

target_link_libraries(t-driver-statistics
    roadnetworkkit
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    mysqlpp
    mysqlclient
)

add_executable(visualEdge
    src/T-driver/visualEdge.cpp
)
target_link_libraries(visualEdge
    roadnetworkkit
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    mysqlpp
    mysqlclient
)

add_executable(vecluster
    src/T-driver/VECluster.cpp
)
target_link_libraries(vecluster
    mysqlpp
    mysqlclient
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)

add_executable(generateHighLevelMap
    src/tools/generateShp.cpp
)

target_link_libraries(generateHighLevelMap
    shp
    mysqlpp
    mysqlclient
    ${Boost_REGEX_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
)

add_executable(testhmap
    src/test/testhighlevelmap.cpp
)
target_link_libraries(testhmap
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(expmain
    src/experiment/main.cpp
    src/experiment/com.cpp
)
target_link_libraries(expmain
    mysqlpp
    mysqlclient
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
)
add_executable(foot_graph_gen
    src/experiment/foot_graph_gen.cpp
    src/experiment/com.cpp
)
target_link_libraries(foot_graph_gen
    mysqlpp
    mysqlclient
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
)

add_executable(ivmm_speed_test
    src/experiment/ivmm_speed_test.cpp
)

target_link_libraries(ivmm_speed_test
    ${Boost_SYSTEM_LIBRARY}
    roadnetworkkit
)

add_executable(turn_stat
    src/experiment/turn_stat.cpp
    src/experiment/com.cpp
)

target_link_libraries(turn_stat
    roadnetworkkit
    mysqlpp
    mysqlclient
    ${Boost_SYSTEM_LIBRARY}
)
install(TARGETS map_match timed_path_draw t-driver-statistics visualEdge vecluster expmain foot_graph_gen ivmm_speed_test turn_stat
    RUNTIME DESTINATION workspace
)

install(TARGETS test_ testivmm testroadmap testhmap
    RUNTIME DESTINATION test
)

