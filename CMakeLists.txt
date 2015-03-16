cmake_minimum_required(VERSION 2.8)
project (roadnetworkkit)

#find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework program_options filesystem system thread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
include_directories(src)

add_library(roadnetworkkit STATIC
    src/generalmap.cpp
    src/roadmap.cpp
    src/gps.cpp
    src/ivmm/ivmm.cpp
    src/time_estimate.cpp
)

target_link_libraries(roadnetworkkit
    shp
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


add_executable(mapMatch
    src/map-match/mapMatch.cpp
)

target_link_libraries(mapMatch
    roadnetworkkit
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)


add_executable(timedPathDraw
    src/tools/trajToShp.cpp
)

target_link_libraries(timedPathDraw
    roadnetworkkit
)

add_executable(t-driver-statistics
    src/T-driver/statistics.cpp
)

target_link_libraries(t-driver-statistics
    roadnetworkkit
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
install(TARGETS mapMatch timedPathDraw t-driver-statistics
    RUNTIME DESTINATION workspace
)

install(TARGETS test_ testivmm testroadmap
    RUNTIME DESTINATION test
)

