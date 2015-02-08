cmake_minimum_required(VERSION 2.6)
project (roadnetworkkit)

find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python unit_test_framework program_options filesystem system serialization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
include_directories(src)

add_executable(test
    test/test.cpp
    src/roadmap.cpp
)

target_link_libraries(test
    shp
)

install(TARGETS test
    RUNTIME DESTINATION workspace
)
