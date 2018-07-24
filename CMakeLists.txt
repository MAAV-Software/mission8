cmake_minimum_required (VERSION 3.10)
project(maav-gnc)


include_directories(include)

add_library(maav-gnc SHARED
            src/gnc.cpp)

set_target_properties(maav-gnc PROPERTIES PUBLIC_HEADER
                      include/gnc.hpp)
target_include_directories(maav-gnc PRIVATE src)

