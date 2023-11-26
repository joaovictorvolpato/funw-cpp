cmake_minimum_required(VERSION 3.26)
project(inheritance_test)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(inheritance_test
        inheritance_test.cpp
        inheritance_test.h
        meta.h)
