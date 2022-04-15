cmake_minimum_required(VERSION 3.16)
project(sim_soa)

set(CMAKE_CXX_STANDARD 20)

add_executable(psoa main.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-deprecated -Werror -pedantic -pedantic-errors -Ofast -O3 -DNDEBUG -fopenmp")
