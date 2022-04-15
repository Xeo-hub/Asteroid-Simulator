cmake_minimum_required(VERSION 3.16)
project(sim_aos)

set(CMAKE_CXX_STANDARD 20)

add_executable(paos main.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-deprecated -Werror -pedantic -pedantic-errors -Ofast -O3 -DNDEBUG -fopenmp")
