cmake_minimum_required(VERSION 3.12)
project(Pathfinder CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add library and application subdirectories
add_subdirectory(src)
add_subdirectory(apps)
