cmake_minimum_required(VERSION 3.12)
project(Pathfinder C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add library and application subdirectories
add_subdirectory(src)
add_subdirectory(apps)
