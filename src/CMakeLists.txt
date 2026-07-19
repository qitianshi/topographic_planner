# Define the static pathfinder library
add_library(pathfinder_lib STATIC
    map.c
    pathfinder.c
    smooth.c
    adt/bitarr.c
    adt/dynarr.c
    adt/heap.c
)

# Configure include directories
target_include_directories(pathfinder_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/pathfinder>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/pathfinder/adt>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/adt
)

# Link math library if needed
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(pathfinder_lib PRIVATE ${MATH_LIBRARY})
endif()
