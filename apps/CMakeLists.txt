# Define the executable
add_executable(pathfinder
    main.cpp
    io.cpp
)

# Set private include directory
target_include_directories(pathfinder
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link the library
target_link_libraries(pathfinder PRIVATE pathfinder_lib)
