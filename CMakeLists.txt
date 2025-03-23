cmake_minimum_required(VERSION 3.10)
project(QuadrotorPlanner)

# Enable C++ language and set the standard
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include OMPL, Eigen, and project-specific paths
include_directories(
    /usr/local/include/ompl-1.6
    /usr/include/eigen3
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/other_stuff
)

# Find Boost (Filesystem, System, Serialization)
find_package(Boost REQUIRED COMPONENTS filesystem system serialization)
include_directories(${Boost_INCLUDE_DIRS})

# Find Python3 with Development and NumPy
find_package(Python3 REQUIRED COMPONENTS Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# Find OMPL and OMPL.app
find_package(ompl REQUIRED)

# Add your executable
add_executable(PayloadFourDrones src/PayloadFourDemo.cpp src/PayloadFourDrones.cpp)

# Link required libraries explicitly and clearly
target_link_libraries(PayloadFourDrones PRIVATE
    ompl
    ompl_app
    ompl_app_base
    Boost::filesystem
    Boost::system
    Boost::serialization  # added explicitly to solve linker errors
    Python3::Python
    Python3::NumPy
    stdc++fs
)

# Include directories clearly
target_include_directories(PayloadFourDrones PRIVATE
    ${OMPL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    include
)


# # Find pybind11 and link it to relevant targets
# find_package(pybind11 REQUIRED)

# # Add the executable for QuadrotorDemo
# add_executable(QuadrotorDemo other_stuff/QuadrotorDemo.cpp other_stuff/QuadrotorPlanning.cpp)
# target_link_libraries(QuadrotorDemo PRIVATE ompl ompl_app ompl_app_base Python3::Python Python3::NumPy)

# # Add the executable for PayloadTwoDronesDemo
# add_executable(PayloadTwoDrones src/PayloadTwoDemo.cpp src/PayloadTwoDrones.cpp)
# target_link_libraries(PayloadTwoDrones PRIVATE stdc++fs ompl ompl_app ompl_app_base)

# Set build type to Debug for debugging purposes
set(CMAKE_BUILD_TYPE Debug)
