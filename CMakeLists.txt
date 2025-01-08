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

# Find Python and link to Python libraries
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# Find pybind11 and link it to relevant targets
find_package(pybind11 REQUIRED)

# # Add the executable for QuadrotorDemo
# add_executable(QuadrotorDemo other_stuff/QuadrotorDemo.cpp other_stuff/QuadrotorPlanning.cpp)
# target_link_libraries(QuadrotorDemo PRIVATE ompl ompl_app ompl_app_base Python3::Python Python3::NumPy)

# # Add the executable for MultiDroneDemo
# add_executable(MultiDroneDemo other_stuff/MultiDroneDemo.cpp other_stuff/MultiDronePlanning.cpp)
# target_link_libraries(MultiDroneDemo PRIVATE ompl ompl_app ompl_app_base Python3::Python Python3::NumPy)

# # Add the executable for PayloadFourDronesDemo
# add_executable(PayloadFourDrones other_stuff/PayloadFourDemo.cpp other_stuff/PayloadFourDrones.cpp)
# target_link_libraries(PayloadFourDrones PRIVATE ompl ompl_app ompl_app_base Python3::Python Python3::NumPy)

# Add the executable for PayloadTwoDronesDemo
add_executable(PayloadTwoDrones src/PayloadTwoDemo.cpp src/PayloadTwoDrones.cpp)
target_link_libraries(PayloadTwoDrones PRIVATE stdc++fs ompl ompl_app ompl_app_base Python3::Python Python3::NumPy pybind11::pybind11)

# # Add the executable for PayloadTwoDronesDemo
# add_executable(PayloadTwoDrones_old other_stuff/PayloadTwoDemo_old.cpp other_stuff/PayloadTwoDrones_old.cpp)
# target_link_libraries(PayloadTwoDrones_old PRIVATE stdc++fs ompl ompl_app ompl_app_base Python3::Python Python3::NumPy pybind11::pybind11)

# Add the executable for Test3DSystem
add_executable(Test3DSystem src/test.cpp src/stateTransition.cpp)
target_link_libraries(Test3DSystem PRIVATE Python3::Python Python3::NumPy stdc++fs)

# Set build type to Debug for debugging purposes
set(CMAKE_BUILD_TYPE Debug)
