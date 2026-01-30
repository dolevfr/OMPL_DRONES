# cmake_minimum_required(VERSION 3.10)
# project(QuadrotorPlanner)

# set(CMAKE_CXX_STANDARD       17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#     add_compile_options(-Wno-attributes)
# endif()

# # ———————————————
# # 1) Let people point at their BLASFEO install
# # ———————————————
# # From the shell: export BLASFEO_PREFIX=~/acados-deps
# if(DEFINED ENV{BLASFEO_PREFIX})
#   list(APPEND CMAKE_PREFIX_PATH $ENV{BLASFEO_PREFIX})
# endif()

# # Or on the command line: -Dblasfeo_PREFIX=/opt/blasfeo
# if(DEFINED blasfeo_PREFIX)
#   list(APPEND CMAKE_PREFIX_PATH ${blasfeo_PREFIX})
# endif()

# # Now find the BLASFEO config (it lives in <prefix>/cmake/blasfeoConfig.cmake)
# list(PREPEND CMAKE_PREFIX_PATH "$ENV{HOME}/acados-install")
# find_package(blasfeo REQUIRED
#     PATH_SUFFIXES cmake
# )

# # ———————————————
# # 2) Pick up Acados
# # ———————————————
# if(DEFINED ENV{ACADOS_PREFIX})
#   list(APPEND CMAKE_PREFIX_PATH $ENV{ACADOS_PREFIX})
# endif()
# if(DEFINED acados_PREFIX)
#   list(APPEND CMAKE_PREFIX_PATH ${acados_PREFIX})
# endif()

# find_package(acados REQUIRED
#     PATH_SUFFIXES
#       acados/cmake
#       share/acados/cmake
#       lib/cmake/acados
# )

# # Do not use add_subdirectory for c_generated_code
# # add_subdirectory(c_generated_code)

# # ———————————————
# # Find OMPL and Boost
# # ———————————————
# find_package(ompl    REQUIRED)                                            # core OMPL
# find_package(Boost   REQUIRED COMPONENTS filesystem system serialization) # only the Boost modules you use
# find_package(pybind11 REQUIRED)                                           # for Python bindings, if needed
# find_package(Python3 REQUIRED COMPONENTS Development NumPy)               # keep if embedding Python

# if(NOT OMPL_INCLUDE_DIRS)
#     # default path used by ompl-1.6 install
#     set(OMPL_INCLUDE_DIRS "/usr/local/include/ompl-1.6")
# endif()

# find_library(OMPL_APP_BASE_LIB
#   NAMES ompl_app_base
#   HINTS ${OMPL_LIBRARY_DIRS} /usr/local/lib /usr/lib
# )
# if(NOT OMPL_APP_BASE_LIB)
#   message(FATAL_ERROR "libompl_app_base.so not found. Install the OMPL.app dev package.")
# endif()

# find_library(OMPL_APP_LIB
#   NAMES ompl_app
#   HINTS ${OMPL_LIBRARY_DIRS} /usr/local/lib /usr/lib
# )
# if(NOT OMPL_APP_LIB)
#   message(FATAL_ERROR "libompl_app.so not found. Install the OMPL.app dev package.")
# endif()

# include_directories(${OMPL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# ———————————————
# Create executable for AcadosOneDrone
# ———————————————

# add_library(acados_solver_payload_1drones SHARED IMPORTED)

# # Set the location of the generated solver library
# set_target_properties(acados_solver_payload_1drones PROPERTIES
#     IMPORTED_LOCATION
#         ${CMAKE_SOURCE_DIR}/c_generated_code/libacados_ocp_solver_payload_1drones.so
#     INTERFACE_INCLUDE_DIRECTORIES
#         ${CMAKE_SOURCE_DIR}/c_generated_code
# )

# add_executable(AcadosOneDrone 
#     src/AcadosOneDemo.cpp 
#     src/AcadosOneDrone.cpp 
#     src/AcadosMPC.cpp 
#     src/MPCSampler.cpp
#     src/AcadosPythonBridge.cpp
# )

# target_include_directories(AcadosOneDrone PRIVATE
#     ${OMPL_INCLUDE_DIRS}          
#     ${CMAKE_SOURCE_DIR}/include
#     ${CMAKE_SOURCE_DIR}/c_generated_code
# )

# target_link_libraries(AcadosOneDrone PRIVATE
#     acados_solver_payload_1drones
#     acados hpipm blasfeo
#     ${OMPL_APP_BASE_LIB} ${OMPL_APP_LIB} ${OMPL_LIBRARIES}
#     Boost::filesystem Boost::system Boost::serialization
#     Python3::Python Python3::NumPy
#     pybind11::embed
# )

# add_executable(PayloadOneDrone src/PayloadOneDemo.cpp src/PayloadOneDrone.cpp)
# target_link_libraries(PayloadOneDrone PRIVATE ompl ompl_app ompl_app_base Boost::filesystem Boost::system Boost::serialization Python3::Python Python3::NumPy stdc++fs)
# target_include_directories(PayloadOneDrone PRIVATE ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} include)

# add_executable(PayloadFourDrones src/PayloadFourDemo.cpp src/PayloadFourDrones.cpp)
# target_link_libraries(PayloadFourDrones PRIVATE ompl ompl_app ompl_app_base Boost::filesystem Boost::system Boost::serialization Python3::Python Python3::NumPy stdc++fs)
# target_include_directories(PayloadFourDrones PRIVATE ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} include)

# add_executable(AccelFourDrones accel_based/AccelFourDemo.cpp accel_based/AccelFourDrones.cpp)
# target_link_libraries(AccelFourDrones PRIVATE ompl ompl_app ompl_app_base Boost::filesystem Boost::system Boost::serialization Python3::Python Python3::NumPy stdc++fs)
# target_include_directories(AccelFourDrones PRIVATE ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} include)

# add_executable(PosFourDrones pos_based/PosFourDemo.cpp pos_based/PosFourDrones.cpp)
# target_link_libraries(PosFourDrones PRIVATE ompl ompl_app ompl_app_base Boost::filesystem Boost::system Boost::serialization Python3::Python Python3::NumPy stdc++fs)
# target_include_directories(PosFourDrones PRIVATE ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} include)

# add_executable(AccelPidFourDrones accel_pid/AccelPidFourDemo.cpp accel_pid/AccelPidFourDrones.cpp)
# target_link_libraries(AccelPidFourDrones PRIVATE ompl ompl_app ompl_app_base Boost::filesystem Boost::system Boost::serialization Python3::Python Python3::NumPy stdc++fs)
# target_include_directories(AccelPidFourDrones PRIVATE ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} include)

# Finish Executables

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
# endif()

cmake_minimum_required(VERSION 3.16)
project(QuadrotorPlanner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wno-attributes)
endif()

# ---- Pick ONE OMPL/omplapp install (your /usr/local has 1.6) ----
# If you ever switch to 1.7, install omplapp 1.7 and point OMPL_PREFIX there.
set(OMPL_PREFIX "/usr/local" CACHE PATH "Prefix where OMPL/omplapp are installed")

# Make sure we don't accidentally pull ROS Humble OMPL (1.7) in this build
# You can also run cmake from a shell where you did NOT source ROS setup.bash.
list(APPEND CMAKE_IGNORE_PATH "/opt/ros/humble")

# OMPL core (headers/libs) — exact 1.6
find_package(ompl 1.6 REQUIRED CONFIG
  PATHS "${OMPL_PREFIX}" "${OMPL_PREFIX}/lib/cmake/ompl"
  NO_DEFAULT_PATH
)

find_package(Ceres REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development Development.Module Development.Embed REQUIRED)
find_package(pybind11 REQUIRED)

# omplapp headers
find_path(OMPLAPP_INCLUDE_PREFIX
  NAMES omplapp/apps/AppBase.h
  PATHS
    ${OMPL_PREFIX}/include
    ${OMPL_PREFIX}/include/ompl-1.6
  NO_DEFAULT_PATH
)

if(NOT OMPLAPP_INCLUDE_PREFIX)
  message(FATAL_ERROR "Could not find omplapp headers under ${OMPL_PREFIX}/include")
endif()

# omplapp libs
find_library(OMPL_APP_LIB
  NAMES ompl_app
  PATHS "${OMPL_PREFIX}/lib"
  NO_DEFAULT_PATH
)
find_library(OMPL_APP_BASE_LIB
  NAMES ompl_app_base
  PATHS "${OMPL_PREFIX}/lib"
  NO_DEFAULT_PATH
)
if(NOT OMPL_APP_LIB OR NOT OMPL_APP_BASE_LIB)
  message(FATAL_ERROR "Could not find omplapp libs under ${OMPL_PREFIX}/lib")
endif()

# Boost (needed by omplapp apps)
find_package(Boost REQUIRED COMPONENTS filesystem system serialization)

# ---- Build target ----
add_executable(AccelPidFourDrones accel_pid/AccelPidFourDemo.cpp accel_pid/AccelPidFourDrones.cpp)
target_include_directories(AccelPidFourDrones PRIVATE ${OMPL_INCLUDE_DIRS} ${OMPLAPP_INCLUDE_PREFIX} ${Boost_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(AccelPidFourDrones PRIVATE ompl ${OMPL_APP_LIB} ${OMPL_APP_BASE_LIB} ${CERES_LIBRARIES} Boost::filesystem Boost::system Boost::serialization stdc++fs pybind11::embed Python3::Python)
set_target_properties(AccelPidFourDrones PROPERTIES BUILD_RPATH "${OMPL_PREFIX}/lib" INSTALL_RPATH "${OMPL_PREFIX}/lib")

# add_executable(PosPidFourDrones pos_pid/PosPidFourDemo.cpp pos_pid/PosPidFourDrones.cpp)
# target_include_directories(PosPidFourDrones PRIVATE ${OMPL_INCLUDE_DIRS} ${OMPLAPP_INCLUDE_PREFIX} ${Boost_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
# target_link_libraries(PosPidFourDrones PRIVATE ompl ${OMPL_APP_LIB} ${OMPL_APP_BASE_LIB} ${CERES_LIBRARIES} Boost::filesystem Boost::system Boost::serialization stdc++fs pybind11::embed Python3::Python)
# set_target_properties(PosPidFourDrones PROPERTIES BUILD_RPATH "${OMPL_PREFIX}/lib" INSTALL_RPATH "${OMPL_PREFIX}/lib")

# Helpful prints
message(STATUS "OMPL_INCLUDE_DIRS = ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPLAPP_INCLUDE_PREFIX = ${OMPLAPP_INCLUDE_PREFIX}")
message(STATUS "OMPL_APP_LIB = ${OMPL_APP_LIB}")
message(STATUS "OMPL_APP_BASE_LIB = ${OMPL_APP_BASE_LIB}")
