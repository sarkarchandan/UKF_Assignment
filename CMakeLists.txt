cmake_minimum_required(VERSION 3.0...3.22)

if(${CMAKE_VERSION} VERSION_LESS 3.22)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(playback)

find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.12 REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(SOURCES
    src/main.cpp
    src/ukf.cpp
    src/tools.cpp
    src/render/render.cpp
)

set(EXEC_NAME ukf_highway)

add_executable(${EXEC_NAME} ${SOURCES})
target_link_libraries(${EXEC_NAME} ${PYTHON_LIBRARIES})
target_link_libraries(${EXEC_NAME} ${Eigen3_Libs})
target_link_libraries(${EXEC_NAME} ${PCL_LIBRARIES})
target_compile_features(${EXEC_NAME} PRIVATE cxx_std_17)

