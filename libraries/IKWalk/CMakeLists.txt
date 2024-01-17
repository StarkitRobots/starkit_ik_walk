###
### Based on Humanoid IKWalk Rhoban 2015
### Starkit 2023
###

cmake_minimum_required(VERSION 2.8)
project(starkit_ik_walk)

#Sources files
set(SOURCES_FILES_WALK
    CubicSpline.cpp
    HumanoidModel.cpp
    IKWalk.cpp
    LegIK.cpp
    Polynom.cpp
    Spline.cpp
)

set(HEADERS_FILES_WALK
    CubicSpline.hpp
    HumanoidModel.hpp
    IKWalk.hpp
    LegIK.hpp
    Polynom.hpp
    Spline.hpp
)

set(PYTHON_BINDING_FILE
    python/python.cpp
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Eigen library dependency
#find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../eigen)

add_subdirectory(../pybind11 pybind11)
#Enable C++11
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#Enable compiler Warning
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")

pybind11_add_module(starkit_ik_walk 
	${SOURCES_FILES_WALK}
	${HEADER_FILES}
	${PYTHON_BINDING_FILE}
)

target_link_libraries(starkit_ik_walk PUBLIC)

set(PYTHON_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../controllers/ik_walk)

install(TARGETS starkit_ik_walk
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
  )
