###
### Based on Humanoid IKWalk Rhoban 2015
### Starkit 2023
###

cmake_minimum_required(VERSION 3.22)
project(starkit_ik_walk)

function(has_package PACKAGE)
    find_package(${PACKAGE} QUIET)

    if(NOT DEFINED ${PACKAGE}_DIR OR ${PACKAGE}_DIR)
        set(HAS_PACKAGE_${PACKAGE} TRUE PARENT_SCOPE)
    else()
        set(HAS_PACKAGE_${PACKAGE} FALSE PARENT_SCOPE)
    endif()

endfunction()


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
    IKWalkInputs.hpp
    LegIK.hpp
    Polynom.hpp
    Spline.hpp
)

set(PYTHON_BINDING_FILE
    python/python.cpp
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Eigen library dependency

has_package(Eigen3)
if(HAS_PACKAGE_Eigen3)
    find_package(Eigen3 REQUIRED)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../eigen)
else()
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../eigen Eigen3)
endif()

set(PYTHON_EXECUTABLE "/usr/bin/python3.9")

has_package(pybind11)
if(HAS_PACKAGE_pybind11)
    find_package(pybind11 REQUIRED)
else()
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pybind11 pybind11)
endif()

#Enable C++17
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
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

