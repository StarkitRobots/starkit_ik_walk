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

include_directories(include/ include/utils include/model include/engine)

add_library(utils SHARED 
    src/utils/geometry/Angle.cpp 
    src/utils/geometry/Point.cpp
    src/utils/spline/PolySpline.cpp
)

add_library(model SHARED
    src/model/HumanoidModel.cpp
    src/model/LegIK.cpp
)
target_link_libraries(model PUBLIC utils)

add_library(engine SHARED
    src/engine/Foot.cpp
    src/engine/FootPose.cpp
    src/engine/WalkEngine.cpp
)
target_link_libraries(engine PUBLIC model utils)

set(PYTHON_BINDING_FILE
    python/python.cpp
)

pybind11_add_module(starkit_ik_walk
	${PYTHON_BINDING_FILE}
)
target_link_libraries(starkit_ik_walk PUBLIC engine model utils)

set(PYTHON_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../controllers/ik_walk)

install(TARGETS starkit_ik_walk
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
)
