cmake_minimum_required(VERSION 3.0)

# TODO double check if this policy is fine
# if(POLICY CMP0053)
#     cmake_policy(SET CMP0053 OLD)
# endif()

set(KDInterface_VERSION_MAJOR 0)
set(KDInterface_VERSION_MINOR 1)

project(KinDynModel)

# Telling RBDL cmake to add its urdfreader module and add it to the build
set(RBDL_BUILD_ADDON_URDFREADER ON CACHE BOOL "Force enable the urdfreader" FORCE)

set(SOURCES ${CMAKE_SOURCE_DIR}/src/KinDynModel.cpp
            ${CMAKE_SOURCE_DIR}/include/KinDynModel.hpp
            ${CMAKE_SOURCE_DIR}/include/KinDynModelRBDL.hpp
            ${CMAKE_SOURCE_DIR}/src/KinDynModelRBDL.cpp
  )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# should be set externally by the user
set(USE_RBDL ON)
set(USE_KDL OFF)

# required for RBDL and other .pc packages
find_package(PkgConfig REQUIRED)

find_package (Eigen3 3.3 REQUIRED)
if (EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIRS})
else ()
  message (FATAL_ERROR "Eigen3 not found. Check the following for the fix:\n \
                        https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html")
endif ()


if(USE_RBDL)
    find_package(RBDL REQUIRED)
    include_directories(${RBDL_INCLUDE_DIR})
endif()

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/include)


# target_link_libraries(${PROJECT_NAME})
