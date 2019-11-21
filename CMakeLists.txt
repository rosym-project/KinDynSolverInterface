cmake_minimum_required(VERSION 3.0)

# TODO double check if this policy is fine
# if(POLICY CMP0053)
#     cmake_policy(SET CMP0053 OLD)
# endif()

set(xQP_VERSION_MAJOR 0)
set(xQP_VERSION_MINOR 1)

project(KinDynModel)

# Telling RBDL cmake to add its urdfreader module and add it to the build
set(RBDL_BUILD_ADDON_URDFREADER ON CACHE BOOL "Force enable the urdfreader" FORCE)
add_subdirectory(externals/rbdl)

set(SOURCES ${CMAKE_SOURCE_DIR}/src/KinDynModel.cpp
            ${CMAKE_SOURCE_DIR}/include/KinDynModel.hpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
