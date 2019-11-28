cmake_minimum_required(VERSION 3.0)

# TODO double check if this policy is fine
# if(POLICY CMP0053)
#     cmake_policy(SET CMP0053 OLD)
# endif()

set(KDMI_VERSION_MAJOR 0)
set(KDMI_VERSION_MINOR 1)

project(KDMI VERSION ${KDMI_VERSION_MAJOR}.${KDMI_VERSION_MINOR})

configure_file(${PROJECT_SOURCE_DIR}/config/version.h.in version.h)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# should be set externally by the user
set(USE_RBDL ON)
set(USE_KDL OFF)

find_package (Eigen3 3.3 REQUIRED)
if (EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIRS})
else ()
  message (FATAL_ERROR "Eigen3 not found. Check the following for the fix:\n \
                        https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html")
endif ()


## ----------------- RBDL Interface ----------------- ##
if(USE_RBDL)
    find_package(RBDL REQUIRED)
    include_directories(${RBDL_INCLUDE_DIR})

    add_library(KinDynModelRBDL SHARED ${CMAKE_SOURCE_DIR}/src/KinDynModelRBDL.cpp)

    target_include_directories(KinDynModelRBDL PUBLIC ${CMAKE_SOURCE_DIR}/include/)

    target_link_libraries(KinDynModelRBDL ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})

    # installing RBDL-interface if it is being used
    install(TARGETS KinDynModelRBDL DESTINATION lib)
    install(FILES ${CMAKE_SOURCE_DIR}/include/KinDynModelRBDL.hpp DESTINATION include/${PROJECT_NAME})
endif()

## ----------------- KDL Interface ----------------- ##
#  TBA
#

## ----------------- Factory ----------------- ##
add_library(KinDynInterface SHARED ${CMAKE_SOURCE_DIR}/src/KinDynModelFactory.cpp)

target_include_directories(KinDynInterface PUBLIC ${CMAKE_SOURCE_DIR}/include)

if(USE_RBDL)
    target_link_libraries(KinDynInterface PUBLIC KinDynModelRBDL)
endif()

## ----------------- INSTALLING ----------------- ##
install(FILES ${CMAKE_SOURCE_DIR}/include/KinDynModel.hpp
              ${CMAKE_SOURCE_DIR}/include/KinDynModelFactory.hpp
              ${PROJECT_BINARY_DIR}/version.h
              DESTINATION include/${PROJECT_NAME})
install(TARGETS KinDynInterface DESTINATION lib)


set(BUILD_TESTS ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
