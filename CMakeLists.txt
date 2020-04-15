cmake_minimum_required(VERSION 3.1)

## TODO double check if this policy is fine
# if(POLICY CMP0053)
#     cmake_policy(SET CMP0053 OLD)
# endif()

include(CMakePackageConfigHelpers)

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

## Versioning
set(KDMI_VERSION_MAJOR 0)
set(KDMI_VERSION_MINOR 1)
project(KDMI VERSION ${KDMI_VERSION_MAJOR}.${KDMI_VERSION_MINOR})
configure_file(${PROJECT_SOURCE_DIR}/config/version.h.in version.h)

## should be set externally by the user
set(USE_RBDL ON)
set(USE_KDL OFF)

## Eigen3 which is required by both RBDL and KDL
find_package (Eigen3 3.3 REQUIRED)
if (EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIRS})
else ()
  message (FATAL_ERROR "Eigen3 not found. Check the following for the fix:\n \
                        https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html")
endif ()


## ----------------- RBDL Interface ----------------- ##
if(USE_RBDL)        
    FIND_PACKAGE(PkgConfig)
    set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
    pkg_search_module(RBDL REQUIRED rbdl)    
    SET(RBDL_URDFReader_LIBRARIES rbdl_urdfreader)
    add_library(KinDynModelRBDL SHARED ${PROJECT_SOURCE_DIR}/src/KinDynModelRBDL.cpp)
    target_include_directories(KinDynModelRBDL PUBLIC
        "$<INSTALL_INTERFACE:include;${RBDL_INCLUDE_DIRS}>"
        "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include;${RBDL_INCLUDE_DIRS}>"
    )
    target_link_directories(KinDynModelRBDL PUBLIC
        "$<INSTALL_INTERFACE:lib;${RBDL_LIBRARY_DIRS}>"
        "$<BUILD_INTERFACE:${RBDL_LIBRARY_DIRS}>"
        )
    target_link_libraries(KinDynModelRBDL ${RBDL_LIBRARIES} ${RBDL_URDFReader_LIBRARIES})

    # installing RBDL-interface if it is being used           
    install(TARGETS KinDynModelRBDL
            EXPORT ${PROJECT_NAME}-targets
            DESTINATION lib)

    install(EXPORT ${PROJECT_NAME}-targets
            FILE   ${PROJECT_NAME}-targets.cmake
            DESTINATION lib/cmake/${PROJECT_NAME})
    
    install(FILES ${PROJECT_SOURCE_DIR}/include/KinDynModelRBDL.hpp
            DESTINATION include/${PROJECT_NAME})
endif()

## ----------------- KDL Interface ----------------- ##
#  TBA
#

## ----------------- Factory ----------------- ##
add_library(KinDynInterface SHARED ${PROJECT_SOURCE_DIR}/src/KinDynModelFactory.cpp)

target_include_directories(KinDynInterface PUBLIC
                           "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                           "$<INSTALL_INTERFACE:include>"
                           )

if(USE_RBDL)
    target_link_libraries(KinDynInterface PUBLIC KinDynModelRBDL)
    set(ADDITIONAL_INCLUDE_DIRS ${RBDL_INCLUDE_DIRS})
    set(ADDITIONAL_LIBRARY_DIRS ${RBDL_LIBRARY_DIRS})
endif()

## ----------------- INSTALLING ----------------- ##
configure_package_config_file("${PROJECT_SOURCE_DIR}/config/${CMAKE_PROJECT_NAME}-config.cmake.in"
                              "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
                               INSTALL_DESTINATION lib/cmake/${CMAKE_PROJECT_NAME})

install(FILES ${PROJECT_SOURCE_DIR}/include/KinDynModel.hpp
              ${PROJECT_SOURCE_DIR}/include/KinDynModelFactory.hpp
              ${PROJECT_BINARY_DIR}/version.h
              DESTINATION include/${PROJECT_NAME})
            
install(TARGETS KinDynInterface
        EXPORT ${PROJECT_NAME}-targets
        DESTINATION lib)


install(EXPORT ${PROJECT_NAME}-targets
        FILE   ${PROJECT_NAME}-targets.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
        DESTINATION lib/cmake/${CMAKE_PROJECT_NAME})

set(BUILD_TESTS ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
