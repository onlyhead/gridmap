cmake_minimum_required(VERSION 3.15)

set(project_name gridmap)

project(${project_name} VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-all -Wno-extra -Wno-pedantic -Wno-maybe-uninitialized -Wno-unused-variable -Wno-reorder)
endif()
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(params
    -Wno-reorder
)

string(TOUPPER ${project_name} project_name_upper)
option(${project_name_upper}_BUILD_EXAMPLES "Build examples" OFF)
option(${project_name_upper}_ENABLE_TESTS "Enable tests" OFF)
include(FetchContent)

# --------------------------------------------------------------------------------------------------
set(ext_deps)

FetchContent_Declare(zoneout GIT_REPOSITORY https://github.com/smolfetch/zoneout.git GIT_TAG 0.3.2)
FetchContent_MakeAvailable(zoneout)
list(APPEND ext_deps zoneout::zoneout)

FetchContent_Declare(concord GIT_REPOSITORY https://github.com/smolfetch/concord.git GIT_TAG 2.2.3)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

FetchContent_Declare(geoson GIT_REPOSITORY https://github.com/smolfetch/geoson.git GIT_TAG 1.4.6)
FetchContent_MakeAvailable(geoson)
list(APPEND ext_deps geoson::geoson) 

FetchContent_Declare(geotiv GIT_REPOSITORY https://github.com/smolfetch/geotiv.git GIT_TAG 2.0.2)
FetchContent_MakeAvailable(geotiv)
list(APPEND ext_deps geotiv::geotiv)

# --------------------------------------------------------------------------------------------------
add_library(${project_name} INTERFACE)
add_library(${project_name}::${project_name} ALIAS ${project_name})
target_include_directories(${project_name} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
include_directories(include)

# --------------------------------------------------------------------------------------------------
include(GNUInstallDirs)

#Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#Install and export the INTERFACE target
install(TARGETS ${project_name} EXPORT ${project_name}Targets)

#Install the export file so that `find_package(${project_name})` works
install(EXPORT ${project_name}Targets
  FILE ${project_name}Targets.cmake
  NAMESPACE ${project_name}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name}
)

#Also export it for in‐source FetchContent usage
export(EXPORT ${project_name}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${project_name}Targets.cmake"
  NAMESPACE ${project_name}::
)


# --------------------------------------------------------------------------------------------------
# GridMap Examples - Automatic discovery of all .cpp files in examples/
if(${project_name_upper}_BUILD_EXAMPLES)
  file(GLOB exe CONFIGURE_DEPENDS examples/*.cpp)

  set(exec_names)
  foreach(src_file IN LISTS exe)
    get_filename_component(exec_name "${src_file}" NAME_WE)
    add_executable(${exec_name} "${src_file}")
    
    # Link with dependencies
    target_link_libraries(${exec_name} ${ext_deps})
    target_compile_options(${exec_name} PRIVATE ${params})
    
    install(TARGETS ${exec_name} DESTINATION bin)
    list(APPEND exec_names ${exec_name})
    
    message(STATUS "GridMap example: ${exec_name}")
  endforeach()
  
  # Create executable list file for reference
  if(exec_names)
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    foreach(exec IN LISTS exec_names)
      get_filename_component(exec_name "${exec}" NAME)
      file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
    endforeach()
  endif()
  
  install(TARGETS ${exec_names} DESTINATION lib/${PROJECT_NAME})
endif()


# --------------------------------------------------------------------------------------------------
# GridMap Tests - Automatic discovery of all .cpp files in test/ using DocTest framework
if(${project_name_upper}_ENABLE_TESTS)
  enable_testing()
  add_definitions(-DENABLE_DOCTEST_IN_LIBRARY)
  
  # Fetch DocTest framework
  FetchContent_Declare(
    DocTest
    GIT_REPOSITORY "https://github.com/doctest/doctest"
    GIT_TAG "v2.4.11"
  )
  FetchContent_MakeAvailable(DocTest)

  file(GLOB test_src CONFIGURE_DEPENDS test/*.cpp)

  set(test_names)
  foreach(src_file IN LISTS test_src)
    get_filename_component(test_name "${src_file}" NAME_WE)
    add_executable(${test_name} "${src_file}")
    
    # Link with dependencies and DocTest
    target_link_libraries(${test_name} ${ext_deps} doctest_with_main)
    target_compile_options(${test_name} PRIVATE ${params})
    
    # Register with CTest
    add_test(NAME ${test_name} COMMAND ${test_name})
    list(APPEND test_names ${test_name})
    
    message(STATUS "GridMap test: ${test_name}")
  endforeach()
endif()
