cmake_minimum_required(VERSION 2.6)
project(ccrrt)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
message(STATUS "EIGEN: ${Eigen_INCLUDE_DIRS}")

add_definitions("-Wall")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB lib_source "src/*.cpp" "src/*.h" "ccrrt/*.h")
list(SORT lib_source)

file(GLOB unit_tests_source "test/*.cpp")
LIST(SORT unit_tests_source)


find_package(OpenSceneGraph)
include_directories(${OpenSceneGraph_INCLUDE_DIRS})
add_definitions(${OpenSourceGraph_DEFINITIONS})
link_libraries(akin osg osgViewer osgDB osgAkin)

add_library(${PROJECT_NAME} SHARED ${lib_source})

enable_testing()

message(STATUS "\n -- UNIT TEST: ")
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${PROJECT_NAME} ${OpenSceneGraph_LIBRARIES})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(utest_src_file)
