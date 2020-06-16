cmake_minimum_required(VERSION 3.9)

set(CMAKE_CXX_STANDARD 17)

# conan is required hard, library name, version and description are configured from conan
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

conan_basic_setup(TARGETS)

if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
    add_definitions(-DTBB_USE_DEBUG=1)
endif ()

add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_FEATURES=1)
add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_NODES=1)

# public header
set(DATAFLOW_TEST_SOURCES
        src/DataflowTest.cpp
        src/source/TestSource.cpp
        src/source/TestSource.h
        src/source/TestSink.cpp
        src/source/TestSink.h
        src/BaseProblem.h
        src/source/ProblemTester.cpp
        src/source/ProblemTester.h
        src/source/ProblemSolver.h
        src/traact/TraactProblemSolver.cpp
        src/traact/TraactProblemSolver.h)

add_executable(DataflowTest ${DATAFLOW_TEST_SOURCES})

conan_target_link_libraries(DataflowTest)

target_include_directories(DataflowTest PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR}
        )

install(TARGETS DataflowTest
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows