cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

# conan is required hard, library name, version and description are configured from conan
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

conan_basic_setup(TARGETS)

if (UNIX)
    if (APPLE)
        MESSAGE(STATUS "Building for Macos.")
        add_definitions(-DTRAACT_TARGET_APPLE=1)
    endif ()
    MESSAGE(STATUS "Building for Unix.")
    add_definitions(-DTRAACT_TARGET_UNIX=1)
elseif (WIN32)
    MESSAGE(STATUS "Building for Windows.")
    add_definitions(-DTRAACT_TARGET_WINDOWS=1)
endif ()

# public header
set(DATAFLOW_TEST_SOURCES
        src/test_main.cpp
        src/traact/TraactDataflowTestNetwork.cpp
        src/traact/TraactDataflowTestNetwork.h
        src/default_tests.cpp
        src/benchmark_test.cpp
        src/traact/TraactDataflowTestPlugin.cpp
        src/traact/TraactDataflowTestPlugin.h
        )

add_executable(${CONAN_PACKAGE_NAME} ${DATAFLOW_TEST_SOURCES})

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

target_include_directories(${CONAN_PACKAGE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_BINARY_DIR}
        )

install(TARGETS ${CONAN_PACKAGE_NAME}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

#add_subdirectory(src_traact_plugin)
add_subdirectory(benchmark)

