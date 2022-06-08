cmake_minimum_required(VERSION 3.16)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CONAN_TRAACT_RUN_ENV_ROOT}/cmake)

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

include(traact_default_library_setup)

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

target_include_directories(${CONAN_PACKAGE_NAME} PUBLIC
        ${CMAKE_BINARY_DIR}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

install(TARGETS ${CONAN_PACKAGE_NAME}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )

add_subdirectory(benchmark)

