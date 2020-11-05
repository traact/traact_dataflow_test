cmake_minimum_required(VERSION 3.16)


set(PLUGIN_SOURCES TraactDataflowTestPlugin.cpp TraactDataflowTestPlugin.h)


add_library(traact_dataflowtest_plugin SHARED
        ${PLUGIN_SOURCES}        
        )

conan_target_link_libraries(traact_dataflowtest_plugin)

# don't set version number so that (linux) only one file exists
#set_target_properties(traact_dataflowtest_plugin PROPERTIES VERSION ${CONAN_PACKAGE_VERSION})

target_include_directories(traact_dataflowtest_plugin PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(traact_dataflowtest_plugin PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src_traact_plugin
        )

IF (NOT WIN32)
    MESSAGE(STATUS "Set boost::ublas alignment to 16")
    set_target_properties(traact_dataflowtest_plugin PROPERTIES COMPILE_DEFINITIONS "BOOST_UBLAS_BOUNDED_ARRAY_ALIGN=__attribute__ ((aligned (16)))")
ENDIF (NOT WIN32)

install(TARGETS traact_dataflowtest_plugin
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

