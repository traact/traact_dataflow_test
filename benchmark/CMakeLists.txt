add_executable(traact_dataflow_benchmark main.cpp ../src/traact/TraactDataflowTestNetwork.cpp ../src/traact/TraactDataflowTestNetwork.h traact_using_dataflow_test.cpp task_flow_async_sources.cpp task_flow_empty.cpp task_flow_async_tf_promise.cpp)

conan_target_link_libraries(traact_dataflow_benchmark)

target_include_directories(traact_dataflow_benchmark PUBLIC
        ${CMAKE_CURRENT_PROJECT_DIR}/src
        ${CMAKE_CURRENT_PROJECT_DIR}/benchmark
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_BINARY_DIR}
        )

install(TARGETS traact_dataflow_benchmark
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows