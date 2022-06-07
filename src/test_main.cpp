#include <gtest/gtest.h>
//#include <spdlog/sinks/stdout_color_sinks.h>
#include <traact/util/Logging.h>

int main(int argc, char **argv) {

    //
//    try {
//        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
//        console_sink->set_level(spdlog::level::trace);
//        console_sink->set_pattern("[%^%l%$] %v");
//
//    }
//    catch (const spdlog::spdlog_ex &ex) {
//        std::cout << "Log initialization failed: " << ex.what() << std::endl;
//    }
    traact::util::initLogging(spdlog::level::trace, "");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
