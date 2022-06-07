#include <gtest/gtest.h>

#include <dataflow_use_case_test/UseCaseFactory.h>
#include <dataflow_use_case_test/source_strategy/AllEvents_PerfectTimestamps.h>
#include <dataflow_use_case_test/source_strategy/MissingEvents_NoisyTimestamps.h>
#include <spdlog/spdlog.h>
#include <dataflow_use_case_test/DataflowTest.h>
#include "traact/TraactDataflowTestNetwork.h"

using TestParameter = std::tuple<int, int, bool, bool>;

class DefaultDataflowTest : public ::testing::TestWithParam<TestParameter> {

};

TEST_P(DefaultDataflowTest, TraactTaskFlow) {
    using namespace dataflow_use_case_test;
    int i_problem;
    int source_strategy;
    bool simulate_sensors;
    bool do_budy_work;
    std::tie(i_problem, source_strategy, simulate_sensors, do_budy_work) = GetParam();
    UseCaseProblem problem = (UseCaseProblem) i_problem;

    std::shared_ptr<UseCase> use_case;
    if (source_strategy == 0) {
        use_case = UseCaseFactory::Create(problem,
                                          std::make_unique<AllEvents_PerfectTimestamps>(),
                                          simulate_sensors,
                                          do_budy_work,
                                          100);
    } else {
        use_case = UseCaseFactory::Create(problem,
                                          std::make_unique<MissingEvents_NoisyTimestamps>(),
                                          simulate_sensors,
                                          do_budy_work,
                                          100);
    }

    auto network = std::make_shared<traact::test::TraactDataflowTestNetwork>();
    DataflowTest test(network, use_case);

    EXPECT_TRUE(test.Test());
}

struct PrintToStringParamName {
    std::string operator()(const ::testing::TestParamInfo<TestParameter> &info) const {
        using namespace dataflow_use_case_test;
        std::stringstream ss;
        int i_problem;
        int source_strategy;
        bool simulate_sensors;
        bool do_budy_work;
        std::tie(i_problem, source_strategy, simulate_sensors, do_budy_work) = info.param;
        UseCaseProblem problem = (UseCaseProblem) i_problem;

        ss << dataflow_use_case_test::UseCaseProblemToString(problem);
        //ss << "_" << (source_strategy == 0 ? "AllEvents_PerfectTs" : "MissingEvents_NoisyTs");
        ss << "_" << (simulate_sensors ? "Delay" : "NoDelay");
        ss << "_" << (do_budy_work ? "Work" : "NoWork");
        return ss.str();

    }
};

//INSTANTIATE_TEST_SUITE_P(
//        AllEvents_PerfectTimestamps,
//        DefaultDataflowTest,
//        ::testing::Combine(::testing::Range(0, 6),::testing::Values(0),::testing::Values(false),::testing::Values(false)),
//        PrintToStringParamName());

INSTANTIATE_TEST_SUITE_P(
    AllEvents_PerfectTimestamps,
    DefaultDataflowTest,
    ::testing::Combine(::testing::Range(0, 5), ::testing::Values(0), ::testing::Bool(), ::testing::Bool()),
    PrintToStringParamName());
INSTANTIATE_TEST_SUITE_P(
    MissingEvents_NoisyTimestamps,
    DefaultDataflowTest,
    ::testing::Combine(::testing::Range(0, 5), ::testing::Values(1), ::testing::Bool(), ::testing::Bool()),
    PrintToStringParamName());
