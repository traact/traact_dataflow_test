/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#include <gtest/gtest.h>

#include <dataflow_use_case_test/UseCaseFactory.h>
#include <dataflow_use_case_test/source_strategy/AllEvents_PerfectTimestamps.h>
#include <dataflow_use_case_test/source_strategy/MissingEvents_NoisyTimestamps.h>
#include <spdlog/spdlog.h>
#include <dataflow_use_case_test/DataflowTest.h>
#include "traact/TraactDataflowTestNetwork.h"

//TEST(Benchmark, In0In1_In2In3_Out0_PerfectData_NoDelay_NoWork) {
//    using namespace dataflow_use_case_test;
//
//    auto use_case = UseCaseFactory::Create(UseCaseProblem::In0In1_In2In3_Out0, std::make_unique<AllEvents_PerfectTimestamps>(), false,
//                                           false, 10000);
//
//
//
//    auto network = std::make_shared<traact::test::TraactDataflowTestNetwork>();
//    DataflowTest test(network, use_case);
//
//    EXPECT_TRUE(test.Test());
//}
//
//TEST(Benchmark, In0In1_In2In3_Out0_PerfectData_NoDelay_Work) {
//    using namespace dataflow_use_case_test;
//
//    auto use_case = UseCaseFactory::Create(UseCaseProblem::In0In1_In2In3_Out0, std::make_unique<AllEvents_PerfectTimestamps>(), false,
//                                           true, 10000);
//
//
//
//    auto network = std::make_shared<traact::test::TraactDataflowTestNetwork>();
//    DataflowTest test(network, use_case);
//
//    EXPECT_TRUE(test.Test());
//}
//
//TEST(Benchmark, In0In1_In2In3_Out0_MissingAndNoisyData_NoDelay_NoWork) {
//    using namespace dataflow_use_case_test;
//
//    auto use_case = UseCaseFactory::Create(UseCaseProblem::In0In1_In2In3_Out0, std::make_unique<MissingEvents_NoisyTimestamps>(), false,
//                                           false, 10000);
//
//
//
//    auto network = std::make_shared<traact::test::TraactDataflowTestNetwork>();
//    DataflowTest test(network, use_case);
//
//    EXPECT_TRUE(test.Test());
//}
//
//TEST(Benchmark, In0In1_In2In3_Out0_MissingAndNoisy_NoDelay_Work) {
//    using namespace dataflow_use_case_test;
//
//    auto use_case = UseCaseFactory::Create(UseCaseProblem::In0In1_In2In3_Out0, std::make_unique<MissingEvents_NoisyTimestamps>(), false,
//                                           true, 10000);
//
//
//
//    auto network = std::make_shared<traact::test::TraactDataflowTestNetwork>();
//    DataflowTest test(network, use_case);
//
//    EXPECT_TRUE(test.Test());
//}