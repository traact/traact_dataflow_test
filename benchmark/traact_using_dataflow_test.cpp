/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/
#include <benchmark/benchmark.h>
#include <dataflow_use_case_test/UseCaseFactory.h>
#include <dataflow_use_case_test/source_strategy/AllEvents_PerfectTimestamps.h>
#include <dataflow_use_case_test/source_strategy/MissingEvents_NoisyTimestamps.h>
#include <spdlog/spdlog.h>
#include <dataflow_use_case_test/DataflowTest.h>
#include "../src/traact/TraactDataflowTestNetwork.h"

class TraactBuffer : public benchmark::Fixture {
 public:
    void SetUp(const ::benchmark::State &state) {

    }

    void TearDown(const ::benchmark::State &state) {

    }

};

static void bench_Traact(benchmark::State &state) {

    while (state.KeepRunning()) {
        using namespace dataflow_use_case_test;

        UseCaseProblem problem = UseCaseProblem::In0In1_In2In3_Out0;

        std::shared_ptr<UseCase> use_case = dataflow_use_case_test::UseCaseFactory::Create(problem,
                                                                                           std::make_unique<
                                                                                               AllEvents_PerfectTimestamps>(),
                                                                                           false,
                                                                                           false,
                                                                                           1000);
        auto network = std::make_shared<traact::test::TraactDataflowTestNetwork>();
        DataflowTest test(network, use_case);
        test.Test();

    }

}

//BENCHMARK(bench_Traact);