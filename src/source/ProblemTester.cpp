/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include "ProblemTester.h"
#include "TestSource.h"
#include "TestSink.h"
#include <source/ProblemSolver.h>

#include <BaseProblem.h>
#include <traact/util/PerformanceMonitor.h>

traact::test::ProblemTester::ProblemTester(const traact::test::ProblemSolver::Ptr &solver,BasicTestCase::Ptr test_case) : solver_(solver), test_case_(test_case) {}

bool traact::test::ProblemTester::test() {


    const std::vector<std::tuple<TimestampType,bool, Eigen::Affine3d> >& expected_results = test_case_->GetExpectedResults();

    size_t expected_count = expected_results.size();
    solver_->prepareProblem(test_case_->GetProblem(), test_case_->GetProblemConfiguration());

    const std::vector<traact::test::SourceConfiguration>& source_configs = test_case_->GetSourceConfigurations();

    std::vector<std::shared_ptr<TestSource> > source_threads;

    for (int source_index = 0; source_index < source_configs.size(); ++source_index) {
        auto source_thread = std::make_shared<TestSource>(source_configs[source_index]);
        solver_->setSourceCallback(source_index, source_thread.get());
        source_threads.emplace_back(source_thread);
    }


    TestSink test_sink(expected_count);
    auto receiveCallback = std::bind(&TestSink::recieveInput, &test_sink, std::placeholders::_1, std::placeholders::_2);
    solver_->setSinkCallback(receiveCallback);

    TimestampType real_ts_source_start = now() + std::chrono::seconds(1);
    util::PerformanceMonitor monitor("Problem: " + std::to_string((int)test_case_->GetProblem()));
    {
        MEASURE_TIME(monitor, 0, "network start")
        solver_->start();
        for (const auto &item : source_threads) {
            item->Start(real_ts_source_start);
        }


    }

    {
        MEASURE_TIME(monitor, 1, "wait for finish source")
        for (const auto &item : source_threads) {
            item->WaitForFinish();
        }

    }

    {
        MEASURE_TIME(monitor, 2, "finish and wait for network stop")
        solver_->stop();
    }

    spdlog::info(monitor.toString());
    const auto &received_results = test_sink.data_;

    spdlog::info("received {0} of {1} results", received_results.size(), expected_results.size());

    spdlog::info("check for order of received results");
    std::map<TimestampType, Eigen::Affine3d> timestamp_to_result;
    std::map<TimestampType, TimestampType> timestamp_to_send;
    std::map<TimestampType, TimestampType> timestamp_to_receive;
    if (received_results.size() > 1) {
        // store first
        TimestampType ts = std::get<0>(received_results[0]);
        timestamp_to_result[ts] = std::get<1>(received_results[0]);
        timestamp_to_receive[ts] = std::get<2>(received_results[0]);

        // check with previous
        for (int ts_index = 1; ts_index < received_results.size(); ++ts_index) {
            TimestampType received_ts = std::get<0>(received_results[ts_index]);
            if (received_ts < ts) {
                spdlog::error("current {0} < previous {1} ", received_ts.time_since_epoch().count(),
                              ts.time_since_epoch().count());
            }

            ts = received_ts;
            timestamp_to_result[ts] = std::get<1>(received_results[ts_index]);
            timestamp_to_receive[ts] = std::get<2>(received_results[ts_index]);

        }


        for (const auto &item : source_threads[0]->data_) {
            timestamp_to_send[item.first] = item.second;
        }
    }

    spdlog::info("check for correct results");
    bool allOK = true;
    for (const auto &expected : expected_results) {
        auto expected_ts = std::get<0>(expected);
        auto expected_valid = std::get<1>(expected);
        auto expected_value = std::get<2>(expected);
        auto find_it = timestamp_to_result.find(expected_ts);
        if (expected_valid && find_it == timestamp_to_result.end()) {
            spdlog::error("missing result for timestamp {0}", expected_ts.time_since_epoch().count());
            allOK = false;
            continue;
        }
        if (!expected_valid && find_it != timestamp_to_result.end()) {
            spdlog::error("result for invalid timestamp {0} exists", expected_ts.time_since_epoch().count());
            allOK = false;
            continue;
        }

        Eigen::Affine3d result = find_it->second;
        if (!result.isApprox(expected_value)) {
            Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
            std::stringstream ss_expected, ss_result;
            ss_expected << expected_value.matrix().format(CleanFmt);
            ss_result << result.matrix().format(CleanFmt);
            allOK = false;
            spdlog::error("result differs for ts {2}, expected: \n {0} \n result:\n {1}", ss_expected.str(),
                          ss_result.str(), expected_ts.time_since_epoch().count());
        }

    }

    if (allOK)
        spdlog::info("all results correct");

    spdlog::info("calculate delay of {0} results: ", timestamp_to_receive.size());
    if (!timestamp_to_receive.empty()) {
        size_t result_count = 0;
        TimeDurationType total_delay = TimeDurationType::min();

        for (const auto &receive_ts : timestamp_to_receive) {
            TimestampType sendTime = timestamp_to_send[receive_ts.first];
            total_delay += (receive_ts.second - sendTime);
            result_count++;
        }
        using nanoToMilliseconds = std::chrono::duration<float, std::milli>;
        using nanoToSeconds = std::chrono::duration<float>;
        spdlog::info("average delay: {0}ns", total_delay.count() / result_count);
    }

    return allOK;


}
