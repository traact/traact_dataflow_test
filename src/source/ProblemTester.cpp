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

    TimeDurationType thread_start_offset = std::chrono::seconds(1);
    bool simulate_sensor = test_case_->GetSourceConfigurations()[0].sleep;

    const std::vector<std::tuple<TimestampType,bool, Eigen::Affine3d> >& expected_results = test_case_->GetExpectedResults();

    size_t expected_count = expected_results.size();
    solver_->prepareProblem(test_case_->GetProblem(), test_case_->GetProblemConfiguration());


    const std::vector<traact::test::SourceConfiguration>& source_configs = test_case_->GetSourceConfigurations();



    std::vector<std::shared_ptr<TestSource> > source_threads;

    std::size_t master_idx = 0;

    for (int source_index = 0; source_index < source_configs.size(); ++source_index) {
        auto source_thread = std::make_shared<TestSource>(source_configs[source_index]);
        solver_->setSourceCallback(source_index, source_thread.get());
        source_threads.emplace_back(source_thread);
    }


    TestSink test_sink(expected_count);
    auto receiveCallback = std::bind(&TestSink::receiveInput, &test_sink, std::placeholders::_1, std::placeholders::_2);
    auto invalidCallback = std::bind(&TestSink::invalidTimestamp, &test_sink, std::placeholders::_1);
    solver_->setSinkCallback(0, receiveCallback);
    solver_->setInvalidCallback(0, invalidCallback);

    TimestampType real_ts_source_start = now() + thread_start_offset;
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
        MEASURE_TIME(monitor, 2, "finish and wait for network Stop")
        solver_->stop();
    }

    if(simulate_sensor){
        spdlog::info("Remove {0} nanoseconds from source time, threads wait to have a somewhat simultaneous start", thread_start_offset.count());
    }
    spdlog::info(monitor.toString());
    using nanoToMilliseconds = std::chrono::duration<float, std::micro>;



    const auto &received_results = test_sink.data_;
    const auto &received_invalid = test_sink.invalid_data_;
    std::size_t expected_valid_results = 0;
    std::size_t expected_invalid_results = 0;

    for(const auto& exp : expected_results){
        if(std::get<1>(exp)){
            expected_valid_results++;
        }else {
            expected_invalid_results++;
        }
    }
    spdlog::info("Received Events: Valid Results: {0} of {1}, Invalid Results {2} of {3} ", received_results.size(), expected_valid_results, received_invalid.size(), expected_invalid_results);
    bool allOK = true;
    if(received_results.size() != expected_valid_results){
        spdlog::error("incorrect number of valid results");
        allOK = false;
    }
    if(received_invalid.size() < expected_invalid_results){
        spdlog::error("smaller number of invalid results");
        allOK = false;
    }
    spdlog::info("check for order of received results");
    std::map<TimestampType, Eigen::Affine3d> timestamp_to_result;
    std::vector<std::map<TimestampType, TimestampType> > source_timestamp_to_send;
    std::map<TimestampType, TimestampType> timestamp_to_receive;
    source_timestamp_to_send.resize(source_threads.size());
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
                allOK = false;
            }

            ts = received_ts;
            timestamp_to_result[ts] = std::get<1>(received_results[ts_index]);
            timestamp_to_receive[ts] = std::get<2>(received_results[ts_index]);

        }


        for(std::size_t source_idx = 0; source_idx < source_threads.size();++source_idx){
            for (const auto &item : source_threads[source_idx]->data_) {
                source_timestamp_to_send[source_idx][item.first] = item.second;
            }
        }

    }

    spdlog::info("check for correct results");

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

        if(expected_valid) {
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
        } else {
            // no result for invalid timestamp, all good
        }


    }

    if (allOK)
        spdlog::info("all results correct");

    spdlog::info("calculate delay of {0} results: ", timestamp_to_receive.size());
    if (!timestamp_to_receive.empty()) {


        TimeDurationType total_time;
        if(simulate_sensor)
            total_time = monitor.getTotalTime() - thread_start_offset;
        else
            total_time = monitor.getTotalTime();
        auto avg_per_mea = nanoToMilliseconds (total_time / (received_results.size()+received_invalid.size()));
        spdlog::info("average time per measurement: {0} micro seconds", avg_per_mea.count());
        double fps = std::chrono::seconds(1) / avg_per_mea;
        spdlog::info("measurements per second: {0}",  fps);

        for(std::size_t source_idx = 0; source_idx < source_threads.size();++source_idx){

            TimeDurationType total_delay = TimeDurationType::min();

            for (const auto &receive_ts : timestamp_to_receive) {
                TimestampType sendTime = source_timestamp_to_send[source_idx][receive_ts.first];
                auto ts_diff = receive_ts.second - sendTime;
                total_delay += ts_diff;
            }

            auto total_micro = nanoToMilliseconds (total_delay);
            spdlog::info("average delay source idx {0}: {1} micro seconds",source_idx, total_micro.count() / timestamp_to_receive.size());


        }


    }

    return allOK;


}
