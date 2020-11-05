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

#include "SyncSourceTestCase.h"

traact::test::SyncSourceTestCase::SyncSourceTestCase(traact::test::Problem problem, bool simulate_sensors) : BasicTestCase(problem, simulate_sensors) {

    TimestampType startTime(std::chrono::milliseconds(10));
    TimeDurationType deltaTime(std::chrono::milliseconds(10));
    std::vector<TimeDurationType> delays;
    size_t num_events = 1000;

    problem_configuration_.sleep = false;
    //problem_configuration_.expected_input_time_delta = std::chrono::milliseconds (10);
    //problem_configuration_.time_delta.push_back(std::chrono::milliseconds (10));


    delays.resize(num_events);
    for(int i=0;i<num_events;++i){
        delays[i] = std::chrono::milliseconds(0);
    }

    double sin_offset = 0;
    double sin_per_second = 0.1;

    switch (problem_) {
        case Problem::Input1_Input2__Input3_Input4:{
            Eigen::Affine3d movement_source;
            source_configurations_.push_back(SourceConfiguration(simulate_sensors_, startTime,deltaTime, delays, num_events, sin_offset, sin_per_second  ));
        }
        case Problem::Input1_Input2__Input3:{
            Eigen::Affine3d movement_source;
            source_configurations_.push_back(SourceConfiguration(simulate_sensors_, startTime,deltaTime, delays, num_events, sin_offset, sin_per_second  ));
        }
        case Problem::Input1_Input2:{
            Eigen::Affine3d movement_source;
            source_configurations_.push_back(SourceConfiguration(simulate_sensors_, startTime,deltaTime, delays, num_events, sin_offset, sin_per_second  ));
        }
        case Problem::Input1:{
            Eigen::Affine3d movement_source;
            source_configurations_.push_back(SourceConfiguration(simulate_sensors_, startTime,deltaTime, delays, num_events, sin_offset, sin_per_second  ));

        }

    }


    ExpectedResult expectedResult(problem,source_configurations_);

    std::set<TimestampType > all_timestamps_set;
    for (const auto &source : expectedResult.sources){
        all_timestamps_set.insert(source.timestamps.begin(),source.timestamps.end());
    }
    std::vector<TimestampType > all_timestamps;
    all_timestamps.insert(all_timestamps.end(), all_timestamps_set.begin(),all_timestamps_set.end());
    std::sort(all_timestamps.begin(),all_timestamps.end());

    if(all_timestamps.size() > num_events){
        spdlog::error("more timestamps then events");
    }




    for (int ts_index = 0; ts_index < all_timestamps.size(); ++ts_index) {
        TimestampType ts = all_timestamps[ts_index];
        bool valid_ts = expectedResult.is_valid_ts(ts);
        Eigen::Affine3d expected_pose = expectedResult.get_pose(ts);


        expected_results_.emplace_back(std::make_tuple(ts, valid_ts, expected_pose));


    }

}

