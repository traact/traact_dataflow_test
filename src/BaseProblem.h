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

#ifndef TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_PROBLEM_BASEPROBLEM_H_
#define TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_PROBLEM_BASEPROBLEM_H_

#include <Eigen/Geometry>
#include <traact/traact.h>

namespace traact::test {

enum class Problem {
  Input1 = 0,
  Input1_Input2,
  Input1_Input2__Input3,
  Input1_Input2__Input3_Input4,
};

struct SourceConfiguration {
  bool sleep;
  TimestampType start_time;
  TimeDurationType time_delta;
  std::vector<TimeDurationType> delay_times;
  size_t num_events;
  Eigen::Affine3d movement;

  TimeDurationType getTimeDelayForIndex(size_t index) {
    size_t deltaIndex = index % delay_times.size();
    return delay_times[deltaIndex];
  }

};

struct ProblemConfiguration {
  bool sleep;
  std::vector<TimeDurationType> time_delta;
  TimeDurationType expected_input_time_delta;

  TimeDurationType getTimeDeltaForIndex(TimestampType ts) {
    size_t deltaIndex = ts.time_since_epoch().count() / expected_input_time_delta.count();
    deltaIndex = deltaIndex % time_delta.size();

    return time_delta[deltaIndex];
  }

};

struct BaseProblem {

  ProblemConfiguration problem_configuration_;

  Eigen::Affine3d execute(TimestampType ts, const Eigen::Affine3d &input1, const Eigen::Affine3d &input2) {

    if(problem_configuration_.sleep){
      TimestampType real_ts = now();
      TimestampType real_ts_until = now() + problem_configuration_.getTimeDeltaForIndex(ts);

      while(real_ts_until > real_ts)
        real_ts = now();
    }
    return input1 * input2;
  }
};

struct ExpectedResult {
  ExpectedResult(Problem problem, const std::vector<SourceConfiguration>& sources) {
    //TODO check for valid SourceConfigurations for problem

    //for now only sync case
    std::vector<Eigen::Affine3d> inputPoses;
    for (const auto &source : sources)
      inputPoses.push_back(source.movement);

    SourceConfiguration baseConfig = sources[0];
    expected_results.reserve(baseConfig.num_events);
    TimestampType ts = baseConfig.start_time;



    for (int ts_index = 0; ts_index < baseConfig.num_events; ++ts_index) {
      Eigen::Affine3d expected_pose;

      switch (problem){
        case Problem::Input1: {
          expected_pose = inputPoses[0];
          break;
        }
        case Problem::Input1_Input2: {
          expected_pose = inputPoses[0]*inputPoses[1];
          break;
        }
        case Problem::Input1_Input2__Input3: {
          expected_pose = (inputPoses[0]*inputPoses[1])*inputPoses[2];
          break;
        }
        case Problem::Input1_Input2__Input3_Input4: {
          expected_pose = (inputPoses[0]*inputPoses[1])*(inputPoses[2]*inputPoses[3]);
          break;
        }
      }

      expected_results.emplace_back(std::make_pair(ts, expected_pose));

      ts = ts+baseConfig.time_delta;
      for (int input_index = 0; input_index < inputPoses.size(); ++input_index) {
        inputPoses[input_index] = inputPoses[input_index] * sources[input_index].movement;
      }
    }
  }

  std::vector<std::pair<TimestampType , Eigen::Affine3d> > expected_results;


};

}
#endif //TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_PROBLEM_BASEPROBLEM_H_
