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
    SourceConfiguration() {}

    SourceConfiguration(bool sleep, const TimestampType &startTime, const TimeDurationType &timeDelta,
                        const std::vector<TimeDurationType> &delayTimes, size_t numEvents,
                        const double sin_offset, const double sin_per_second) : sleep(sleep), start_time(startTime), time_delta(timeDelta),
                                                           delay_times(delayTimes), num_events(numEvents),
                                                                           sin_offset(sin_offset),sin_per_second(sin_per_second) {}

    bool sleep;
  TimestampType start_time;
  TimeDurationType time_delta;
  std::vector<TimeDurationType> delay_times;
  size_t num_events;
    double sin_offset;
    double sin_per_second;

  TimeDurationType getTimeDelayForIndex(size_t index) {
    size_t deltaIndex = index % delay_times.size();
    return delay_times[deltaIndex];
  }

};

struct ProblemConfiguration {
  bool sleep{false};
  std::vector<TimeDurationType> time_delta;
  TimeDurationType expected_input_time_delta;

  TimeDurationType getTimeDeltaForTs(TimestampType ts) {
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
      TimestampType real_ts_until = now() + problem_configuration_.getTimeDeltaForTs(ts);

      while(real_ts_until > real_ts)
        real_ts = now();
    }
    return input1 * input2;
  }
};

struct ExpectedSource {
    ExpectedSource();
    ExpectedSource(SourceConfiguration config) : configuration(config) {
        timestamps.resize(configuration.num_events);

        TimestampType current_ts = configuration.start_time;
        for(int i =0; i< configuration.num_events;++i){
            timestamps[i] = current_ts;
            current_ts = current_ts + configuration.time_delta;
        }
    }

    SourceConfiguration configuration;

    std::vector<TimestampType> timestamps;

    bool is_timestamp(TimestampType ts){
        return std::find(timestamps.begin(), timestamps.end(), ts) != timestamps.end();
    }

    Eigen::Affine3d getPose(TimestampType ts) {
        using nanoMilliseconds = std::chrono::duration<double, std::milli>;
        double seconds = nanoMilliseconds(ts.time_since_epoch()).count();

        double p = seconds * configuration.sin_per_second;
        double pos = std::sin(configuration.sin_offset + p);

        Eigen::Affine3d result = Eigen::Affine3d::Identity();
        result.translation() = Eigen::Vector3d (0,0, pos);


        return result;

    }


};

struct ExpectedResult {
    ExpectedResult(Problem p, const std::vector<SourceConfiguration> &source_configs) : problem(p)
    {
        for(auto source_config : source_configs){
            sources.emplace_back(ExpectedSource(source_config));
        }
    }

    std::vector<ExpectedSource> sources;
    Problem problem;

    bool is_valid_ts(TimestampType ts){
        bool result = true;
        for(auto& source : sources){
            result = result && source.is_timestamp(ts);
        }

        return result;
    }

    Eigen::Affine3d get_pose(TimestampType ts){

        switch (problem) {
            case Problem::Input1:{
                return sources[0].getPose(ts);
            }
            case Problem::Input1_Input2:{
                return sources[0].getPose(ts)*sources[1].getPose(ts);
            }
            case Problem::Input1_Input2__Input3:{
                return (sources[0].getPose(ts)*sources[1].getPose(ts))*sources[2].getPose(ts);
            }
            case Problem::Input1_Input2__Input3_Input4:{
                return (sources[0].getPose(ts)*sources[1].getPose(ts))*(sources[2].getPose(ts)*sources[3].getPose(ts));
            }


        }

    }


};

}
#endif //TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_PROBLEM_BASEPROBLEM_H_
