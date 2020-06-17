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

#ifndef TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_TESTSOURCE_H_
#define TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_TESTSOURCE_H_

#include <traact/component/facade/ApplicationAsyncSource.h>
#include <traact/spatial.h>
#include "BaseProblem.h"

namespace traact::test {




class TestSource {
 public:
  typedef typename std::function<void(TimestampType, Eigen::Affine3d)> Callback;
  explicit TestSource(const SourceConfiguration& config) : source_configuration_(config)  {
    internal_data_.setIdentity();
    running_ = false;
    callback_ = nullptr;
  }

  bool Start(TimestampType real_ts_start) {
    running_ = true;
    real_ts_start_ = real_ts_start;
    spdlog::info("starting SourceThread");
    thread_.reset(new std::thread(std::bind(&TestSource::threadLoop, this)));
	return true;
  }
  bool Stop() {
    spdlog::info("stopping SourceThread");
    if (running_) {
      running_ = false;
      thread_->join();
    }
	return true;
  }

  void WaitForFinish() {
    thread_->join();
  }

  // send timestamp, real time of send
  std::vector<std::pair<TimestampType , TimestampType> > data_;

  void SetCallback(const Callback &callback) {
    callback_ = callback;
  }
 private:
  SourceConfiguration source_configuration_;
  Eigen::Affine3d internal_data_;
  std::shared_ptr<std::thread> thread_;
  bool running_;
  Callback callback_;
  TimestampType real_ts_start_;

  void threadLoop() {
    using namespace traact::spatial;
    using namespace traact;

    // init runtime parameter
    data_.reserve(source_configuration_.num_events);
    size_t output_count = 0;
    TimestampType ts = source_configuration_.start_time;
    internal_data_ = source_configuration_.movement;

    TimestampType next_real_ts = real_ts_start_;
    TimestampType current_real_ts = now();


    while (running_ && output_count < source_configuration_.num_events) {
      if(source_configuration_.sleep) {
        while(next_real_ts > current_real_ts){
          current_real_ts = now();
        }
      }


      data_.emplace_back(std::make_pair(ts, now()));
      callback_(ts, internal_data_);

      internal_data_ = internal_data_ * source_configuration_.movement;

      ts = ts + source_configuration_.time_delta;
      next_real_ts = next_real_ts + source_configuration_.getTimeDelayForIndex(output_count);
      output_count++;

    }
    spdlog::trace("source quit loop");
    running_ = false;
  }

};

}

#endif //TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_TESTSOURCE_H_
