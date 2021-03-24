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

#include "TestSource.h"

void traact::test::TestSource::threadLoop() {
    using namespace traact::spatial;
    using namespace traact;

    // Init runtime parameter
    data_.reserve(source_configuration_.num_events);
    size_t output_count = 0;


    TimestampType next_real_ts = real_ts_start_;
    TimestampType current_real_ts = now();


    while (running_ && output_count < source_configuration_.num_events) {
        if(source_configuration_.sleep) {

            while(next_real_ts > current_real_ts){
                std::chrono::milliseconds time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(next_real_ts - current_real_ts);
                if(time_diff.count() > 1) {
                    std::this_thread::sleep_for( time_diff - std::chrono::milliseconds(1));
                } else {
                    std::this_thread::yield();
                }
                current_real_ts = now();
            }
        }


        TimestampType ts = expected_source_.timestamps[output_count];
        auto pose = expected_source_.getPose(ts);
        data_.emplace_back(std::make_pair(ts, now()));
        callback_(ts, pose);

        next_real_ts = next_real_ts + source_configuration_.time_delta+ source_configuration_.getTimeDelayForIndex(output_count);
        output_count++;


    }
    spdlog::trace("source quit loop");
    running_ = false;
}
