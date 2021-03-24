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

#ifndef TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_TESTSINK_H_
#define TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_TESTSINK_H_

#include <Eigen/Geometry>
#include <traact/datatypes.h>
#include <traact/util/Logging.h>
//#include <boost/progress.hpp>
namespace traact::test {
class TestSink {
 public:
  TestSink(size_t expected_count){
    data_.reserve(expected_count);
      invalid_data_.reserve(expected_count);
  }
  void receiveInput(TimestampType ts, const Eigen::Affine3d & data){
    data_.emplace_back(ts, data, now());
  }

  void invalidTimestamp(TimestampType ts) {
      invalid_data_.emplace_back(ts, now());
  }

  std::vector<std::tuple<TimestampType , Eigen::Affine3d, TimestampType> > data_;
  std::vector<std::tuple<TimestampType , TimestampType> > invalid_data_;

};
}

#endif //TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_TESTSINK_H_
