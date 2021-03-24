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

#ifndef TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_PROBLEMSOLVER_H_
#define TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_PROBLEMSOLVER_H_
#include <memory>
#include <BaseProblem.h>
#include <functional>
#include <source/TestSource.h>
namespace traact::test {
class ProblemSolver {
 public:
  typedef typename std::shared_ptr<ProblemSolver> Ptr;
  typedef typename std::function<void(TimestampType, Eigen::Affine3d)> DataCallback;
    typedef typename std::function<void(TimestampType)> CancelCallback;
  virtual ~ProblemSolver() = default;

  virtual void prepareProblem(Problem problem, const ProblemConfiguration &problem_configuration) =0;

  virtual void setSinkCallback(std::size_t idx, const DataCallback &callback) = 0;
  virtual void setInvalidCallback(std::size_t idx, const CancelCallback &callback) = 0;
  virtual void setSourceCallback(size_t idx, TestSource* source) = 0;

  virtual void start() = 0;

  virtual void stop() = 0;


};
}
#endif //TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_SOURCE_PROBLEMSOLVER_H_
