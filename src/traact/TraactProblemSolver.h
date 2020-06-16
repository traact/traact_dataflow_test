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

#ifndef TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_TRAACT_TRAACTPROBLEMSOLVER_H_
#define TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_TRAACT_TRAACTPROBLEMSOLVER_H_

#include <source/ProblemSolver.h>
#include <traact/traact.h>
#include <traact/facade/Facade.h>
#include <traact/component/facade/ApplicationSyncSink.h>
#include <traact/component/facade/ApplicationAsyncSource.h>
#include <traact/spatial.h>
namespace traact::test {
class TraactProblemSolver : public ProblemSolver{
 public:
  typedef typename traact::component::facade::ApplicationAsyncSource<spatial::Pose6DHeader> SourceType;
  typedef typename std::shared_ptr<component::facade::ApplicationSyncSink<spatial::Pose6DHeader> > SinkPtr;
  typedef typename std::shared_ptr<component::facade::ApplicationAsyncSource<spatial::Pose6DHeader> > SourcePtr;
  void prepareProblem(Problem problem) override;

  void setSinkCallback(const ProblemSolver::Callback &callback) override;
  void setSourceCallback(size_t index, TestSource *source) override;
  void start() override;
  void stop() override;

 private:
  DefaultInstanceGraphPtr pattern_graph_ptr_;

  std::shared_ptr<facade::Facade> facade_;

  SinkPtr sink_;
  std::vector<SourcePtr > sources_;

};
}

#endif //TRAACTMULTI_TRAACT_DATAFLOW_TEST_SRC_TRAACT_TRAACTPROBLEMSOLVER_H_
