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

#include "TraactProblemSolver.h"
void traact::test::TraactProblemSolver::prepareProblem(traact::test::Problem problem) {
  facade_ = std::make_shared<facade::Facade>();
  sink_.reset();
  sources_.clear();

  pattern_graph_ptr_ = std::make_shared<DefaultInstanceGraph>("test1");
  switch (problem){
    case Problem ::Input1:{
      DefaultPatternInstancePtr
          source_pattern =
          pattern_graph_ptr_->addPattern("source", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
      DefaultPatternInstancePtr
          sink_pattern =
          //pattern_graph_ptr->addPattern("sink", myfacade.instantiatePattern("Pose6DPrint"));
          pattern_graph_ptr_->addPattern("sink", facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));

      pattern_graph_ptr_->connect("source", "output", "sink", "input");

      facade_->loadDataflow(pattern_graph_ptr_);

      DefaultComponentPtr source = facade_->getComponent("source");
      auto app_source = std::dynamic_pointer_cast<SourceType >(source);
      sources_.emplace_back(app_source);

      DefaultComponentPtr sink = facade_->getComponent("sink");
      sink_ = std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink);
      break;
    }
    case Problem ::Input1_Input2:{
      DefaultPatternInstancePtr
          source_pattern =
          pattern_graph_ptr_->addPattern("source", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
      DefaultPatternInstancePtr
          source2_pattern =
          pattern_graph_ptr_->addPattern("source2", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
      DefaultPatternInstancePtr
          sink_pattern =
          pattern_graph_ptr_->addPattern("sink", facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
      DefaultPatternInstancePtr
          mul_pattern =
          pattern_graph_ptr_->addPattern("mul", facade_->instantiatePattern("MultiplicationComponent"));

      pattern_graph_ptr_->connect("source", "output", "mul", "input0");
      pattern_graph_ptr_->connect("source2", "output", "mul", "input1");
      pattern_graph_ptr_->connect("mul", "output", "sink", "input");

      facade_->loadDataflow(pattern_graph_ptr_);

      DefaultComponentPtr source = facade_->getComponent("source");
      auto app_source = std::dynamic_pointer_cast<SourceType >(source);
      sources_.emplace_back(app_source);
      sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source2")));

      DefaultComponentPtr sink = facade_->getComponent("sink");
      sink_ = std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink);
      break;
    }
  }

}

void traact::test::TraactProblemSolver::setSinkCallback(const traact::test::ProblemSolver::Callback &callback) {
  sink_->SetCallback(callback);
}
void traact::test::TraactProblemSolver::start() {
  facade_->start();


}
void traact::test::TraactProblemSolver::stop() {
  facade_->stop();
}

void traact::test::TraactProblemSolver::setSourceCallback(size_t index, traact::test::TestSource *source) {
  source->SetCallback(std::bind(&SourceType::newValue, sources_[index], std::placeholders::_1, std::placeholders::_2));
}
