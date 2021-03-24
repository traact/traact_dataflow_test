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
#include "../../src_traact_plugin/TraactDataflowTestPlugin.h"
#include <traact/util/Logging.h>
void
traact::test::TraactProblemSolver::prepareProblem(Problem problem, const ProblemConfiguration &problem_configuration) {
  facade_ = std::make_shared<facade::Facade>();
  sink_.clear();
  sources_.clear();

  pattern_graph_ptr_ = std::make_shared<DefaultInstanceGraph>("test1");
  switch (problem){
    case Problem ::Input1:{
      DefaultPatternInstancePtr
          source_pattern =
          pattern_graph_ptr_->addPattern("source", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
      DefaultPatternInstancePtr
          sink_pattern =
          pattern_graph_ptr_->addPattern("sink", facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));

      pattern_graph_ptr_->connect("source", "output", "sink", "input");

        buffer::TimeDomainManagerConfig td_config;
        td_config.time_domain = 0;
        td_config.ringbuffer_size = 30;
        td_config.master_source = "source";
        td_config.source_mode = SourceMode::WaitForBuffer;
        td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
        td_config.max_offset = std::chrono::milliseconds(1);
        td_config.max_delay = std::chrono::milliseconds(100);
        td_config.measurement_delta = std::chrono::milliseconds(10);

        pattern_graph_ptr_->timedomain_configs[0] = td_config;

      facade_->loadDataflow(pattern_graph_ptr_);

        sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source")));

      DefaultComponentPtr sink = facade_->getComponent("sink");
        sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink));
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
          pattern_graph_ptr_->addPattern("mul", facade_->instantiatePattern("DataflowTestComponent"));

      pattern_graph_ptr_->connect("source", "output", "mul", "input0");
      pattern_graph_ptr_->connect("source2", "output", "mul", "input1");
      pattern_graph_ptr_->connect("mul", "output", "sink", "input");

      buffer::TimeDomainManagerConfig td_config;
      td_config.time_domain = 0;
      td_config.ringbuffer_size = 30;
      td_config.master_source = "source";
      td_config.source_mode = SourceMode::WaitForBuffer;
      td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
      td_config.max_offset = std::chrono::milliseconds(1);
      td_config.max_delay = std::chrono::milliseconds(100);
      td_config.measurement_delta = std::chrono::milliseconds(10);

      pattern_graph_ptr_->timedomain_configs[0] = td_config;

      facade_->loadDataflow(pattern_graph_ptr_);

        sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source")));
      sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source2")));

      DefaultComponentPtr sink = facade_->getComponent("sink");
        sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink));

        std::shared_ptr<test::DataflowTestComponent>
                mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul"));

        mul_pattern_tmp->base_problem_.problem_configuration_ = problem_configuration;
      break;
    }
      case Problem ::Input1_Input2__Input3:{
          DefaultPatternInstancePtr
                  source_pattern =
                  pattern_graph_ptr_->addPattern("source", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  source2_pattern =
                  pattern_graph_ptr_->addPattern("source2", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  source3_pattern =
                  pattern_graph_ptr_->addPattern("source3", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  sink_pattern =
                  pattern_graph_ptr_->addPattern("sink", facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  mul_pattern =
                  pattern_graph_ptr_->addPattern("mul1", facade_->instantiatePattern("DataflowTestComponent"));
          DefaultPatternInstancePtr
                  mul_pattern2 =
                  pattern_graph_ptr_->addPattern("mul2", facade_->instantiatePattern("DataflowTestComponent"));


          pattern_graph_ptr_->connect("source", "output", "mul1", "input0");
          pattern_graph_ptr_->connect("source2", "output", "mul1", "input1");
          pattern_graph_ptr_->connect("mul1", "output", "mul2", "input0");
          pattern_graph_ptr_->connect("source3", "output", "mul2", "input1");
          pattern_graph_ptr_->connect("mul2", "output", "sink", "input");

          buffer::TimeDomainManagerConfig td_config;
          td_config.time_domain = 0;
          td_config.ringbuffer_size = 30;
          td_config.master_source = "source";
          td_config.source_mode = SourceMode::WaitForBuffer;
          td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
          td_config.max_offset = std::chrono::milliseconds(1);
          td_config.max_delay = std::chrono::milliseconds(100);
          td_config.measurement_delta = std::chrono::milliseconds(10);

          pattern_graph_ptr_->timedomain_configs[0] = td_config;


          facade_->loadDataflow(pattern_graph_ptr_);

          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source")));
          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source2")));
          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source3")));

          DefaultComponentPtr sink = facade_->getComponent("sink");
          sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink));


          std::shared_ptr<test::DataflowTestComponent>
                  mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul1"));

          mul_pattern_tmp->base_problem_.problem_configuration_ = problem_configuration;

          std::shared_ptr<test::DataflowTestComponent>
                  mul_pattern2_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul2"));

          mul_pattern2_tmp->base_problem_.problem_configuration_ = problem_configuration;
          break;
      }

      case Problem ::Input1_Input2__Input3_Input4:{
          DefaultPatternInstancePtr
                  source_pattern =
                  pattern_graph_ptr_->addPattern("source", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  source2_pattern =
                  pattern_graph_ptr_->addPattern("source2", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  source3_pattern =
                  pattern_graph_ptr_->addPattern("source3", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  source4_pattern =
                  pattern_graph_ptr_->addPattern("source4", facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  sink_pattern =
                  pattern_graph_ptr_->addPattern("sink", facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
          DefaultPatternInstancePtr
                  mul_pattern =
                  pattern_graph_ptr_->addPattern("mul1", facade_->instantiatePattern("DataflowTestComponent"));
          DefaultPatternInstancePtr
                  mul_pattern2 =
                  pattern_graph_ptr_->addPattern("mul2", facade_->instantiatePattern("DataflowTestComponent"));
          DefaultPatternInstancePtr
                  mul_pattern3 =
                  pattern_graph_ptr_->addPattern("mul3", facade_->instantiatePattern("DataflowTestComponent"));

          pattern_graph_ptr_->connect("source", "output", "mul1", "input0");
          pattern_graph_ptr_->connect("source2", "output", "mul1", "input1");
          pattern_graph_ptr_->connect("mul1", "output", "mul3", "input0");
          pattern_graph_ptr_->connect("source3", "output", "mul2", "input0");
          pattern_graph_ptr_->connect("source4", "output", "mul2", "input1");
          pattern_graph_ptr_->connect("mul1", "output", "mul3", "input0");
          pattern_graph_ptr_->connect("mul2", "output", "mul3", "input1");
          pattern_graph_ptr_->connect("mul3", "output", "sink", "input");

          buffer::TimeDomainManagerConfig td_config;
          td_config.time_domain = 0;
          td_config.ringbuffer_size = 30;
          td_config.master_source = "source";
          td_config.source_mode = SourceMode::WaitForBuffer;
          td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
          td_config.max_offset = std::chrono::milliseconds(1);
          td_config.max_delay = std::chrono::milliseconds(100);
          td_config.measurement_delta = std::chrono::milliseconds(10);

          pattern_graph_ptr_->timedomain_configs[0] = td_config;


          facade_->loadDataflow(pattern_graph_ptr_);


          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source")));
          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source2")));
          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source3")));
          sources_.emplace_back(std::dynamic_pointer_cast<SourceType >(facade_->getComponent("source4")));

          DefaultComponentPtr sink = facade_->getComponent("sink");
          sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink));


          std::shared_ptr<test::DataflowTestComponent>
                  mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul1"));

          mul_pattern_tmp->base_problem_.problem_configuration_ = problem_configuration;

          std::shared_ptr<test::DataflowTestComponent>
                  mul_pattern2_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul2"));

          mul_pattern2_tmp->base_problem_.problem_configuration_ = problem_configuration;

          std::shared_ptr<test::DataflowTestComponent>
                  mul_pattern3_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul3"));

          mul_pattern3_tmp->base_problem_.problem_configuration_ = problem_configuration;
          break;
      }
  }

}

void traact::test::TraactProblemSolver::setSinkCallback(std::size_t idx, const DataCallback &callback) {
  sink_[idx]->SetCallback(callback);
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

void traact::test::TraactProblemSolver::setInvalidCallback(std::size_t idx,
                                                           const traact::test::ProblemSolver::CancelCallback &callback) {
    //invalid_callbacks_[idx] = callback;
    sink_[idx]->SetInvalidCallback(std::bind(callback, std::placeholders::_1));
    //sink_[idx]->SetInvalidCallback(std::bind(&TraactProblemSolver::invalid_callback, this, std::placeholders::_1, std::placeholders::_2, idx));
}

void traact::test::TraactProblemSolver::invalid_callback(traact::TimestampType ts, std::size_t mea_idx,
                                                         std::size_t sink_idx) {
    invalid_callbacks_[sink_idx](ts);
}

traact::test::TraactProblemSolver::TraactProblemSolver() {

}
