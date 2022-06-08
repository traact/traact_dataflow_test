/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "TraactDataflowTestNetwork.h"
#include "TraactDataflowTestPlugin.h"
#include <traact/util/Logging.h>
#include <traact/facade/DefaultFacade.h>

namespace traact::test {
void TraactDataflowTestNetwork::Init(dataflow_use_case_test::UseCaseProblem problem,
                                     const dataflow_use_case_test::BaseProblemList &problem_list,
                                     const dataflow_use_case_test::ResultCallback &result_callback,
                                     const dataflow_use_case_test::InvalidCallback &invalid_callback,
                                     dataflow_use_case_test::TimeDurationType max_ts_offset) {
    using namespace dataflow_use_case_test;

    const int ringbuffer_size = 3;

    result_callback_ = result_callback;
    invalid_callback_ = invalid_callback;
    facade_ = std::make_shared<facade::DefaultFacade>();
    sink_.clear();
    sources_.clear();

    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = ringbuffer_size;
    td_config.master_source = "source";
    td_config.source_mode = SourceMode::WAIT_FOR_BUFFER;
    td_config.missing_source_event_mode = MissingSourceEventMode::WAIT_FOR_EVENT;
    td_config.max_offset = max_ts_offset / 2;
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::milliseconds(10);

    pattern_graph_ptr_ = std::make_shared<DefaultInstanceGraph>("test1");
    switch (problem) {
        case UseCaseProblem::In0_Out0: {
            DefaultPatternInstancePtr
                source_pattern =
                pattern_graph_ptr_->addPattern("source",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                sink_pattern =
                pattern_graph_ptr_->addPattern("sink",
                                               facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));

            pattern_graph_ptr_->connect("source", "output", "sink", "input");

            pattern_graph_ptr_->timedomain_configs[0] = td_config;

            facade_->loadDataflow(pattern_graph_ptr_);

            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source")));

            DefaultComponentPtr sink = facade_->getComponent("sink");
            sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(
                sink));
            sink_[0]->setCallback(std::bind(&TraactDataflowTestNetwork::SendValid,
                                            this,
                                            0,
                                            0,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            sink_[0]->setInvalidCallback(std::bind(&TraactDataflowTestNetwork::SendInvalid,
                                                   this,
                                                   0,
                                                   0,
                                                   std::placeholders::_1));
            break;
        }
        case UseCaseProblem::In0In1_Out0: {
            DefaultPatternInstancePtr
                source_pattern =
                pattern_graph_ptr_->addPattern("source",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source2_pattern =
                pattern_graph_ptr_->addPattern("source2",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                sink_pattern =
                pattern_graph_ptr_->addPattern("sink",
                                               facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                mul_pattern =
                pattern_graph_ptr_->addPattern("mul", facade_->instantiatePattern("DataflowTestComponent"));

            pattern_graph_ptr_->connect("source", "output", "mul", "input0");
            pattern_graph_ptr_->connect("source2", "output", "mul", "input1");
            pattern_graph_ptr_->connect("mul", "output", "sink", "input");

            pattern_graph_ptr_->timedomain_configs[0] = td_config;

            facade_->loadDataflow(pattern_graph_ptr_);

            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source2")));

            DefaultComponentPtr sink = facade_->getComponent("sink");
            sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(
                sink));
            sink_[0]->setCallback(std::bind(&TraactDataflowTestNetwork::SendValid,
                                            this,
                                            0,
                                            0,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            sink_[0]->setInvalidCallback(std::bind(&TraactDataflowTestNetwork::SendInvalid,
                                                   this,
                                                   0,
                                                   0,
                                                   std::placeholders::_1));
            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul"));

            mul_pattern_tmp->base_problem_ = problem_list[0];
            break;
        }
        case UseCaseProblem::In0In1_In2_Out0: {
            DefaultPatternInstancePtr
                source_pattern =
                pattern_graph_ptr_->addPattern("source",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source2_pattern =
                pattern_graph_ptr_->addPattern("source2",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source3_pattern =
                pattern_graph_ptr_->addPattern("source3",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                sink_pattern =
                pattern_graph_ptr_->addPattern("sink",
                                               facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
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

            pattern_graph_ptr_->timedomain_configs[0] = td_config;

            facade_->loadDataflow(pattern_graph_ptr_);

            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source2")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source3")));

            DefaultComponentPtr sink = facade_->getComponent("sink");
            sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(
                sink));
            sink_[0]->setCallback(std::bind(&TraactDataflowTestNetwork::SendValid,
                                            this,
                                            0,
                                            0,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            sink_[0]->setInvalidCallback(std::bind(&TraactDataflowTestNetwork::SendInvalid,
                                                   this,
                                                   0,
                                                   0,
                                                   std::placeholders::_1));
            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul1"));

            mul_pattern_tmp->base_problem_ = problem_list[0];

            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern2_tmp =
                std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul2"));

            mul_pattern2_tmp->base_problem_ = problem_list[1];
            break;
        }

        case UseCaseProblem::In0In1_In2In3_Out0: {
            DefaultPatternInstancePtr
                source_pattern =
                pattern_graph_ptr_->addPattern("source",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source2_pattern =
                pattern_graph_ptr_->addPattern("source2",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source3_pattern =
                pattern_graph_ptr_->addPattern("source3",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source4_pattern =
                pattern_graph_ptr_->addPattern("source4",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                sink_pattern =
                pattern_graph_ptr_->addPattern("sink",
                                               facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
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

            pattern_graph_ptr_->timedomain_configs[0] = td_config;

            facade_->loadDataflow(pattern_graph_ptr_);

            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source2")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source3")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source4")));

            DefaultComponentPtr sink = facade_->getComponent("sink");
            sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(
                sink));
            sink_[0]->setCallback(std::bind(&TraactDataflowTestNetwork::SendValid,
                                            this,
                                            0,
                                            0,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            sink_[0]->setInvalidCallback(std::bind(&TraactDataflowTestNetwork::SendInvalid,
                                                   this,
                                                   0,
                                                   0,
                                                   std::placeholders::_1));
            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul1"));

            mul_pattern_tmp->base_problem_ = problem_list[0];

            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern2_tmp =
                std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul2"));

            mul_pattern2_tmp->base_problem_ = problem_list[1];

            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern3_tmp =
                std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul3"));

            mul_pattern3_tmp->base_problem_ = problem_list[2];
            break;
        }
        case UseCaseProblem::In0In1_In2In3_Out0_Out1: {
            DefaultPatternInstancePtr
                source_pattern =
                pattern_graph_ptr_->addPattern("source",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source2_pattern =
                pattern_graph_ptr_->addPattern("source2",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source3_pattern =
                pattern_graph_ptr_->addPattern("source3",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                source4_pattern =
                pattern_graph_ptr_->addPattern("source4",
                                               facade_->instantiatePattern("ApplicationAsyncSource_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                sink0_pattern =
                pattern_graph_ptr_->addPattern("sink0",
                                               facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
            DefaultPatternInstancePtr
                sink1_pattern =
                pattern_graph_ptr_->addPattern("sink1",
                                               facade_->instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));
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
            pattern_graph_ptr_->connect("mul1", "output", "sink0", "input");
            pattern_graph_ptr_->connect("source3", "output", "mul2", "input0");
            pattern_graph_ptr_->connect("source4", "output", "mul2", "input1");
            pattern_graph_ptr_->connect("mul1", "output", "mul3", "input0");
            pattern_graph_ptr_->connect("mul2", "output", "mul3", "input1");
            pattern_graph_ptr_->connect("mul3", "output", "sink1", "input");

            pattern_graph_ptr_->timedomain_configs[0] = td_config;

            facade_->loadDataflow(pattern_graph_ptr_);

            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source2")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source3")));
            sources_.emplace_back(std::dynamic_pointer_cast<SourceType>(facade_->getComponent("source4")));

            DefaultComponentPtr sink0 = facade_->getComponent("sink0");
            sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(
                sink0));
            sink_[0]->setCallback(std::bind(&TraactDataflowTestNetwork::SendValid,
                                            this,
                                            0,
                                            0,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            sink_[0]->setInvalidCallback(std::bind(&TraactDataflowTestNetwork::SendInvalid,
                                                   this,
                                                   0,
                                                   0,
                                                   std::placeholders::_1));

            DefaultComponentPtr sink1 = facade_->getComponent("sink1");
            sink_.emplace_back(std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(
                sink1));
            sink_[1]->setCallback(std::bind(&TraactDataflowTestNetwork::SendValid,
                                            this,
                                            0,
                                            1,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            sink_[1]->setInvalidCallback(std::bind(&TraactDataflowTestNetwork::SendInvalid,
                                                   this,
                                                   0,
                                                   1,
                                                   std::placeholders::_1));

            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern_tmp = std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul1"));

            mul_pattern_tmp->base_problem_ = problem_list[0];

            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern2_tmp =
                std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul2"));

            mul_pattern2_tmp->base_problem_ = problem_list[1];

            std::shared_ptr<test::DataflowTestComponent>
                mul_pattern3_tmp =
                std::dynamic_pointer_cast<test::DataflowTestComponent>(facade_->getComponent("mul3"));

            mul_pattern3_tmp->base_problem_ = problem_list[2];
            break;
        }
    }

}

void TraactDataflowTestNetwork::Start() {
    facade_->start();
}

void TraactDataflowTestNetwork::Stop() {
    facade_->stop();
}

void TraactDataflowTestNetwork::NewData(int time_domain, const dataflow_use_case_test::PortTsDataList &data) {
    for (const auto &event : data) {
        int port;
        dataflow_use_case_test::TimestampType ts;
        dataflow_use_case_test::TestDataType value;
        std::tie(port, ts, value) = event;
        if (time_domain == 0) {
            sources_[port]->newValue(ts, value);
        } else {
            sources_[port - 2]->newValue(ts, value);
        }
    }
}

void TraactDataflowTestNetwork::SendValid(int td, int port, dataflow_use_case_test::TimestampType ts,
                                          dataflow_use_case_test::TestDataType data) {
    result_callback_(td, std::make_tuple(port, ts, data));

}

void TraactDataflowTestNetwork::SendInvalid(int td, int port, dataflow_use_case_test::TimestampType ts) {
    invalid_callback_(td, port, ts);
}

}


