/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_DATAFLOW_TEST_SRC_TRAACT_TRAACTDATAFLOWTESTPLUGIN_H_
#define TRAACT_DATAFLOW_TEST_SRC_TRAACT_TRAACTDATAFLOWTESTPLUGIN_H_

#include <traact/spatial.h>
#include <traact/pattern/Pattern.h>

#include <dataflow_use_case_test/TestUtils.h>
#include <traact/traact.h>

namespace traact::test {

class DataflowTestComponent : public component::Component {
 public:
    explicit DataflowTestComponent(const std::string &name) : Component(name) {}

    static pattern::Pattern::Ptr GetPattern(){
        using namespace traact::spatial;
        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("DataflowTestComponent", traact::Concurrency::UNLIMITED,component::ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort("input0", Pose6DHeader::NativeTypeName)
            .addConsumerPort("input1", Pose6DHeader::NativeTypeName)
            .addProducerPort("output", Pose6DHeader::NativeTypeName);

        pattern->addCoordinateSystem("A", false)
            .addCoordinateSystem("B", false)
            .addCoordinateSystem("C", false)
            .addEdge("A", "B", "input0")
            .addEdge("B", "C", "input1")
            .addEdge("A", "C", "output");

        return
            pattern;
    };

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto timestamp = data.getTimestamp();
        const auto &input_0 = data.getInput<traact::spatial::Pose6DHeader>(0);
        const auto &input_1 = data.getInput<traact::spatial::Pose6DHeader>(1);
        auto &output = data.getOutput<traact::spatial::Pose6DHeader>(0);

        output = base_problem_->Execute(timestamp, input_0, input_1);

        return true;
    }

    std::shared_ptr<dataflow_use_case_test::BaseProblem> base_problem_;
};

}
#endif //TRAACT_DATAFLOW_TEST_SRC_TRAACT_TRAACTDATAFLOWTESTPLUGIN_H_
