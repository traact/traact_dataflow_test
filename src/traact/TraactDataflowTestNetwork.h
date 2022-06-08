/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_dataflow_use_case_test_SRC_TRAACT_TRAACTPROBLEMSOLVER_H_
#define TRAACTMULTI_TRAACT_dataflow_use_case_test_SRC_TRAACT_TRAACTPROBLEMSOLVER_H_

#include <dataflow_use_case_test/DataflowNetwork.h>
#include <traact/traact.h>
#include <traact/facade/Facade.h>
#include <traact/component/facade/ApplicationSyncSink.h>
#include <traact/component/facade/ApplicationAsyncSource.h>
#include <traact/spatial.h>

namespace traact::test {

class TraactDataflowTestNetwork : public dataflow_use_case_test::DataflowNetwork {
 public:
    typedef typename traact::component::facade::ApplicationAsyncSource<spatial::Pose6DHeader> SourceType;
    typedef typename std::shared_ptr<component::facade::ApplicationSyncSink<spatial::Pose6DHeader> > SinkPtr;
    typedef typename std::shared_ptr<component::facade::ApplicationAsyncSource<spatial::Pose6DHeader> > SourcePtr;

    void Init(dataflow_use_case_test::UseCaseProblem problem,
              const dataflow_use_case_test::BaseProblemList &problem_list,
              const dataflow_use_case_test::ResultCallback &result_callback,
              const dataflow_use_case_test::InvalidCallback &invalid_callback,
              dataflow_use_case_test::TimeDurationType max_ts_offset) override;

    void Start() override;

    void Stop() override;

    void NewData(int time_domain, const dataflow_use_case_test::PortTsDataList &data) override;

 private:
    DefaultInstanceGraphPtr pattern_graph_ptr_;

    std::shared_ptr<facade::Facade> facade_;

    std::vector<SinkPtr> sink_;
    std::vector<SourcePtr> sources_;

    dataflow_use_case_test::ResultCallback result_callback_;
    dataflow_use_case_test::InvalidCallback invalid_callback_;

    void SendValid(int td,
                   int port,
                   dataflow_use_case_test::TimestampType ts,
                   dataflow_use_case_test::TestDataType data);
    void SendInvalid(int td, int port, dataflow_use_case_test::TimestampType ts);

};
}

#endif //TRAACTMULTI_TRAACT_dataflow_use_case_test_SRC_TRAACT_TRAACTPROBLEMSOLVER_H_
