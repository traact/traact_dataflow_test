/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/
#include <benchmark/benchmark.h>
#include <taskflow/taskflow.hpp>

class TaskFlowAsyncSources : public benchmark::Fixture {
 public:
    void SetUp(const ::benchmark::State &state) {
        promises_.resize(4);
        futures_.resize(4);

        taskflow_.name("(in0*in1)*(in2*in3)=out0");

        auto in0 = taskflow_.emplace([&, index = 0]() {
            futures_[index].wait();
            return 0;
        }).name("in0");
        auto in1 = taskflow_.emplace([&, index = 1]() {
            futures_[index].wait();
            return 0;
        }).name("in1");
        auto in2 = taskflow_.emplace([&, index = 2]() {
            futures_[index].wait();
            return 0;
        }).name("in2");
        auto in3 = taskflow_.emplace([&, index = 3]() {
            futures_[index].wait();
            return 0;
        }).name("in3");
        auto mul0 = taskflow_.emplace([&]() {

            return 0;
        }).name("mul0");
        auto mul1 = taskflow_.emplace([&]() {

            return 0;
        }).name("mul1");
        auto mul2 = taskflow_.emplace([&]() {

            return 0;
        }).name("mul2");
        auto out0 = taskflow_.emplace([&]() {

            return 0;
        }).name("out0");

        in0.precede(mul0);
        in1.precede(mul0);
        in2.precede(mul1);
        in3.precede(mul1);
        mul0.precede(mul2);
        mul1.precede(mul2);
        mul2.precede(out0);

    }

    void TearDown(const ::benchmark::State &state) {

    }

    tf::Executor executor_{4};
    tf::Taskflow taskflow_;
    std::vector<std::promise<bool>> promises_;
    std::vector<std::future<bool>> futures_;
};

BENCHMARK_DEFINE_F(TaskFlowAsyncSources, All)(benchmark::State &state) {
    while (state.KeepRunning()) {
        for (int i = 0; i < 4; ++i) {
            promises_[i] = {};
            futures_[i] = promises_[i].get_future();
        }

        executor_.run(taskflow_);

        for (int i = 0; i < 4; ++i) {
            promises_[i].set_value(true);
        }

        executor_.wait_for_all();

    }
}

BENCHMARK_REGISTER_F(TaskFlowAsyncSources, All)->MeasureProcessCPUTime();