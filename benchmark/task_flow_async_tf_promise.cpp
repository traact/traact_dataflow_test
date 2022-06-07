/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/
#include <benchmark/benchmark.h>
#include <taskflow/taskflow.hpp>

class TaskFlowAsyncTfPromise : public benchmark::Fixture {
 public:
    void SetUp(const ::benchmark::State &state) {

        taskflow_.name("(in0*in1)*(in2*in3)=out0");

        auto in0 = taskflow_.emplace([&, index = 0]() {
            return 0;
        }).name("in0");
        auto in1 = taskflow_.emplace([&, index = 1]() {
            return 0;
        }).name("in1");
        auto in2 = taskflow_.emplace([&, index = 2]() {
            return 0;
        }).name("in2");
        auto in3 = taskflow_.emplace([&, index = 3]() {
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

        auto in0task = taskflow_in0_.emplace([]() {
            return 0;
        });
        auto in1task = taskflow_in1_.emplace([]() {
            return 0;
        });
        auto in2task = taskflow_in2_.emplace([]() {
            return 0;
        });
        auto in3task = taskflow_in3_.emplace([]() {
            return 0;
        });

        in0.acquire(in0_semaphore_);
        in1.acquire(in1_semaphore_);
        in2.acquire(in2_semaphore_);
        in3.acquire(in3_semaphore_);

        in0task.release(in0_semaphore_);
        in1task.release(in1_semaphore_);
        in2task.release(in2_semaphore_);
        in3task.release(in3_semaphore_);

    }

    void TearDown(const ::benchmark::State &state) {

    }

    tf::Executor executor_{4};
    tf::Taskflow taskflow_;
    tf::Taskflow taskflow_in0_;
    tf::Taskflow taskflow_in1_;
    tf::Taskflow taskflow_in2_;
    tf::Taskflow taskflow_in3_;
    tf::Semaphore in0_semaphore_{0};
    tf::Semaphore in1_semaphore_{0};
    tf::Semaphore in2_semaphore_{0};
    tf::Semaphore in3_semaphore_{0};
};

BENCHMARK_DEFINE_F(TaskFlowAsyncTfPromise, All)(benchmark::State &state) {
    while (state.KeepRunning()) {

        executor_.run(taskflow_);
        executor_.run(taskflow_in0_);
        executor_.run(taskflow_in1_);
        executor_.run(taskflow_in2_);
        executor_.run(taskflow_in3_);

        executor_.wait_for_all();

    }
}

BENCHMARK_REGISTER_F(TaskFlowAsyncTfPromise, All)->MeasureProcessCPUTime();