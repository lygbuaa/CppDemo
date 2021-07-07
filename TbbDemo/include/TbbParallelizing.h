#ifndef TBB_PARALLELIZING_H
#define TBB_PARALLELIZING_H

#include <eigen3/Eigen/Dense>
#include <atomic>
#include "tbb/tbb.h"
#include "utils/DepthUtils.h"

namespace HugoLiuGithub
{
class TbbParallelizing
{
public:
    TbbParallelizing() noexcept {}
    ~TbbParallelizing(){}

    static double compute(const int N){
        Eigen::MatrixXf m1 = Eigen::MatrixXf::Random(N, N);
        Eigen::MatrixXf m2 = Eigen::MatrixXf::Random(N, N);
        Eigen::MatrixXf m3 = m1*m2;
        return m3.sum();
    }

    /*
    * parallel_for() only needs operator()
    */
    class ParallelWrapper_1
    {
        public:
            static std::atomic<size_t> counter_;
            ParallelWrapper_1(){
                LOGI("create thread [%ld]", (size_t)counter_);
                ++ counter_;
            }

            void operator()(const tbb::blocked_range<size_t>& r) const {
                for(size_t i=r.begin(); i!=r.end(); ++i){
                    compute(256);
                }
            }
    };

    void RunSerial_1(size_t n){
        for(size_t i = 0; i < n; ++i){
            compute(256);
        }
    }

    /*
    * affinity_partitioner can improve a little performance
    */
    void RunParallel_1(size_t n){
        #define USE_AFFINITY_PARTITIONER 0

        ParallelWrapper_1 pw;
        #if USE_AFFINITY_PARTITIONER
            static tbb::affinity_partitioner ap;
            tbb::parallel_for(tbb::blocked_range<size_t>(0, n), pw, ap);
        #else
            tbb::parallel_for(tbb::blocked_range<size_t>(0, n), pw);
        #endif
    }

    /*
    * parallel_reduce() needs split-constructor, join(), and operator()
    */
    class ParallelWrapper_2
    {
        public:
            static std::atomic<size_t> counter_;
            double min_ {std::numeric_limits<double>::max()};
            size_t index_ {0};
            size_t id_;

            ParallelWrapper_2() {
                LOGI("create father [%ld]", (size_t)counter_);
            }

            ParallelWrapper_2(ParallelWrapper_2& other, tbb::split) : min_(std::numeric_limits<double>::max()), index_(0) {
                id_ = ++counter_;
                LOGI("create derived [%ld]", id_);
            }

            void join(const ParallelWrapper_2& other){
                LOGI("[%ld] join [%ld]", other.id_, this->id_);
                if(other.min_ < this->min_){
                    this->min_ = other.min_;
                    this->index_ = other.index_;
                }
            }

            void operator()(const tbb::blocked_range<size_t>& r) {
                for(size_t i = r.begin(); i!=r.end(); ++i){
                    double val = compute(256);
                    if(val < min_){
                        min_ = val;
                        index_ = i;
                    }
                }
            }
    };

    double RunSerial_2(size_t n){
        double min = std::numeric_limits<double>::max();
        size_t index = 0;
        for(size_t i = 0; i < n; ++i){
            double val = compute(256);
            if(val < min){
                min = val;
                index = i;
            }
        }
        return min;
    }

    double RunParallel_2(size_t n){
        ParallelWrapper_2 pw;
        tbb::parallel_reduce(tbb::blocked_range<size_t>(0, n), pw);
        return pw.min_;
    }

    /*
    * for input, copy constructor must be defined, tbb::flow_control::stop() must be called
    */
    class PpInputFunc
    {
        public:
            static std::atomic<size_t> counter_;
            size_t id_;
            size_t total_;

            PpInputFunc(size_t n) : total_(n) {
                id_ = 0;
                LOGI("create input [%ld]", (size_t)counter_);
            }
            PpInputFunc(const PpInputFunc& other){
                id_ = ++counter_;
                this->total_ = other.total_;
                LOGI("input copied [%ld]", id_);
            }
            ~PpInputFunc(){}

            double operator()(tbb::flow_control& fc) const {
                static int total = 0;
                if(total < total_){
                    double val = compute(64);
                    LOGI("input [%ld] start", total++);
                    return val;
                }else{
                    fc.stop();
                    LOGI("input [%ld] exit", total);
                    return 0.0f;
                }
            }
    };

    /*
    * for mid, only need operator()
    */
    class PpMidFunc
    {
        public:
            static std::atomic<size_t> counter_;
            size_t id_;

            PpMidFunc(){
                id_ = ++counter_;
                LOGI("create mid [%ld]", (size_t)counter_);
            }

            double operator()(double tmp) const {
                static int total = 0;
                double val = compute(512);
                LOGI("mid [%ld] pass", total++);
                return val;
            }
    };

    /*
    * for output, only need operator()
    */
    class PpOutputFunc
    {
        public:
            static std::atomic<size_t> counter_;
            size_t id_;

            PpOutputFunc(){
                id_ = ++counter_;
                LOGI("create output [%ld]", id_);
            }

            void operator()(double tmp) const {
                static int total = 0;
                compute(64);
                LOGI("output [%ld] done", total++);
            }
    };

    void RunSerial_3(size_t n){
        for(int i = 0; i < n; ++i){
            compute(64);
            compute(512);
            compute(64);
        }
    }

    void RunPipeline(size_t n){
        const int ntoken = 8;
        tbb::filter_t<void, double> f1(tbb::filter::serial_in_order, PpInputFunc(n));
        tbb::filter_t<double, double> f2(tbb::filter::parallel, PpMidFunc());
        tbb::filter_t<double, void> f3(tbb::filter::serial_in_order, PpOutputFunc());

        tbb::filter_t<void, void> f = f1 & f2 & f3;
        tbb::parallel_pipeline(ntoken, f);
    }

};
}

#endif