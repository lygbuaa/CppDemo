#ifndef TBB_PARALLELIZING_H
#define TBB_PARALLELIZING_H

#include <eigen3/Eigen/Dense>
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

        #if USE_AFFINITY_PARTITIONER
            static tbb::affinity_partitioner ap;
            tbb::parallel_for(tbb::blocked_range<size_t>(0, n), ParallelWrapper_1(), ap);
        #else
            tbb::parallel_for(tbb::blocked_range<size_t>(0, n), ParallelWrapper_1());
        #endif
    }

    /*
    * parallel_reduce() needs split-constructor, join(), and operator()
    */
    class ParallelWrapper_2
    {
        public:
            double min_ {std::numeric_limits<double>::max()};
            size_t index_ {0};
            static size_t counter_;
            size_t id_;

            ParallelWrapper_2() {
                LOGI("create father [%ld]", counter_);
            }

            ParallelWrapper_2(ParallelWrapper_2& other, tbb::split) : min_(std::numeric_limits<double>::max()), index_(0) {
                id_ = ++counter_;
                LOGI("create derive [%ld]", id_);
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

};
}

#endif