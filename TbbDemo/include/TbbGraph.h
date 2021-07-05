#ifndef TBB_GRAPH_H
#define TBB_GRAPH_H

#include <eigen3/Eigen/Dense>
#include "tbb/tbb.h"
#include "utils/DepthUtils.h"

namespace HugoLiuGithub
{
class TbbGraph
{
public:
    typedef tbb::flow::function_node<double, double> FuncNode_t;
    typedef tbb::flow::continue_msg msg_t;
    typedef tbb::flow::continue_node<msg_t> ContNode_t;

private:
    tbb::flow::graph graph_;

    /*  
    * dataflow-graph using function_node
    */
    std::unique_ptr<FuncNode_t> df_node11_;
    std::unique_ptr<FuncNode_t> df_node21_;
    std::unique_ptr<FuncNode_t> df_node22_;
    std::unique_ptr<FuncNode_t> df_node31_;

    /*
    * dependence-graph using continue_node
    */
   std::unique_ptr<ContNode_t> dp_node11_;
   std::unique_ptr<ContNode_t> dp_node21_;
   std::unique_ptr<ContNode_t> dp_node22_;
   std::unique_ptr<ContNode_t> dp_node31_;

public:
    TbbGraph() noexcept {;}
    virtual ~TbbGraph() {;}

    TbbGraph(const TbbGraph& other) = delete;
    TbbGraph(TbbGraph&& other) = delete;
    TbbGraph& operator=(TbbGraph const& other) = delete;
    TbbGraph& operator=(TbbGraph&& other) = delete;

    static void spin_for( const double sec ) {
        tbb::tick_count start = tbb::tick_count::now();
        while( (tbb::tick_count::now() - start).seconds() < sec ) ;
    }

    static int compute(const int N){
        Eigen::MatrixXf m1 = Eigen::MatrixXf::Random(N, N);
        Eigen::MatrixXf m2 = Eigen::MatrixXf::Random(N, N);
        Eigen::MatrixXf m3 = m1*m2;
        return N;
    }

    /*
    * build dataflow-graph: node11 --> (node21 || node22) --> node31
    */
    void BuildDataFlowGraph(){
        HANG_STOPWATCH();
        /* input node */
        df_node11_ = std::unique_ptr<FuncNode_t> (new FuncNode_t(graph_, tbb::flow::unlimited, \
            [&](double input) -> double {
                LOGI("node11 sleep: %f", input);
                spin_for(input);
                return input;
            }
        ));

        /* mid node1, could be parallel */
        df_node21_ = std::unique_ptr<FuncNode_t> (new FuncNode_t(graph_, tbb::flow::unlimited, \
            [&](double input) -> double {
                LOGI("node21 sleep: %f", input);
                spin_for(input);
                return input;
            }
        ));

        /* mid node2, sleep longer, could be parallel */
        df_node22_ = std::unique_ptr<FuncNode_t> (new FuncNode_t(graph_, tbb::flow::unlimited, \
            [&](double input) -> double {
                LOGI("node22 sleep: %f", input*2);
                spin_for(input*2);
                return input;
            }
        ));

        /* set final node to serial */
        df_node31_ = std::unique_ptr<FuncNode_t> (new FuncNode_t(graph_, 1, \
            [&](double input) -> double {
                LOGI("node31 sleep: %f", input);
                spin_for(input);
                return input;
            }
        ));

        tbb::flow::make_edge(*df_node11_, *df_node21_);
        tbb::flow::make_edge(*df_node11_, *df_node22_);
        tbb::flow::make_edge(*df_node21_, *df_node31_);
        tbb::flow::make_edge(*df_node22_, *df_node31_);
    }

    void RunDataFlowGraph(){
        df_node11_ -> try_put(1.0f);
        graph_.wait_for_all();
    }

    /*
    * build dependence-graph: node11 --> (node21 || node22) --> node31
    * continue_node is always assumed to have unlimited concurrency and will immediately spawn a task whenever its dependencies are met.
    */
    void BuildDependenceGraph(){
        HANG_STOPWATCH();
        /* input node */
        dp_node11_ = std::unique_ptr<ContNode_t> (new ContNode_t(graph_, \
            [&](const msg_t& input){
                const double sec = 1.0f;
                LOGI("node11 sleep: %f", sec);
                spin_for(sec);
            }
        ));

        /* mid node1, could be parallel */
        dp_node21_ = std::unique_ptr<ContNode_t> (new ContNode_t(graph_, \
            [&](const msg_t& input){
                const double sec = 1.0f;
                LOGI("node21 sleep: %f", sec);
                spin_for(sec);
            }
        ));

        /* mid node2, sleep longer, could be parallel */
        dp_node22_ = std::unique_ptr<ContNode_t> (new ContNode_t(graph_, \
            [&](const msg_t& input){
                const double sec = 2.0f;
                LOGI("node22 sleep: %f", sec);
                spin_for(sec);
            }
        ));

        /* set final node to serial */
        dp_node31_ = std::unique_ptr<ContNode_t> (new ContNode_t(graph_, \
            [&](const msg_t& input){
                const double sec = 1.0f;
                LOGI("node31 sleep: %f", sec);
                spin_for(sec);
            }
        ));

        tbb::flow::make_edge(*dp_node11_, *dp_node21_);
        tbb::flow::make_edge(*dp_node11_, *dp_node22_);
        tbb::flow::make_edge(*dp_node21_, *dp_node31_);
        tbb::flow::make_edge(*dp_node22_, *dp_node31_);
    }

    void RunDependenceGraph(){
        msg_t msg;
        dp_node11_ -> try_put(msg);
        graph_.wait_for_all();
    }


};
}
#endif