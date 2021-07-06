#include "gtest/gtest.h"
#include "TbbGraph.h"
#include "TbbParallelizing.h"

namespace HugoLiuGithub{

#if 0
class TestTbbGraph :public testing::Test
{
public:
    TbbGraph g_;

protected:
    virtual void SetUp()
    {
        LOGI("SetUp");
        g_.BuildDataFlowGraph();
        g_.BuildDependenceGraph();
    }
    virtual void TearDown()
    {
        LOGI("TearDown");
    }
};

TEST_F(TestTbbGraph, testcase1){
    LOGI("TestTbbGraph testcase1");
    g_.RunDataFlowGraph();
}

TEST_F(TestTbbGraph, testcase2){
    LOGI("TestTbbGraph testcase2");
    g_.RunDependenceGraph();
}

#endif

class TestTbbParallelizing : public testing::Test
{
public:
    TbbParallelizing p_;
protected:
    virtual void SetUp()
    {
        LOGI("SetUp");
    }
    virtual void TearDown()
    {
        LOGI("TearDown");
    }
};

TEST_F(TestTbbParallelizing, testcase1){
    p_.RunSerial_2(64);
}

TEST_F(TestTbbParallelizing, testcase2){
    p_.RunParallel_2(64);
}

}

int main(int argc, char** argv) {
    for(int i = 0; i < argc; i++){
        fprintf(stderr, "argv[%d] = %s\n", i, argv[i]);
    }
    google::InitGoogleLogging(argv[0]);
    FLAGS_minloglevel = 0;
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}