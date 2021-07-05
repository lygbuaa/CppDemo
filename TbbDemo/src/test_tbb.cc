#include "gtest/gtest.h"
#include "TbbGraph.h"

namespace HugoLiuGithub{

class TestTbb :public testing::Test
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

TEST_F(TestTbb, testcase1){
    LOGI("TestTbb testcase1");
    g_.RunDataFlowGraph();
}

TEST_F(TestTbb, testcase2){
    LOGI("TestTbb testcase2");
    g_.RunDependenceGraph();
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