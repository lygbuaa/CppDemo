#include "gtest/gtest.h"
#include <eigen3/Eigen/Dense>
#include "utils/DepthUtils.h"

namespace HugoLiuGithub{

class TestTbb :public testing::Test
{
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

TEST_F(TestTbb, testcase1){
    LOGI("TestTbb testcase1");
    HANG_STOPWATCH();
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