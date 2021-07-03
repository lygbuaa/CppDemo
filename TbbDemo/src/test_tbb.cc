#include "gtest/gtest.h"
#include <glog/logging.h>

class TestTbb :public testing::Test
{
protected:
    virtual void SetUp()
    {
        LOG(INFO) << "SetUp";
    }
    virtual void TearDown()
    {
        LOG(INFO) << "TearDown";
    }
};

TEST_F(TestTbb, testcase1){
    LOG(INFO) << "TestTbb testcase1";
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