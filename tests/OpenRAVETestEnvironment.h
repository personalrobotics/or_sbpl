#include <gtest/gtest.h>
#include <openrave-core.h>

class OpenRAVETestEnvironment : public testing::Environment {
public:
    virtual void SetUp()
    {
        OpenRAVE::RaveInitialize(true);
    }

    virtual void TearDown()
    {
        OpenRAVE::RaveDestroy();
    }
};
