
#include <gtest/gtest.h>


class TestGazeboROSMotorPlugin : public ::testing::Test
{
};


TEST(TestGazeboROSMotorPlugin, Pass)
{
  int * test_ptr = nullptr;
  ASSERT_EQ(nullptr, test_ptr);
}
