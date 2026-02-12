#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

#include "statistics/WelfordAlgorithm.h"

class WelfordAlgorithmTestHelper : public WelfordAlgorithm {
 public:
    explicit WelfordAlgorithmTestHelper(int windowSize) : WelfordAlgorithm(windowSize) {
    }

    double *GetWindow() const {
        return window_;
    }
};

TEST(WelfordAlgorithmTest, ValidateInputs) {
    WelfordAlgorithmTestHelper welford(10);

    welford.AddSample(10);
    welford.AddSample(11);
    welford.AddSample(12);
    welford.AddSample(13);
    welford.AddSample(14);
    welford.AddSample(15);
    welford.AddSample(16);
    welford.AddSample(17);
    welford.AddSample(18);
    welford.AddSample(19);

    EXPECT_EQ(welford.GetWindow()[0], 10);
    EXPECT_EQ(welford.GetWindow()[1], 11);
    EXPECT_EQ(welford.GetWindow()[2], 12);
    EXPECT_EQ(welford.GetWindow()[3], 13);
    EXPECT_EQ(welford.GetWindow()[4], 14);
    EXPECT_EQ(welford.GetWindow()[5], 15);
    EXPECT_EQ(welford.GetWindow()[6], 16);
    EXPECT_EQ(welford.GetWindow()[7], 17);
    EXPECT_EQ(welford.GetWindow()[8], 18);
    EXPECT_EQ(welford.GetWindow()[9], 19);

    welford.AddSample(20);
    welford.AddSample(21);
    welford.AddSample(22);
    welford.AddSample(23);
    welford.AddSample(24);

    EXPECT_EQ(welford.GetWindow()[0], 20);
    EXPECT_EQ(welford.GetWindow()[1], 21);
    EXPECT_EQ(welford.GetWindow()[2], 22);
    EXPECT_EQ(welford.GetWindow()[3], 23);
    EXPECT_EQ(welford.GetWindow()[4], 24);
    EXPECT_EQ(welford.GetWindow()[5], 15);
    EXPECT_EQ(welford.GetWindow()[6], 16);
    EXPECT_EQ(welford.GetWindow()[7], 17);
    EXPECT_EQ(welford.GetWindow()[8], 18);
    EXPECT_EQ(welford.GetWindow()[9], 19);

    welford.AddSample(25);
    welford.AddSample(26);
    welford.AddSample(27);
    welford.AddSample(28);
    welford.AddSample(29);

    EXPECT_EQ(welford.GetWindow()[0], 20);
    EXPECT_EQ(welford.GetWindow()[1], 21);
    EXPECT_EQ(welford.GetWindow()[2], 22);
    EXPECT_EQ(welford.GetWindow()[3], 23);
    EXPECT_EQ(welford.GetWindow()[4], 24);
    EXPECT_EQ(welford.GetWindow()[5], 25);
    EXPECT_EQ(welford.GetWindow()[6], 26);
    EXPECT_EQ(welford.GetWindow()[7], 27);
    EXPECT_EQ(welford.GetWindow()[8], 28);
    EXPECT_EQ(welford.GetWindow()[9], 29);
}

TEST(WelfordAlgorithmTest, ValidateInputsWithDifferentWindowSize) {
    WelfordAlgorithmTestHelper welford(5);

    welford.AddSample(10);
    welford.AddSample(11);
    welford.AddSample(12);
    welford.AddSample(13);
    welford.AddSample(14);
    welford.AddSample(15);
    welford.AddSample(16);
    welford.AddSample(17);
    welford.AddSample(18);
    welford.AddSample(19);

    EXPECT_EQ(welford.GetWindow()[0], 15);
    EXPECT_EQ(welford.GetWindow()[1], 16);
    EXPECT_EQ(welford.GetWindow()[2], 17);
    EXPECT_EQ(welford.GetWindow()[3], 18);
    EXPECT_EQ(welford.GetWindow()[4], 19);
}

TEST(WelfordAlgorithmTest, ValidateResultWindowSize10) {
    WelfordAlgorithmTestHelper welford(10);

    welford.AddSample(10);
    welford.AddSample(11);
    welford.AddSample(12);
    welford.AddSample(13);
    welford.AddSample(14);
    welford.AddSample(15);
    welford.AddSample(16);
    welford.AddSample(17);
    welford.AddSample(18);
    welford.AddSample(19);

    EXPECT_NEAR(welford.GetMean(), 14.5, 0.1);
    EXPECT_NEAR(welford.GetVariance(), 9.1, 0.1);
    EXPECT_NEAR(welford.GetDeviation(), std::sqrt(9.1), 0.1);
}

TEST(WelfordAlgorithmTest, ValidateResultWindowSize5) {
    WelfordAlgorithmTestHelper welford(5);

    welford.AddSample(10);
    welford.AddSample(11);
    welford.AddSample(12);
    welford.AddSample(13);
    welford.AddSample(14);
    welford.AddSample(15);
    welford.AddSample(16);
    welford.AddSample(17);
    welford.AddSample(18);
    welford.AddSample(19);

    EXPECT_NEAR(welford.GetMean(), 17, 0.1);
    EXPECT_NEAR(welford.GetVariance(), 2.5, 0.1);
    EXPECT_NEAR(welford.GetDeviation(), std::sqrt(2.5), 0.1);
}