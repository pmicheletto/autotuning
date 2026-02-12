#ifndef SRC_MAXDEVIATIONFINDER_H_
#define SRC_MAXDEVIATIONFINDER_H_

#include <cmath>

#include "WelfordAlgorithm.h"

class MaxDeviationFinder {
   public:
    explicit MaxDeviationFinder(unsigned int stop_time);
    ~MaxDeviationFinder();

    bool AddData(double raw_input, unsigned int timestamp);

    bool CheckTimeout(unsigned int timestamp);
    void MaxDeviationTimerStarter(unsigned int start_time);
    double GetMaxStdDeviation();
    double TruncateToFirstDigit(double value);
    double GetMaxTruncatedStdDeviation();
    void Clear();

   private:
    unsigned int stop_time_;
    double mean_value_;
    double std_deviation_;
    double max_std_deviation_;
    WelfordAlgorithm welford_filter_;
    WelfordAlgorithm welford_deviation_;
    unsigned int start_time_;
};

#endif  // SRC_MAXDEVIATIONFINDER_H_