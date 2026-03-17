#ifndef STATISTICS_MAXDEVIATIONFINDER_H_
#define STATISTICS_MAXDEVIATIONFINDER_H_

#include <cmath>

#include "WelfordAlgorithm.h"

class MaxDeviationFinder {
 public:
  explicit MaxDeviationFinder(unsigned int stop_time);
  ~MaxDeviationFinder() = default;

  bool AddData(double raw_input, unsigned int timestamp);
  bool CheckTimeout(unsigned int timestamp);
  void MaxDeviationTimerStarter(unsigned int start_time);
  double GetMaxStdDeviation() const;
  double TruncateToFirstDigit(double value) const;
  double GetMaxTruncatedStdDeviation() const;
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

#endif  // STATISTICS_MAXDEVIATIONFINDER_H_