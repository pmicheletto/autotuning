#ifndef STATISTICS_WELFORDALGORITHM_H_
#define STATISTICS_WELFORDALGORITHM_H_

class WelfordAlgorithm {
 public:
  explicit WelfordAlgorithm(unsigned int window_size = 10);
  WelfordAlgorithm& operator=(const WelfordAlgorithm&) = delete;
  WelfordAlgorithm(const WelfordAlgorithm&) = delete;
  ~WelfordAlgorithm();

  void AddSample(double new_value);
  void Clear();
  double GetMean();
  double GetVariance();
  double GetDeviation();

 protected:
  double* window_;

 private:
  unsigned int window_size_;
  unsigned int count_;
  double mean_;
  double m2_;  // squared distance from the mean
  double sum_;
  double sum_squares_;
  double oldest_value_;
  unsigned int window_index_;
};

#endif  // STATISTICS_WELFORDALGORITHM_H_