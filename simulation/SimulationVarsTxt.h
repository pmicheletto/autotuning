#ifndef SIMULATION_SIMULATION_VARS_TXT_H_
#define SIMULATION_SIMULATION_VARS_TXT_H_

#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

// CSV: header on first line; following lines are comma-separated values only.
// writeEverySamples: append one row every N calls (1 = every call).
class SimulationVarsTxt {
 public:
  explicit SimulationVarsTxt(const std::string& path, unsigned writeEverySamples = 100)
      : _out(path, std::ios::out | std::ios::trunc), _every(writeEverySamples) {
    if (!_out) {
      throw std::runtime_error("SimulationVarsTxt: could not open " + path);
    }
    if (_every == 0) {
      throw std::runtime_error("SimulationVarsTxt: writeEverySamples must be >= 1");
    }
    _out << "time,step,speed,jerk,speed_filt,jerk_max_current_gain_test,test_phase\n";
    _out.flush();
  }

  void append(double time, double step, double speed, std::optional<double> jerk,
              double speed_filtered, double jerk_max_current_gain_test,
              const std::string& test_phase) {
    if (_every > 1 && (_sampleIndex % _every) != 0) {
      ++_sampleIndex;
      return;
    }
    _out << std::to_string(time) << ',' << std::to_string(step) << ','
         << std::to_string(speed) << ',';
    if (jerk.has_value()) {
      _out << std::to_string(*jerk);
    } else {
      _out << "nan";
    }
    _out << ',' << std::to_string(speed_filtered) << ','
         << std::to_string(jerk_max_current_gain_test) << ',' << test_phase << '\n';
    _out.flush();
    ++_sampleIndex;
  }

 private:
  std::ofstream _out;
  unsigned _every;
  unsigned _sampleIndex{0};
};

#endif
