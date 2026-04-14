#ifndef SIMULATION_SIMULATION_VARS_TXT_H_
#define SIMULATION_SIMULATION_VARS_TXT_H_

#include <fstream>
#include <stdexcept>
#include <string>

// CSV: 1.ª linha cabeçalho; restantes só valores separados por vírgula.
// writeEverySamples: grava 1 em cada N chamadas a append (1 = todas).
class SimulationVarsTxt {
 public:
  explicit SimulationVarsTxt(const std::string& path, unsigned writeEverySamples = 100)
      : _out(path, std::ios::out | std::ios::trunc), _every(writeEverySamples) {
    if (!_out) {
      throw std::runtime_error("SimulationVarsTxt: nao foi possivel abrir " + path);
    }
    if (_every == 0) {
      throw std::runtime_error("SimulationVarsTxt: writeEverySamples deve ser >= 1");
    }
    _out << "time,step,speed\n";
    _out.flush();
  }

  void append(double time, double step, double speed) {
    if (_every > 1 && (_sampleIndex % _every) != 0) {
      ++_sampleIndex;
      return;
    }
    _out << std::to_string(time) << ',' << std::to_string(step) << ',' << std::to_string(speed)
         << '\n';
    _out.flush();
    ++_sampleIndex;
  }

 private:
  std::ofstream _out;
  unsigned _every;
  unsigned _sampleIndex{0};
};

#endif
