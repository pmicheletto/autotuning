#ifndef TUNING_SYSTEM_SYSTEM_INTERFACE_H_
#define TUNING_SYSTEM_SYSTEM_INTERFACE_H_

#include <memory>
#include <string>

class ISystem {
 public:
  virtual ~ISystem() = default;

  // Avança a simulação (ou amostragem em hardware) no tempo por \p dt segundos.
  // O degrau de alimentação é definido via setVariable("step.height", value).
  virtual void advanceSimulation(double dt) = 0;

  // Lê um valor do sistema
  virtual double readValue(const std::string& varName) = 0;

  // Escreve um valor no sistema (ex.: setVariable("step.height", value) para o degrau de alimentação)
  virtual void setVariable(const std::string& varName, double value) = 0;
};

// Factory simples - retorna simulação por padrão
std::unique_ptr<ISystem> createSystem(bool useSimulation = true,
                                      const std::string& fmuPath = "");

#endif
