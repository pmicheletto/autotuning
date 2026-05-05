#ifndef TUNING_SYSTEM_SYSTEM_INTERFACE_H_
#define TUNING_SYSTEM_SYSTEM_INTERFACE_H_

#include <memory>
#include <string>

class ISystem {
 public:
  virtual ~ISystem() = default;

  virtual void reset() = 0;

  // Advances simulation or hardware sampling by `dt` seconds.
  virtual void advanceSimulation(double dt) = 0;

  virtual double readValue(const std::string& varName) = 0;

  virtual void setVariable(const std::string& varName, double value) = 0;
};

std::unique_ptr<ISystem> createSystem(bool useSimulation = true,
                                      const std::string& fmuPath = "");

#endif  // TUNING_SYSTEM_SYSTEM_INTERFACE_H_
