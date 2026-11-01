# Autotuning — UML (Mermaid)

Abre este ficheiro no Cursor/VS Code (pré-visualização Markdown com suporte Mermaid) ou cola cada bloco em [mermaid.live](https://mermaid.live) → **Actions** → exportar PNG/SVG.

## Diagrama de classes

```mermaid
classDiagram
  direction TB

  class ISystem {
    <<interface>>
    +reset()
    +advanceSimulation(dt)
    +readValue(varName)
    +setVariable(varName, value)
  }

  class SimulationSystem {
    <<.cpp only>>
    -fmuPath
    -simulationTime
  }

  class HardwareSystem {
    <<stub .cpp>>
  }

  class ModelicaWrapper {
    -fmu unique_ptr
    -slave unique_ptr
    +init(startTime)
    +stepFMU(stepSize)
    +getReal(varName)
    +setReal(varName, value)
  }

  class fmi4cpp_FMU {
    <<external fmi4cpp>>
  }

  class TuningTask {
    -controllerType
    -state
    -plantModel
    -candidates
    +stepSimulation()
    +update()
    +getResult()
    +noteControllerTestJerkSample(jerk)
  }

  class Sundaresan {
    -state
    -noise
    +Execute()
    +GetPlantModel()
  }

  class SkogestadTuner {
    <<utility>>
    +calculatePI(model, tc)$
    +calculatePID(model, tc)$
  }

  class ParameterValidator {
    <<utility>>
    +validate(params)$
  }

  class loop_control {
    <<namespace>>
    +closeLoop(system)
    +openLoop(system)
    +applyPIDWithTimeConstants(...)
  }

  class PlantModel {
    +K
    +tau1
    +tau2
    +theta
  }

  class PIDParameters {
    +Kp
    +Ti
    +Td
  }

  class MaxDeviationFinder {
    +AddData()
    +GetMaxStdDeviation()
  }

  class WelfordAlgorithm {
    +AddSample()
    +GetDeviation()
  }

  class JerkEstimator {
    +AddSpeed()
  }

  class SimulationVarsTxt {
    +append(...)
  }

  class UDPSender {
    +send(message)
  }

  ISystem <|.. SimulationSystem : implements
  ISystem <|.. HardwareSystem : implements
  SimulationSystem *-- ModelicaWrapper : owns
  ModelicaWrapper *-- fmi4cpp_FMU : uses

  TuningTask *-- Sundaresan : owns
  TuningTask o-- ISystem : uses
  TuningTask ..> SkogestadTuner : uses
  TuningTask ..> ParameterValidator : uses
  TuningTask ..> loop_control : uses
  TuningTask ..> PlantModel : holds
  TuningTask ..> PIDParameters : holds

  Sundaresan o-- ISystem : uses
  Sundaresan *-- MaxDeviationFinder
  Sundaresan *-- WelfordAlgorithm : _welfordCalc
  Sundaresan ..> PlantModel : produces

  MaxDeviationFinder *-- WelfordAlgorithm : x2

  JerkEstimator *-- WelfordAlgorithm : speed + jerk filters

  note for TuningTask "Orquestra identificação (Sundaresan),\nSkogestad, testes em malha fechada."
  note for SimulationSystem "Definido em system_simulation.cpp;\nexpõe ISystem sobre FMU."
```

## Estados do `TuningTask` (`TuningState`)

```mermaid
stateDiagram-v2
  [*] --> PLANT_IDENTIFICATION
  PLANT_IDENTIFICATION --> CONTROLLER_TUNING : planta identificada
  PLANT_IDENTIFICATION --> ERROR : falha Sundaresan
  CONTROLLER_TUNING --> TEST_CONTROLLER : candidatos PID gerados
  CONTROLLER_TUNING --> ERROR : validação / erro
  TEST_CONTROLLER --> COMPLETED : melhor candidato escolhido
  TEST_CONTROLLER --> ERROR : erro
  COMPLETED --> [*]
  ERROR --> [*]
```

## Componentes (CMake / camadas)

```mermaid
flowchart LR
  subgraph exe["Executáveis"]
    sim[sim / main]
    demo[sundaresan_demo]
    tests[tests gtest]
  end
  subgraph simlib["simulation_lib"]
    MW[ModelicaWrapper]
    SI[system_interface + simulation]
  end
  subgraph tunlib["tuning_lib"]
    TT[TuningTask, Sundaresan, loop_control, ...]
  end
  subgraph statlib["statistics_lib"]
    JE[JerkEstimator, Welford, MaxDeviation]
  end
  sim --> simlib
  sim --> tunlib
  simlib --> tunlib
  simlib --> fmi4cpp[fmi4cpp]
  tunlib --> statlib
  demo --> simlib
  tests --> simlib
```
