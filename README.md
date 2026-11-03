# Autotuning

Autotuning is a C++17 project for control engineers and researchers who want
to experiment with automatic PID tuning on Modelica plant models. It uses
FMUs for simulation and includes tuning algorithms, a simulation wrapper,
statistics utilities, and unit/integration tests.

## Authorship

I,[Patrícia Micheletto](https://github.com/pmicheletto), created and developed this project. I began working
on it in February 2026, as reflected in the project history.

The project is built with CMake and uses [FMI4cpp](https://github.com/NTNU-IHB/FMI4cpp)
as a Git submodule. GoogleTest is downloaded automatically during CMake
configuration.

## Requirements

- CMake 3.15 or newer
- A C++17 compiler (for example, GCC or Clang)
- Git, including support for submodules
- libzip and pugixml development headers and libraries
- Internet access on the first configure, to download GoogleTest

On Debian/Ubuntu, install the compiler and build tools with:

```sh
sudo apt-get update
sudo apt-get install -y build-essential cmake git libzip-dev libpugixml-dev
```

## Build and test

Clone the repository including its FMI4cpp submodule, then configure, build,
and run the tests:

```sh
git clone --recurse-submodules https://github.com/pmicheletto/autotuning.git
cd autotuning
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

If the repository was cloned without submodules, initialize them before
configuring:

```sh
git submodule update --init --recursive
```

## Run the simulation

The `sim` executable uses `simulation/modelica_models/variantC.fmu` by default.
Run it from the build directory so its default relative path resolves:

```sh
cd build
./sim
```

An alternate FMU can be supplied as the first argument:

```sh
./build/sim /path/to/model.fmu
```

The simulation sends UDP data to `127.0.0.1:9870` and writes `sim_trace.txt`
in its working directory. FMI4cpp is a separate third-party project and keeps
its own license.

## License

This project is distributed under the Mozilla Public License 2.0. See
[LICENSE](LICENSE) for the full text.
