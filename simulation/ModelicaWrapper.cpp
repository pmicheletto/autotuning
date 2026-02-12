#include "ModelicaWrapper.h"
#include <stdexcept>

ModelicaWrapper::ModelicaWrapper(const std::string& fmuPath) {
    _m_fmu = std::make_unique<fmi4cpp::fmi2::fmu>(fmuPath);
    
    _m_slave = _m_fmu->as_cs_fmu()->new_instance();
}

void ModelicaWrapper::init(double startTime) {
    _m_time = startTime;
    _m_slave->setup_experiment(startTime);
    _m_slave->enter_initialization_mode();
    _m_slave->exit_initialization_mode();
}

void ModelicaWrapper::stepFMU(double stepSize) {
    if (!_m_slave->step(stepSize)) {
        throw std::runtime_error("Erro no passo da FMU!");
    }
}

double ModelicaWrapper::getReal(const std::string& varName) {
    double value = 0.0;
    auto vr = _m_fmu->get_model_description()->get_variable_by_name(varName).value_reference;
    
    if (!_m_slave->read_real(vr, value)) {
        throw std::runtime_error("Erro ao ler variavel: " + varName);
    }
    return value;
}

void ModelicaWrapper::setReal(const std::string& varName, double value) {
    auto vr = _m_fmu->get_model_description()->get_variable_by_name(varName).value_reference;
    
    if (!_m_slave->write_real(vr, value)) {
        throw std::runtime_error("Erro ao escrever variavel: " + varName);
    }
}