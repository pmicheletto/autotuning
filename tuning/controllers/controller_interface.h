#ifndef TUNING_CONTROLLER_INTERFACE_H_
#define TUNING_CONTROLLER_INTERFACE_H_

#include "controller_type.h"

class Controller {
public:
    virtual ~Controller() = default;
    virtual ControllerType getType() const = 0;
};

#endif
