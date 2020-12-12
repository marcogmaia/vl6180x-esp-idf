#pragma once

#include "vl6180x_api.h"

class VL6180X {
private:
    VL6180xDev_t dev;

public:
    VL6180X(i2c_port_t port, uint8_t addr, SemaphoreHandle_t mutex);
    ~VL6180X();
    
    VL6180x_RangeData_t& range() const {
        static VL6180x_RangeData_t range;
        VL6180x_RangePollMeasurement(dev, &range);
        return range;  // magic static
    }
};
