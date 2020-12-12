#include "vl6180x.hpp"


VL6180X::VL6180X(i2c_port_t port, uint8_t addr, SemaphoreHandle_t mutex) {
    static MyDev_t device;
    dev               = &device;
    dev->i2c_port_num = port;
    dev->i2c_address  = addr;
    dev->mutex        = mutex;

    // VL6180x_SetupGPIO1(dev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
    // VL6180x_RangeConfigInterrupt(dev, CONFIG_GPIO_INTERRUPT_LEVEL_HIGH);

    VL6180x_InitData(dev);
    VL6180x_Prepare(dev);

    vTaskDelay(pdMS_TO_TICKS(10));
}

VL6180X::~VL6180X() {
    // delete dev;
}

// VL6180x_RangeData_t& VL6180X::range() const {
//     static VL6180x_RangeData_t range;
//     VL6180x_RangePollMeasurement(dev, &range);
//     return range;  // magic static
// }
