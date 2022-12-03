#include "touchSensor.h"
#include "hardware/gpio.h"

bool touchSensorIsClosed(uint8_t id) {
    switch (id) {
        case 0:
            return gpio_get(STEPPER0_LS_PIN);
        case 1:
            return gpio_get(STEPPER1_LS_PIN);
        case 2:
            return gpio_get(STEPPER2_LS_PIN);
        case 3:
            return gpio_get(STEPPER3_LS_PIN);
        default:
            return false;
    }
}

void setUpTouchSensor(uint8_t id) {
    switch (id) {
        case 0:
            gpio_init(STEPPER0_LS_PIN);
            gpio_set_dir(STEPPER0_LS_PIN, GPIO_IN);
            break;
        case 1:
            gpio_init(STEPPER1_LS_PIN);
            gpio_set_dir(STEPPER1_LS_PIN, GPIO_IN);
            break;
        case 2:
            gpio_init(STEPPER2_LS_PIN);
            gpio_set_dir(STEPPER2_LS_PIN, GPIO_IN);
            break;
        case 3:
            gpio_init(STEPPER3_LS_PIN);
            gpio_set_dir(STEPPER3_LS_PIN, GPIO_IN);
            break;
        default:
            break;
    }
}

void setUpAllTouchSensors(void) {
    for (int i = 0; i < 4; ++i) {
        setUpTouchSensor(i);
    }
}


