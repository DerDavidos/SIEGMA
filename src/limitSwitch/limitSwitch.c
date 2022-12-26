#include "limitSwitch.h"
#include "hardware/gpio.h"

bool limitSwitchIsClosed(limitSwitch_t limitSwitch) {
    return gpio_get(limitSwitch.pin);
}

limitSwitch_t createLimitSwitch(uint8_t id) {
    limitSwitch_t limitSwitch;

    switch (id) {
        case 0:
            limitSwitch = LIMIT_SWITCH_0;
            break;
        case 1:
            limitSwitch = LIMIT_SWITCH_1;
            break;
        case 2:
            limitSwitch = LIMIT_SWITCH_2;
            break;
        case 3:
            limitSwitch = LIMIT_SWITCH_3;
            break;
        default:
            return (limitSwitch_t) {.pin=-1};
    }

    gpio_init(limitSwitch.pin);
    gpio_set_dir(limitSwitch.pin, GPIO_IN);
    return limitSwitch;
}
