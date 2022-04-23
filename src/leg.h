//
// Created by Kevin Williams on 4/19/22.
//

#ifndef PICO_SPIDER_LEG_H
#define PICO_SPIDER_LEG_H

#include <kinematics/link.h>
#include "structs.h"
#include "pca9685.h"


class Leg {
private:
    pca9685_pwm_config_st *_servo_controller;
public:
    static const uint8_t num_joints = 3;
    ik_link_st link;
    const uint8_t quadrant;
    uint8_tuple3_st pwm_channels; // channels on adafruit 16 pwm device

    Leg(const ik_link_st &link, const uint8_t quadrant, const uint8_tuple3_st &pwm_channel) : pwm_channels(
            pwm_channels), link(link),
                                                                                              quadrant(quadrant) {

    }


};

#endif //PICO_SPIDER_LEG_H
