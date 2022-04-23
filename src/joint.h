//
// Created by Kevin Williams on 4/19/22.
//

#ifndef PICO_SPIDER_JOINT_H
#define PICO_SPIDER_JOINT_H

#include "pca9685.h"

class Joint {
public:
    pca9685_pwm_config_st servo;

};

#endif //PICO_SPIDER_JOINT_H
