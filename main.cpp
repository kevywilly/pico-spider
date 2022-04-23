#define DEBUG 1

#include <vector>
#include <stdio.h>
#include <pico/stdio.h>
#include "hardware/i2c.h"
#include "ik/chain.h"
#include "ik/inverse.h"


#define COXA_LEN 24
#define COXA_THETA 45
#define COXA_THETA_MIN -10
#define COXA_THETA_MAX 90
#define COXA_ALPHA 90

#define FEMUR_LEN 38
#define FEMUR_THETA 0
#define FEMUR_THETA_MIN -90
#define FEMUR_THETA_MAX 90
#define FEMUR_ALPHA 0

#define TIBIA_LEN 77
#define TIBIA_THETA 90
#define TIBIA_THETA_MIN 0
#define TIBIA_THETA_MAX 90+45
#define TIBIA_ALPHA 0

#define PWM_DRIVER_SDA 4
#define PWM_DRIVER_SCL 5
#define PWM_DRIVER_I2C i2c0
#define PWM_DRIVER_ADDRESS 0x40

#define SERVO_MIN_PWM 124
#define SERVO_MAX_PWM 544

#include "src/pca9685.h"
#include "src/quadruped.h"

using namespace ik;

Quadruped *quadruped;

bool keypressed() {
    int c = getchar_timeout_us(100);
    if (c > 0 && c != PICO_ERROR_TIMEOUT) {
        printf("Key Pressed! \n");
        return true;
    }
    return false;
}

void test_servo() {
    pca9685_pwm_config_st svc = {i2c0, 4, 5, 0x40};
    pca9685_begin(&svc);
    pca9685_set_frequency(&svc, 50);
    while (!keypressed()) {
        pca9685_set_micros(&svc, 1, SERVO_MIN_PWM);
        printf("%d\n", SERVO_MIN_PWM);
        sleep_ms(5000);
        if (!keypressed())
            break;
        pca9685_set_micros(&svc, 1, SERVO_MAX_PWM);
        printf("%d\n", SERVO_MAX_PWM);
        sleep_ms(5000);
    }
}

void init_body() {
    vector<Chain> chains;
    chains.push_back(Chain("c0", {COXA_THETA,FEMUR_THETA,TIBIA_THETA}, {COXA_LEN,FEMUR_LEN,TIBIA_LEN}, {COXA_ALPHA,FEMUR_ALPHA,TIBIA_ALPHA},{0,0,0}));
    chains.push_back(Chain("c1", {COXA_THETA,FEMUR_THETA,TIBIA_THETA}, {COXA_LEN,FEMUR_LEN,TIBIA_LEN}, {COXA_ALPHA,FEMUR_ALPHA,TIBIA_ALPHA},{0,0,0}));
    chains.push_back(Chain("c2", {COXA_THETA,FEMUR_THETA,TIBIA_THETA}, {COXA_LEN,FEMUR_LEN,TIBIA_LEN}, {COXA_ALPHA,FEMUR_ALPHA,TIBIA_ALPHA},{0,0,0}));
    chains.push_back(Chain("c3", {COXA_THETA,FEMUR_THETA,TIBIA_THETA}, {COXA_LEN,FEMUR_LEN,TIBIA_LEN}, {COXA_ALPHA,FEMUR_ALPHA,TIBIA_ALPHA},{0,0,0}));

    vector<int> pins = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};

    quadruped = new Quadruped(
            chains,
            pins,
            {.i2c = i2c0, .sda = PWM_DRIVER_SDA, .scl = PWM_DRIVER_SCL, .i2c_address = PWM_DRIVER_ADDRESS, .frequency = 50}
    );
}

int main() {

    // Enable UART so we can print status output
    stdio_init_all();

    while (!keypressed()) {}

    init_body();
    quadruped->print();

    vector<vector<float>> pos = quadruped->getPositions();
    quadruped->setTargetPositions(pos);
    while(!quadruped->atTargets()) {
        quadruped->moveTowardTargets();
    }

    quadruped->print();
    quadruped->setTargetOffsets({{0,0,-10},{0,0,-10}, {0,0,0}, {0,0,0}});
    while(!quadruped->atTargets()) {
        quadruped->moveTowardTargets();
    }
    quadruped->print();
    //test_servo();
    return 0;
}


/*
 *

 */