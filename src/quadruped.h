//
// Created by Kevin Williams on 4/19/22.
//

#ifndef PICO_SPIDER_QUADRUPED_H
#define PICO_SPIDER_QUADRUPED_H

#include <ik/chain.h>

using namespace ik;

#define ZERO_TRANSFORM  {{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
class Quadruped {

public:

    static const int num_chains = 4;
    static const int dof = 3;
    static const int num_links = dof * num_chains;

    transform_matrix_t home = ZERO_TRANSFORM;

    /**
     * Vector of legs that make up the legs of the quaduped
     */
    vector<Chain> legs;
    /**
     * Servo Channel on Servo Controller
     */
    vector<int> channels;

    /**
     * Orientation of servo 1 normal, -1 means inverted
     */
    vector<int> orientation;
    /**
     * PWM Controller
     */
    pca9685_pwm_config_st pwm_config;

    Quadruped() {}

    Quadruped(const vector<Chain> &legs, const vector<int> &channels, const vector<int> &orientation,
              const pca9685_pwm_config_st &pwmConfig) : legs(legs), channels(channels), orientation(orientation),
                                                        pwm_config(pwmConfig) {}

    void init() {
        pca9685_begin(&pwm_config);
    }

    void setHome(transform_matrix_t t) {
        home = t;
    }

    void resetHome() {
        home = ZERO_TRANSFORM;
    }

    void goHome() {
        setTransform(home);
        applyTransform(0);
    }

    vector<vector<float>> getAngles() {
        return {legs[0].angles, legs[1].angles, legs[2].angles, legs[3].angles};
    }

    void calcPositions() {
        for (auto &item: legs) {
            item.calcPosition();
        }
    }

    void setTransform(transform_matrix_t transform) {
        transform_matrix_t t = add(transform, home);
        for (int i = 0; i < num_chains; i++) {
            legs.at(i).targetAngles = ik3d_transform(&legs.at(i), transform.at(i));
        }
    }

    /**
     * Move gradually toward targets
     */
    void applyTransform(int step = 0) {
        if (atTargets()) return;

        for (auto &item: legs) {
            item.moveTowardTarget(1);
        }
        applyAngles();
    }

    bool atTargets() {
        for (auto &item: legs) {
            if (!item.atTarget()) {
                return false;
            }
        }
        return true;
    }

    void print() {
        for (auto &item: legs) {
            item.print();
        }
    }

    void applyAngles() {
        vector<int>::iterator it = channels.begin();
        vector<int>::iterator ot = orientation.begin();
        for (auto &item: legs) {
            for (auto &item: item.angles) {
                pca9685_set_position(&pwm_config, *it++, item * (*ot++));
            }
        }
    }

};

#endif //PICO_SPIDER_QUADRUPED_H
