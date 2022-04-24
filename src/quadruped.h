//
// Created by Kevin Williams on 4/19/22.
//

#ifndef PICO_SPIDER_QUADRUPED_H
#define PICO_SPIDER_QUADRUPED_H
#include <ik/chain.h>

using namespace ik;

static const vector<vector<float>> HOME_OFFSET = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},};

class Quadruped {

public:

    static const int num_chains = 4;
    static const int dof = 3;
    static const int num_links = dof*num_chains;

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

    vector<vector<float>> getPositions() {
        vector<vector<float>> p;
        for (const auto &item: legs){
            p.push_back(item.position);
        }
        return p;
    }

    vector<vector<float>> getAngles() {
        vector<vector<float>> p;
        for (const auto &item: legs){
            p.push_back(item.angle);
        }
        return p;
    }

    vector<vector<float>> getTargetAngles() {
        vector<vector<float>> p;
        for (const auto &item: legs){
            p.push_back(item.targetAngle);
        }
        return p;
    }

    void calcPositions() {
        for (auto &item: legs){
            item.calcPosition();
        }
    }

    void setTargetPositions(vector<vector<float>> p) {
        for(int i=0; i < num_chains; i++) {
            legs.at(i).targetAngle = ik3d(&legs.at(i), p.at(i));
        }
    }

    void setTargetOffsets(vector<vector<float>> p) {
        for(int i=0; i < num_chains; i++) {
            legs.at(i).targetAngle = ik3d(&legs.at(i), legs.at(i).positionOffset(p.at(i)));
        }
    }

    void moveTowardTargets() {
        if(atTargets()) return;

        for (auto &item: legs) {
            item.moveTowardTarget(1);
        }
        applyAngles();
    }

    bool atTargets() {
        for (auto &item: legs) {
            if(!item.atTarget()) {
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
        for(auto &item : legs) {
            for(auto &item : item.angle) {
                pca9685_set_position(&pwm_config, *it++, item * (*ot++));
            }
        }
    }

};

#endif //PICO_SPIDER_QUADRUPED_H
