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
    pca9685_pwm_config_st pwm_config;
    vector<Chain> chains;
    vector<int> pins;

    Quadruped() {}
    Quadruped(vector<Chain> chains, vector<int> pins, pca9685_pwm_config_st pwm_config) : chains(chains), pins(pins), pwm_config(pwm_config) {
    }

    void init() {
        pca9685_begin(&pwm_config);
    }

    vector<vector<float>> getPositions() {
        vector<vector<float>> p;
        for (const auto &item: chains){
            p.push_back(item.position);
        }
        return p;
    }

    vector<vector<float>> getAngles() {
        vector<vector<float>> p;
        for (const auto &item: chains){
            p.push_back(item.angle);
        }
        return p;
    }

    vector<vector<float>> getTargetAngles() {
        vector<vector<float>> p;
        for (const auto &item: chains){
            p.push_back(item.targetAngle);
        }
        return p;
    }

    void calcPositions() {
        for (auto &item: chains){
            item.calcPosition();
        }
    }

    void setTargetPositions(vector<vector<float>> p) {
        for(int i=0; i < num_chains; i++) {
            chains.at(i).targetAngle = ik3d(&chains.at(i), p.at(i));
        }
    }

    void setTargetOffsets(vector<vector<float>> p) {
        for(int i=0; i < num_chains; i++) {
            chains.at(i).targetAngle = ik3d(&chains.at(i), chains.at(i).positionOffset(p.at(i)));
        }
    }

    void moveTowardTargets() {
        for (auto &item: chains) {
            item.moveTowardTarget(1);
        }
    }
    bool atTargets() {
        for (auto &item: chains) {
            if(!item.atTarget()) {
                return false;
            }
        }
        return true;
    }

    void print() {
        for (auto &item: chains) {
            item.print();
        }
    }

};

#endif //PICO_SPIDER_QUADRUPED_H
