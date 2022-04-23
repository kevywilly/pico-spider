//
// Created by Kevin Williams on 4/19/22.
//

#ifndef PICO_SPIDER_QUADRUPED_H
#define PICO_SPIDER_QUADRUPED_H

#include <fk/chain.h>

using namespace fk;


class Quadruped {

public:
    static const int num_chains = 4;
    static const int dof = 3;
    static const int num_links = dof*num_chains;
    pca9685_pwm_config_st pwm_config;
    Chain * chains;
    Link ** links;
    uint8_t * pins;
    float * targetAngles;
    float * targetPositions;

    Quadruped() {}
    Quadruped(Chain *chains, uint8_t *, pca9685_pwm_config_st pwm_config) : chains(chains), pins(pins), pwm_config(pwm_config) {
        links = new Link*[num_links];
        Link ** p = links;
        for(int i=0; i < num_chains; i++) {
            for(int j = 0; j<dof; j++) {
                *p++ = &chains[i].links[j];
            }
        }

        float * targetPositions = new float[12];
        float * targetAngles = new float[12];
        for(int i=0; i < 12; i++) {
            targetAngles[i] = 0;
            targetPositions[i] = 0;
        }
    }

    void init() {
        pca9685_begin(&pwm_config);
    }

    f3d_t * getPositions() {
        f3d_t * p = new f3d_t[num_chains];
        for(int i=0; i < num_chains; i++) {
            p[i] = chains[i].position;
        }
        return p;
    }

    void calcPositions(f3d_t * results) {
        for(int i=0; i < num_chains; i++) {
            results[i] = chains[i].calcPosition();
        }
    }

    void setTargetPositions(f3d_t * positions) {
        float * a = targetAngles;
        float * p = targetPositions;

        for(int i=0; i < num_chains; i++) {
            *p++ = positions[i].x;
            *p++ = positions[i].y;
            *p++ = positions[i].z;

            ik3d(&chains[i], positions[i], a);
            a+=3;
        }
    }

    void setTargetPositions(f3d_t position) {
        f3d_t p[4] = {position, position, position, position};
        setTargetPositions(p);
    }


    void calcTargets() {
        //Pos3d * p = targetAngles;
        for(int i=0; i < num_chains; i++) {

        }
    }


};

#endif //PICO_SPIDER_QUADRUPED_H
