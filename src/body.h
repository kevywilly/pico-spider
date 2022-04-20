//
// Created by Kevin Williams on 4/19/22.
//

#ifndef PICO_SPIDER_BODY_H
#define PICO_SPIDER_BODY_H

#include <kinematics/chain.h>

class Body {
private:
    pca9685_t _pwm;
public:
    static const uint8_t num_chains = 4;
    ik_chain_st * chains;

    Body(pca9685_t pwm_config, ik_chain_st c0, ik_chain_st c1, ik_chain_st c2, ik_chain_st c3) {
        _pwm = pwm_config;
        chains = new ik_chain_st [4];
        chains[0] = c0;
        chains[1] = c1;
        chains[2] = c2;
        chains[3] = c3;
    }

    void Init() {
        // Init PWM
        pca9685_begin(&_pwm);
        pca9685_set_frequency(&_pwm, 50);

        // Init Chains
        for(int i=0; i < num_chains; i++) {
            for(int j=0; j < chains[i].params.num_links; j++) {

            }
        }
    }

    void GoHome(ik_link_st link) {
        //link.params.theta =
    }
};
#endif //PICO_SPIDER_BODY_H
