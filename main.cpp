#define DEBUG 1

#include <stdio.h>
#include <pico/stdio.h>
#include "hardware/i2c.h"
#include "kinematics//pos.h"
#include "kinematics/link.h"
#include "kinematics/inverse.h"

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
#include "kinematics/chain.h"
#include "src/body.h"

Body * body;
ik_chain_st c0;
ik_chain_st c1;
ik_chain_st c2;
ik_chain_st c3;

bool keypressed() {
    int c = getchar_timeout_us(100);
    if(c > 0 && c != PICO_ERROR_TIMEOUT) {
        printf("Key Pressed! \n");
        return true;
    }
    return false;
}

void init_body() {

    pca9685_t pwm_config = {.i2c = PWM_DRIVER_I2C, .sda = PWM_DRIVER_SDA, .scl = PWM_DRIVER_SCL, .i2c_address = PWM_DRIVER_ADDRESS};

    ik_link_params_st coxa_params = {
            .id = "COXA",
            .theta_base = COXA_THETA,
            .theta_min = COXA_THETA_MIN,
            .theta_max = COXA_THETA_MAX,
            .dh = {.r = COXA_LEN, .alpha = COXA_ALPHA, .d = 0}
    };

    ik_link_params_st femur_params = {
            .id = "FEMUR",
            .theta_base = FEMUR_THETA,
            .theta_min = FEMUR_THETA_MIN,
            .theta_max = FEMUR_THETA_MAX,
            .dh = {.r = FEMUR_LEN, .alpha = FEMUR_ALPHA, .d = 0}
    };
    ik_link_params_st tibia_params = {
            .id = "TIBIA",
            .theta_base = TIBIA_THETA,
            .theta_min = TIBIA_THETA_MIN,
            .theta_max = TIBIA_THETA_MAX,
            .dh = {.r = TIBIA_LEN, .alpha = TIBIA_ALPHA, .d = 0}
    };

    ik_link_st leg0_links[3] = {
            ik_link_init(coxa_params),
            ik_link_init(femur_params),
            ik_link_init(tibia_params)
    };

    ik_link_st leg1_links[3] = {
            ik_link_init(coxa_params),
            ik_link_init(femur_params),
            ik_link_init(tibia_params)
    };

    ik_link_st leg2_links[3] = {
            ik_link_init(coxa_params),
            ik_link_init(femur_params),
            ik_link_init(tibia_params)
    };

    ik_link_st leg3_links[3] = {
            ik_link_init(coxa_params),
            ik_link_init(femur_params),
            ik_link_init(tibia_params)
    };

    ik_chain_params_st leg0 = {.id = "LEG-0", .links = leg0_links, .num_links = 3};



    ik_chain_params_st leg1 = {.id = "LEG-1", .links = leg1_links, .num_links = 3};
    ik_chain_params_st leg2 = {.id = "LEG-2", .links = leg2_links, .num_links = 3};
    ik_chain_params_st leg3 = {.id = "LEG-3", .links = leg3_links, .num_links = 3};


    c0 = ik_chain_init(leg0);
    c1 = ik_chain_init(leg1);
    c2 = ik_chain_init(leg2);
    c3 = ik_chain_init(leg3);

    body = new Body(pwm_config, c0, c1, c2, c3);



}
void test_servo() {
    pca9685_t svc = {i2c0, 4,5, 0x40};
    pca9685_begin(&svc);
    pca9685_set_frequency(&svc, 50);
    while(!keypressed()) {
        pca9685_set_micros(&svc, 1, SERVO_MIN_PWM);
        printf("%d\n", SERVO_MIN_PWM);
        sleep_ms(5000);
        if(!keypressed())
            break;
        pca9685_set_micros(&svc, 1, SERVO_MAX_PWM);
        printf("%d\n", SERVO_MAX_PWM);
        sleep_ms(5000);
    }
}

void test() {

    ik_link_st coxa0;
    ik_link_st femur0;
    ik_link_st tibia0;
    ik_chain_st chain0;

    coxa0 = ik_link_init({"COXA0", 45, -10, 90, {24,90,0}});
    femur0 = ik_link_init({"FEMUR0", 0, -90, 90, {38,0,0}});
    tibia0 = ik_link_init({"TIBIA0", 90,0, 90+45, {77,0,0}});

    ik_link_st links[3] = {coxa0, femur0, tibia0};

    chain0 = ik_chain_init({.id = "LEG0", .links = links, .num_links = 3});


    printf("Position: \n");
    ik_pos3d_print(chain0.position);
    ik_3dof_result_st angles_est = ik_estimate_3dof_chain(&chain0, chain0.position);
    printf("\nAngles: \n");
    ik_3dof_result_print(angles_est);

}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();

    while(!keypressed()){}
    init_body();

    printf("\ncompare\n");
    printf("Position: \n");
    ik_pos3d_print(body->chains[0].position);
    ik_3dof_result_st angles = ik_estimate_3dof_chain(&body->chains[0], body->chains[0].position);
    printf("\nAngles: \n");
    ik_3dof_result_print(angles);

    //test_servo();
    return 0;
}


/*
 *

 */