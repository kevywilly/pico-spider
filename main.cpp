#include <stdio.h>
#include <pico/stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "kinematics//pos.h"
#include "kinematics/matrix.h"
#include "kinematics/link.h"

#define SERVO_MIN_PWM 124
#define SERVO_MAX_PWM 544

#include "src/pca9685.h"
#include "kinematics/chain.h"


bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

#define SDA 4
#define SCL 5


void wait_for_key() {
    bool done = false;
    while(!done) {
        int c = getchar_timeout_us(100);
        done = (c > 0 && c != PICO_ERROR_TIMEOUT);
        printf("\npress any key to continue\n");
        sleep_ms(1000);
    }
}

void scani2c() {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}

void test_servo() {
    pca9685_t svc = pca9685_instance(i2c0, 4,5, 0x40);
    pca9685_begin(&svc);
    pca9685_set_frequency(&svc, 50);
    while(1) {
        pca9685_set_micros(&svc, 1, SERVO_MIN_PWM);
        printf("%d\n", SERVO_MIN_PWM);
        sleep_ms(5000);
        pca9685_set_micros(&svc, 1, SERVO_MAX_PWM);
        printf("%d\n", SERVO_MAX_PWM);
        sleep_ms(5000);
    }
}

ik_link_st coxa0;
ik_link_st femur0;
ik_link_st tibia0;
ik_chain_st chain0;


int main() {
    // Enable UART so we can print status output
    stdio_init_all();

    coxa0 = ik_link_init("COXA0", 0, -10, 90,{24,90,0});
    femur0 = ik_link_init("FEMUR0", 0, -90, 90, {38,0,0});
    tibia0 = ik_link_init("TIBIA0", 0, 90, 90+45, {77,0,0});
    ik_link_st links[3] = {coxa0, femur0, tibia0};

    chain0 = ik_chain_init("LEG0", links, 3);

    //_ik_chain_gen_fk(&chain0);
    while(1) {
        ik_pos3d_print(chain0.position);
    }

    test_servo();
    return 0;
}
