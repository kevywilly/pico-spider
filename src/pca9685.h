//
// Created by Kevin Williams on 4/15/22.
// credit: https://github.com/aleopardstail/PCA9685_Servo_Controller
//

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#ifndef PICO_PCA9685_H
#define PICO_PCA9685_H

#ifndef PCA9685_I2C_ADDRESS
#define PCA9685_I2C_ADDRESS (uint8_t)0x40
#endif

#ifndef MAPFN
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min)
#endif

#ifndef SERVO_MIN_PWM
#define SERVO_MIN_PWM 104
#endif

#ifndef SERVO_MAX_PWM
#define SERVO_MAX_PWM 508
#endif

#ifndef SERVO_MAX_ANGLE
#define SERVO_MAX_ANGLE 90
#endif

#ifndef SERVO_MIN_ANGLE
#define SERVO_MIN_ANGLE -90
#endif

typedef struct pca9685_pwm_config_st {
    i2c_inst_t *i2c;
    uint8_t sda;
    uint8_t scl;
    uint8_t i2c_address;
    uint8_t frequency;
} pca9685_pwm_config_st;

void pca9685_write_register(pca9685_pwm_config_st *pca, uint8_t reg, uint8_t value) {
    uint8_t D[2];
    D[0] = reg;
    D[1] = value;
    i2c_write_blocking(pca->i2c, pca->i2c_address, D, 2, false);    // write register then value
    return;
}

uint8_t pca9685_read_register(pca9685_pwm_config_st *pca, uint8_t reg) {
    uint8_t D[1];
    D[0] = reg;

    i2c_write_blocking(pca->i2c, pca->i2c_address, D, 1, false);    // write register
    i2c_read_blocking(pca->i2c, pca->i2c_address, D, 1, false);        // read value

    return D[0];
}

void pca9685_set_frequency(pca9685_pwm_config_st *pca, uint16_t frequency) {
    int preScalerVal = (25000000 / (4096 * frequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

    //need to be in sleep mode to set the pre-scaler
    uint8_t M1 = pca9685_read_register(pca, 0x00);
    pca9685_write_register(pca, 0x00, ((M1 & ~0b10000000) | 0b00010000));
    pca9685_write_register(pca, 0xFE, (uint8_t) preScalerVal);

    // restart
    pca9685_write_register(pca, 0x00, ((M1 & ~0b00010000) | 0b10000000));
    sleep_us(
            500);        // <-- sleep functions not ideal, better to have a flag that clears after a period of time (or checks
    // to see if it should be cleared when its value is checked)
    return;
}

void pca9685_set_micros(pca9685_pwm_config_st *pca, uint8_t channel, int16_t micros) {
    // map the angle to a PWM value (-90 to +90) to the servo
    // PWM_min & PWM_max values

    uint16_t PWM = micros;
    uint16_t ChannelOffset = channel * 10;        // adds 0-160 to the counter values


    uint8_t D[5];

    uint16_t ChannelOn = 0 + ChannelOffset;
    uint16_t ChannelOff = PWM + ChannelOffset;

    D[0] = 0x06 + (4 * channel);
    D[1] = (0x00FF & ChannelOn);
    D[2] = (0xFF00 & ChannelOn) >> 8;
    D[3] = (0x00FF & ChannelOff);
    D[4] = (0xFF00 & ChannelOff) >> 8;

    i2c_write_blocking(pca->i2c, pca->i2c_address, D, 5, false);

}

void pca9685_set_position(pca9685_pwm_config_st *pca, uint8_t channel, int8_t angle) {
    // map the angle to a PWM value (-90 to +90) to the servo
    // PWM_min & PWM_max values


    uint16_t PWM = (uint16_t) map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PWM, SERVO_MAX_PWM);
    uint16_t ChannelOffset = channel * 10;        // adds 0-160 to the counter values

    uint8_t D[5];

    uint16_t ChannelOn = 0 + ChannelOffset;
    uint16_t ChannelOff = PWM + ChannelOffset;

    D[0] = 0x06 + (4 * channel);
    D[1] = (0x00FF & ChannelOn);
    D[2] = (0xFF00 & ChannelOn) >> 8;
    D[3] = (0x00FF & ChannelOff);
    D[4] = (0xFF00 & ChannelOff) >> 8;

    i2c_write_blocking(pca->i2c, pca->i2c_address, D, 5, false);

}

void pca9685_begin(pca9685_pwm_config_st *pca) {
    i2c_init(pca->i2c, 100 * 1000);
    gpio_set_function(pca->sda, GPIO_FUNC_I2C);
    gpio_set_function(pca->scl, GPIO_FUNC_I2C);
    gpio_pull_up(pca->sda);
    gpio_pull_up(pca->scl);

    // configure the PCA9685 for driving servos
    pca9685_write_register(pca, 0x00, 0b10100000);
    pca9685_write_register(pca, 0x01, 0b00000100);

    pca9685_set_frequency(pca, pca->frequency);
}

/*
 * bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
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
 */

#endif //PICO_SPIDER_PCA9685_H
