#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include "sysctl.h"
#include "bsp.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"
#include "spi.h"

#define PWM_PERIOD 0x1000U
#define NUM_OF_PWM_CHANNEL 16

extern uint8_t pwm_txf[5*NUM_OF_PWM_CHANNEL];

void PWM_init(void);
void duty_change(uint8_t channel, float duty);
void phase_change(uint8_t channel, float phase);
void pwm_dev_addr(uint8_t channel, uint8_t addr);
void spi_send_data(uint8_t *data, uint32_t len);
#define pwm_update() spi_send_data_standard(PWM_SPI, PWM_SPI_CHIP_SELECT, NULL, 0, (const uint8_t *)pwm_txf, 5*NUM_OF_PWM_CHANNEL);
#endif