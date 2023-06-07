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

void PWM_init(void);
void duty_change(uint8_t *pwm_buf, uint8_t channel, float duty);
void phase_change(uint8_t *pwm_buf, uint8_t channel, float phase);
void pwm_dev_addr(uint8_t *pwm_buf, uint8_t channel, uint8_t addr);
void spi_send_data(uint8_t *data, uint32_t len);

#endif