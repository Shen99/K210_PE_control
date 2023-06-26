#ifndef PWM_H
#define PWM_H

#include "spi.h"

#include "pin_config.h"

#define PWM_PERIOD 0x1000U
#define NUM_OF_PWM_CHANNEL 16
#define PWM_TXF_SIZE 5*NUM_OF_PWM_CHANNEL

#if PWM_USE_DMA
    extern uint32_t pwm_txf[PWM_TXF_SIZE];
    extern volatile uint8_t pwm_update_status;

    extern spi_data_t pwm_data;
    extern plic_interrupt_t pwm_irq;

    #define pwm_update_non_blocking() pwn_spi_dma_transfer(PWM_SPI, PWM_SPI_CHIP_SELECT, pwm_data, &pwm_irq);
    #define pwm_update() {pwm_update_status = 0; pwn_spi_dma_transfer(PWM_SPI, PWM_SPI_CHIP_SELECT, pwm_data, &pwm_irq); while(!pwm_update_status) {;}}
#else
    extern uint8_t pwm_txf[PWM_TXF_SIZE];
    #define pwm_update() spi_send_data_standard(PWM_SPI, PWM_SPI_CHIP_SELECT, NULL, 0, (const uint8_t *)pwm_txf, PWM_TXF_SIZE);
#endif

void PWM_init(void);
void duty_change(uint8_t channel, float duty);
void phase_change(uint8_t channel, float phase);
void pwm_dev_addr(uint8_t channel, uint8_t addr);
void spi_send_data(uint8_t *data, uint32_t len);

#endif