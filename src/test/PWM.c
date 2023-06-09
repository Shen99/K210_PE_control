#include "pin_config.h"
#include "PWM.h"

#define PWM_CS_LOW()    gpiohs_set_pin(PWM_CS_GPIO_NUM, GPIO_PV_LOW)
#define PWM_CS_HIGH()   gpiohs_set_pin(PWM_CS_GPIO_NUM, GPIO_PV_HIGH)

#if PWM_USE_DMA
    uint32_t pwm_txf[5*NUM_OF_PWM_CHANNEL] = {0, 0, 0, 0, 0x0U};
    volatile uint8_t pwm_update_status = false;

    int PWM_update_done(void *ctx);

    spi_data_t pwm_data = (spi_data_t)
    {
        .tx_channel = PWM_TX_DMA_CHANNEL,
        .tx_buf = pwm_txf,
        .tx_len = PWM_TXF_SIZE,
        .transfer_mode = SPI_TMOD_TRANS,
        .fill_mode = false
    };

    plic_interrupt_t pwm_irq = (plic_interrupt_t)
    {
        .callback = PWM_update_done,
        .ctx = NULL,
        .priority = 1,
    };

    int PWM_update_done(void *ctx)
    {
        pwm_update_status = true;
        return 0;
    }
#else
    uint8_t pwm_txf[5*NUM_OF_PWM_CHANNEL] = {0, 0, 0, 0, 0x0U};
#endif

void PWM_init(void)
{
    fpioa_set_function(PWM_SCLK, PWM_SCLK_FUNC);
    fpioa_set_function(PWM_CS, PWM_CS_FUNC);
    fpioa_set_function(PWM_MOSI, PWM_MOSI_FUNC);
    fpioa_set_function(PWM_MISO, PWM_MISO_FUNC);
    fpioa_set_function(PWM_SYNC, PWM_SYNC_FUNC);
    fpioa_set_function(PWM_RESET, PWM_RESET_FUNC);
    fpioa_set_function(PWM_DATA_SYNC, PWM_DATA_SYNC_FUNC);

    gpiohs_set_drive_mode(PWM_SYNC_GPIO_NUM, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(PWM_RESET_GPIO_NUM, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(PWM_DATA_SYNC_GPIO_NUM, GPIO_DM_OUTPUT);
    // gpiohs_set_drive_mode(PWM_CS_GPIO_NUM, GPIO_DM_OUTPUT);

    spi_init(PWM_SPI, PWM_SPI_MODE, PWM_SPI_FORMAT, 8, 0);
    uint32_t spi_rate = spi_set_clk_rate(PWM_SPI, PWM_SPI_RATE);
    printf("PWM spi freq: %u\n", spi_rate);

    gpiohs_set_pin(PWM_DATA_SYNC_GPIO_NUM, GPIO_PV_HIGH);
    gpiohs_set_pin(PWM_SYNC_GPIO_NUM, GPIO_PV_LOW);
    // PWM_CS_HIGH();

    gpiohs_set_pin(PWM_RESET_GPIO_NUM, GPIO_PV_LOW);
    msleep(10);
    gpiohs_set_pin(PWM_RESET_GPIO_NUM, GPIO_PV_HIGH);

    msleep(40);
    gpiohs_set_pin(PWM_SYNC_GPIO_NUM, GPIO_PV_HIGH);

    for (int i = 0; i < 5*NUM_OF_PWM_CHANNEL; i++)
    {
        pwm_txf[i] = 0;
    }
    for (int i = 0; i < NUM_OF_PWM_CHANNEL; i++)
    {
        pwm_dev_addr(i, 0);
        duty_change(i, 0.1f);
        phase_change(i, 1.0f);
    }
    for (int i = 8; i < NUM_OF_PWM_CHANNEL; i++)
    {
        pwm_dev_addr(i, 1);
    }
}

// duty 0 ~ 1.0
void duty_change(uint8_t channel, float duty)
{
    uint16_t duty_num =  (uint16_t)(duty * PWM_PERIOD);
    if (duty == 0.0f)
    {
        duty_num = PWM_PERIOD + 1;
    }
    pwm_txf[(channel * 5) + 1] = duty_num >> 8;
    pwm_txf[(channel * 5) + 2] = duty_num & 0x00FFU;
}

// phase 0~1.0
void phase_change(uint8_t channel, float phase)
{
    if (phase == 0.0f) {
        phase = 1.0f;
    }
    uint16_t phase_num =  (uint16_t)(phase * PWM_PERIOD);
    pwm_txf[(channel * 5) + 3] = phase_num >> 8;
    pwm_txf[(channel * 5) + 4] = phase_num & 0x00FFU;
}

void pwm_dev_addr(uint8_t channel, uint8_t addr)
{
    pwm_txf[(channel * 5) + 0] = addr;
}
