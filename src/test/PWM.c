#include "pin_config.h"
#include "PWM.h"

#define PWM_CS_LOW()    gpiohs_set_pin(PWM_CS_GPIO_NUM, GPIO_PV_LOW)
#define PWM_CS_HIGH()   gpiohs_set_pin(PWM_CS_GPIO_NUM, GPIO_PV_HIGH)

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
}


void spi_send_data(uint8_t *data, uint32_t len)
{
    spi_send_data_standard(SPI_DEVICE_1, PWM_SPI_CHIP_SELECT, NULL, 0, (const uint8_t *)data, len);
}

// duty 0 ~ 1.0
void duty_change(uint8_t *pwm_buf, uint8_t channel, float duty)
{
    uint16_t duty_num =  (uint16_t)(duty * PWM_PERIOD);
    if (duty == 0.0f)
    {
        duty_num = PWM_PERIOD + 1;
    }
    pwm_buf[(channel * 5) + 1] = duty_num >> 8;
    pwm_buf[(channel * 5) + 2] = duty_num & 0x00FFU;
}

// phase 0~1.0
void phase_change(uint8_t *pwm_buf, uint8_t channel, float phase)
{
    if (phase == 0.0f) {
        phase = 1.0f;
    }
    uint16_t phase_num =  (uint16_t)(phase * PWM_PERIOD);
    pwm_buf[(channel * 5) + 3] = phase_num >> 8;
    pwm_buf[(channel * 5) + 4] = phase_num & 0x00FFU;
}

void pwm_dev_addr(uint8_t *pwm_buf, uint8_t channel, uint8_t addr)
{
    pwm_buf[(channel * 5) + 0] = addr;
}
