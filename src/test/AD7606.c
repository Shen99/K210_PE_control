#include <stdio.h>
#include "sysctl.h"
#include "bsp.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"
#include "spi.h"
#include "timer.h"

#include "AD7606.h"

#if AD7606_USE_DMA
    // use double buf to prevent data integrity
    volatile uint32_t AD7606_rx_buf[NUM_OF_AD7606_CHANNEL];
    volatile double AD7606_buf[NUM_OF_AD7606_CHANNEL];
    volatile uint8_t AD7606_trans_complete = false;
    volatile uint8_t AD7606_buf_ready = false;
    volatile uint8_t AD7606_dma_current = AD7606_RX0_DMA_CHANNEL;

    int AD7606_transfer_done(void *ctx);

    spi_data_t AD7606_0_data = (spi_data_t)
    {
        .rx_channel = AD7606_RX0_DMA_CHANNEL,
        .rx_buf = AD7606_rx_buf,
        .rx_len = AD7606_DMA_TRANS_COUNT,
        .transfer_mode = SPI_TMOD_RECV
    };
    plic_interrupt_t AD7606_0_irq = (plic_interrupt_t)
    {
        .callback = AD7606_transfer_done,
        .ctx = NULL,
        .priority = 1
    };

    spi_data_t AD7606_1_data = (spi_data_t)
    {
        .rx_channel = AD7606_RX1_DMA_CHANNEL,
        .rx_buf = (AD7606_rx_buf + 8),
        .rx_len = AD7606_DMA_TRANS_COUNT,
        .transfer_mode = SPI_TMOD_RECV
    };

    int AD7606_transfer_done(void *ctx)
    {
        //if you need two piece of AD7606,you need to cancel the comment
      
        if (AD7606_dma_current == AD7606_RX0_DMA_CHANNEL)
        {
            gpiohs_set_pin(AD7606_CS_GPIO_NUM, GPIO_PV_HIGH);
            AD7606_dma_current = AD7606_RX1_DMA_CHANNEL;
            gpiohs_set_pin(AD7606_CS_1_GPIO_NUM, GPIO_PV_LOW);
            AD7606_spi_dma_transfer(AD7606_SPI, AD7606_SPI_0_CHIP_SELECT, AD7606_1_data, &AD7606_0_irq);
        } else {
           
            gpiohs_set_pin(AD7606_CS_1_GPIO_NUM, GPIO_PV_HIGH);
            AD7606_buf_ready = false;
            for (int i = 0; i < NUM_OF_AD7606_CHANNEL; i++)
            {
                AD7606_buf[i] = (int16_t)(AD7606_rx_buf[i])/(float)32768.0f * 10000;  //real is 10
            }
            AD7606_buf_ready = true;
            AD7606_trans_complete = true;
            return 0;
        }
        return 0;
    }

    int AD7606_data_retrieve(void *ctx)
    {
        AD7606_dma_current = AD7606_RX0_DMA_CHANNEL;
        gpiohs_set_pin(AD7606_CS_GPIO_NUM, GPIO_PV_LOW);
        AD7606_spi_dma_transfer(AD7606_SPI, AD7606_SPI_0_CHIP_SELECT, AD7606_0_data, &AD7606_0_irq);
        return 0;
    }

    int AD7606_trggier_non_blocking(void *ctx)
    {
        AD7606_trans_complete = false;
        gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_LOW);
        asm volatile("nop \n nop \n nop \n nop \n nop");
        gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_HIGH);
        return 0;
    }

    void AD7606_trggier(void)
    {
        AD7606_trggier_non_blocking(NULL);
        while(!AD7606_trans_complete) {;}
    }
#else
    int16_t AD7606_rx_buf[NUM_OF_AD7606_CHANNEL];
    volatile uint8_t AD7606_trans_complete = false;
    void AD7606_trggier(void)
    {
        AD7606_trans_complete = false;
        gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_LOW);
        asm volatile("nop \n nop \n nop \n nop \n nop");
        gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_HIGH);
        while(!AD7606_trans_complete) {;}
    }

    int AD7606_data_retrieve(void *ctx)
    {
        gpiohs_set_pin(AD7606_CS_GPIO_NUM, GPIO_PV_LOW);
        spi_receive_data_standard(AD7606_SPI, SPI_CHIP_SELECT_0, NULL, 0, (uint8_t *)AD7606_rx_buf, 16);
        gpiohs_set_pin(AD7606_CS_GPIO_NUM, GPIO_PV_HIGH);
        gpiohs_set_pin(AD7606_CS_1_GPIO_NUM, GPIO_PV_LOW);
        spi_receive_data_standard(AD7606_SPI, SPI_CHIP_SELECT_0, NULL, 0, (uint8_t *)(AD7606_rx_buf+8), 16);
        gpiohs_set_pin(AD7606_CS_1_GPIO_NUM, GPIO_PV_HIGH);
        AD7606_trans_complete = true;
        return 0;
    }
#endif

void AD7606_init(void)
{
    // initialize the AD7606 rx_buf
    for (int i = 0; i < NUM_OF_AD7606_CHANNEL; i++)
    {
        AD7606_rx_buf[i] = 0;
    }

    fpioa_set_function(AD7606_SCLK, AD7606_SCLK_FUNC);
    fpioa_set_function(AD7606_CS, AD7606_CS_FUNC);
    fpioa_set_function(AD7606_MISO, AD7606_MISO_FUNC);
    fpioa_set_function(AD7606_CONVAB, AD7606_CONVAB_FUNC);
    fpioa_set_function(AD7606_BUSY, AD7606_BUSY_FUNC);
    fpioa_set_function(AD7606_RESET, AD7606_RESET_FUNC);

    gpiohs_set_drive_mode(AD7606_CONVAB_GPIO_NUM, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(AD7606_BUSY_GPIO_NUM, GPIO_DM_INPUT_PULL_UP);
    gpiohs_set_drive_mode(AD7606_RESET_GPIO_NUM, GPIO_DM_OUTPUT);

    fpioa_set_function(AD7606_CS_1, AD7606_CS_1_FUNC);

    gpiohs_set_drive_mode(AD7606_CS_GPIO_NUM, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(AD7606_CS_1_GPIO_NUM, GPIO_DM_OUTPUT);
    gpiohs_set_pin(AD7606_CS_GPIO_NUM, GPIO_PV_HIGH);
    gpiohs_set_pin(AD7606_CS_1_GPIO_NUM, GPIO_PV_HIGH);

    gpiohs_set_pin_edge(AD7606_BUSY_GPIO_NUM, GPIO_PE_FALLING);
    gpiohs_irq_register(AD7606_BUSY_GPIO_NUM, 1, AD7606_data_retrieve, NULL);

    spi_init(AD7606_SPI, AD7606_SPI_MODE, AD7606_SPI_FORMAT, 16, 0);
    uint32_t spi_rate = spi_set_clk_rate(AD7606_SPI, AD7606_SPI_RATE);
    printf("AD7606 spi freq: %u\n", spi_rate);

    gpiohs_set_pin(AD7606_CONVAB_GPIO_NUM, GPIO_PV_HIGH);
    // at least 50ns
    gpiohs_set_pin(AD7606_RESET_GPIO_NUM, GPIO_PV_LOW);
    msleep(1);
    gpiohs_set_pin(AD7606_RESET_GPIO_NUM, GPIO_PV_HIGH);
    msleep(1);
    gpiohs_set_pin(AD7606_RESET_GPIO_NUM, GPIO_PV_LOW);

    AD7606_spi_dma_init(AD7606_SPI, AD7606_0_data, &AD7606_0_irq);
    AD7606_spi_dma_init(AD7606_SPI, AD7606_1_data, &AD7606_0_irq);
}
