#include <stdio.h>
#include <unistd.h>
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"
#include <bsp.h>
#include <sysctl.h>
#include <syslog.h>
#include <timer.h>
#include <plic.h>

int main(void)
{
    // set cpu freq to 540Mhz
    sysctl_pll_set_freq(SYSCTL_PLL0, 1100000000);
    uint64_t core = current_coreid();
    printf("Core %ld Hello world\n", core);

    printf("Core freq: %d\n", sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));

    /* Clear stdin buffer before scanf */
    sys_stdin_flush();

    while(1);
}
