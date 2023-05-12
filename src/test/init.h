#ifndef INIT_H
#define INIT_H

#include <stdio.h>
#include "sysctl.h"
#include "fpioa.h"
#include "gpio.h"
#include "gpiohs.h"
#include "spi.h"

void io_set_power(void);
void AD7606_init(void);

#endif