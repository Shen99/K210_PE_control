#ifndef CONTROL_H
#define CONTROL_H

#define KHZ (1000)
#define MHZ (1000000)
#define GHZ (1000000000)

#define CONTROL_FREQ (20*KHZ)
#define CONTROL_PERIOD (1.0f/CONTROL_FREQ)

int control_loop(void *ctx);
#endif