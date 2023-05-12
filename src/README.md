The `F_CLK`, `F_CS`, `F_DO`, ... pins is SPI3 for flash as default.

`IO_16` as Boot select, when power up, if it is high, boot from flash, if low,
enter into ISP. After reset, this pin can be used as other normal IO.

`IO_0`, `IO_1`, `IO_2`, `IO_3` is JTAG as default after reset.

`IO_4`, `IO_5` is ISP also UART.

`RST(RESET)` for system reset.

when use standard SPI, MOSI is `D0`, MISO is `D1`.