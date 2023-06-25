# K210 playground

## configuration

Clone the repo and initialize it.

```shell
mkdir K210
cd K210
git clone https://github.com/vertexi/K210_power_elec.git
cd .\K210_power_elec\
git submodule update --init --recursive
```

you need place the [toolchian](https://github.com/kendryte/kendryte-gnu-toolchain/releases) to the parent folder of this repo directory, this is the file tree looks like:

```
├── K210_power_elec
│   ├── README.md
│   ├── build
│   ├── build.py
│   ├── kendryte-standalone-sdk
│   └── src
└── kendryte-toolchain
    ├── bin
    ├── include
    ├── lib
    ├── libexec
    ├── riscv64-unknown-elf
    └── share
```

Install the [kflash](https://github.com/kendryte/kflash.py) python package for flash firmware.

```shell
pip install kflash
pip install tabulate
```

set `com` variable in `build.py` which is the serial port for the K210.

## build and flash

if you use a "USB-SERIAL CH340" serial device just `python build.py` maybe ok.

`python build.py -c <COM_PORT>` to specific a serial device to load program, or `python build.py -l` to list serial devices.
