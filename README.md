# K210 playground

## configuration

you need place the [toolchian](https://github.com/kendryte/kendryte-gnu-toolchain/releases) to `../`, this is the file tree looks like:

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

set `com` variable in `build.py` which is the serial port for the K210.

```python
# you need change this to your real port
com = "COM7"
```

## build and flash

just `python build.py`
