# Firmware for the Nucleo32 F303K8

**Be Carefull**, some hardware modifications are needed.

In order to keep compatibility with Arduino Nano, ST has wired `PB6` and `PB7` to `PA6` and `PA5` in order to get *i2c* on Arduino pins `A4` and `A5`.

So the solder bridges `SB16` and `SB18` **have to be removed**.
