# Hardware
- [adafruit feather HUZZAH board](https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/overview)
- [BME680 temperature, humidity, barometric pressure and VOC gas sensor](https://www.adafruit.com/product/3660/)
- [DS18B20 Digital temperature sensor](https://www.adafruit.com/product/374)

# Software
- `micropython/` (https://github.com/micropython/micropython.git)
- `modules/bsec/`
  - http://docs.micropython.org/en/latest/develop/cmodules.html
  - https://www.bosch-sensortec.com/software-tools/software/bsec/
  - https://github.com/BoschSensortec/BME68x-Sensor-API

## Build
https://github.com/micropython/micropython/blob/master/ports/esp8266/README.md#build-instructions

### Dependencies
Use `esp-open-sdk` in docker, from https://github.com/larsks/docker-image-esp-open-sdk

Use our own patch on top:
```shell
cd esp-open-sdk-for-micropython
make

# install 'esp-sdk' wrapper that forwards calls to docker, it mounts full $HOME
docker run --rm -u $UID -v $HOME/bin:/target esp-open-sdk install-wrapper
# usage
https://github.com/larsks/docker-image-esp-open-sdk#wrapper-script
```

### Prepare
```shell
cd micropython
# get submodules for esp8266 port
esp-sdk make -C ports/esp8266 submodules
# build micropython cross compiler (<1min)
esp-sdk make -C mpy-cross
```

### Build
```shell
cd micropython

# build port (~30s)
cd ports/esp8266
#esp-sdk make clean
esp-sdk make USER_C_MODULES=../../../modules
```
result:
```
Use make V=1 or set BUILD_VERBOSE in your environment to increase build verbosity.
Including User C Module from ../../../modules/bsec
...
LINK build-GENERIC/firmware.elf
   text    data     bss     dec     hex filename
 621600    1012   66384  688996   a8364 build-GENERIC/firmware.elf
Create build-GENERIC/firmware-combined.bin
esptool.py v1.2
flash     32912
 .text    30776 at 0x40100000
 .data    1012 at 0x3ffe8000
 .rodata  1080 at 0x3ffe8400
padding   3952
irom0text 589744
total     626608
md5       022d3e9e5131954dc098f181cf56b3bc
```


## Use
Follow flash doc:
https://github.com/micropython/micropython/blob/master/ports/esp8266/README.md#build-instructions
