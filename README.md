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
# patch micropython, cf https://github.com/devbis/st7789_mpy/#overflow-of-iram1_0_seg
git apply ../0001-fix-iram1-overflow-move-bsec-text-to-irom0.patch
```

### Build
```shell
cd micropython

# build port (~30s)
cd ports/esp8266
#esp-sdk make clean
esp-sdk make USER_C_MODULES=../../../modules
```
Result:
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

More build options:
- append `MICROPY_BSEC_DEBUG_VERBOSE=1` to the `make` command to get debug logs of the BSEC module
## Use
Follow flash doc:
https://github.com/micropython/micropython/blob/master/ports/esp8266/README.md#build-instructions

## Example
```python
import machine
import time
import bsec

i2c = machine.I2C(scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
bme680 = bsec.BME680_I2C(i2c)
bme680.init()

while True:
    measurement_timestamp_ns = time.time_ns()
    bme680.force_measurement(measurement_timestamp_ns)
    delay_us = bme680.get_read_data_delay_us()
    time.sleep_us(delay_us)
    data = bme680.read_data(measurement_timestamp_ns)
    print(data)
    next_call_timestamp_ns = bme680.get_next_call_timestamp_ns()
    delay_us = max(next_call_timestamp_ns - time.time_ns(), 0) // 1000
    print("sleep (in s)", delay_us/1000000)
    time.sleep_us(delay_us)

# example:
# {'iaq': 144.758, 'staticIaqAccuracy': 3, 'gasResistance': 171795.0, 'rawHumidity': 49.9882, 'co2Accuracy': 3, 'breathVocAccuracy': 3, 'staticIaq': 103.249, 'iaqAccuracy': 3, 'breathVocEquivalent': 1.63182, 'co2Equivalent': 1032.49, 'humidity': 65.8691, 'pressure': 100328.0, 'temperature': 24.1201, 'rawTemperature': 28.7868}

# After iaqAccuracy reaches 3, i.e. calibrated
# Save state
calibrated_state = bme680.get_state()
# Save to persistent memory
state_filename = 'bsec_bme680_calibrated_state.bin'
with open(state_filename, 'wb') as f:
    wrote = f.write(calibrated_state)
print('wrote', wrote, 'bytes on file', state_filename)

# Load from persistent memory
with open(state_filename, 'rb') as f:
    calibrated_state = f.read()
print('read', len(calibrated_state), 'bytes on file', state_filename)
# Restore
bme680.set_state(calibrated_state)
```

## Calibrate the BME680 and BSEC
(see `BSEC_1.4.8.0_Generic_Release/integration_guide/BST-BME680-Integration-Guide-AN008-48.pdf`)
- expose the sensor once to good air (e.g. outdoor air) and bad air (e.g. box with exhaled breath) for auto-trimming.
- wait for `accuracy` to reach `3`
- save calibration state:
  - connect with `screen`
  - CTRL-C
  - `sensors_bme680_save_state(sensor_bme680)`
  - reset board

# Development
## Generate stub micropython C module
```shell
pip install git+https://github.com/pazzarpj/micropython-ustubby.git
cd modules
python -c 'import ustubby; import bsec; print(ustubby.stub_module(bsec))'
```
