// Include MicroPython API.
#include "py/obj.h"
#include "py/objint.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mphal.h"

/* Use the following bme680 driver: https://github.com/BoschSensortec/BME68x-Sensor-API/blob/v4.4.6/bme68x.h */
#include "bme68x.h"
/* BSEC header files are available in the inc/ folder of the release package */
#include "bsec_interface.h"
#include "bsec_datatypes.h"

#if MICROPY_BSEC_DEBUG_VERBOSE // print debugging info
#define DEBUG_PRINT (1)
#define DEBUG_printf DEBUG_printf
#else // don't print debugging info
#define DEBUG_PRINT (0)
#define DEBUG_printf(...) (void)0
#endif


uint64_t mpz_to_64bit_int(mp_obj_int_t* arg, bool is_signed)
{
  //see mpz_as_int_checked
  uint64_t maxCalcThreshold = is_signed ? 140737488355327 : 281474976710655;

  mpz_t* z = &arg->mpz;
  if( !is_signed && z->neg )
  {
    mp_raise_TypeError(MP_ERROR_TEXT("Source integer must be unsigned"));
  }

  size_t len = z->len;
  uint64_t val = 0;

  while( len-- > 0 )
  {
    if( val > maxCalcThreshold )
    {
      mp_raise_TypeError(MP_ERROR_TEXT("Value too large for 64bit integer"));
    }
    val = ( val << MPZ_DIG_SIZE ) | z->dig[len];
  }

#ifdef _MSC_VER
#pragma warning( disable : 4146 )
#endif
  if( z->neg )
  {
    val = -val;
  }
#ifdef _MSC_VER
#pragma warning( default : 4146 )
#endif

  return val;
}


void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Error [%d] : Null pointer"), api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Error [%d] : Communication failure"), api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Error [%d] : Incorrect length parameter"), api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Error [%d] : Device not found"), api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Error [%d] : Self test error"), api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Warning [%d] : No new data found"), api_name, rslt);
            break;
        default:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BME680 Error [%d] : Unknown error code"), api_name, rslt);
            break;
    }
}

void _bsec_check_status(const char api_name[], bsec_library_return_t status)
{
  if (status != BSEC_OK) {
    if (status < BSEC_OK) {
      mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  BSEC Error [%d]"), api_name, status);
    } else {
      DEBUG_printf("API name [%s]  BSEC Warning [%d]\n", api_name, status);
    }
  }
}



// this is the actual C-structure for our new object
typedef struct _bsec_BME680_I2C_obj_t {
  mp_obj_base_t base;

  mp_obj_base_t *i2c_obj;
  uint8_t address;

  uint16_t sample_count;

  struct bme68x_dev bme;
  struct bme68x_conf conf;
  struct bme68x_heatr_conf heatr_conf;
  struct bme68x_data data;

  bsec_version_t version;
  uint64_t next_call_timestamp_ns;
  bsec_library_return_t status;
  uint8_t nSensorSettings;
  bsec_sensor_configuration_t virtualSensors[BSEC_NUMBER_OUTPUTS], sensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
  bsec_bme_settings_t bme680Settings;

  float iaq, rawTemperature, pressure, rawHumidity, gasResistance, stabStatus, runInStatus, temperature, humidity, staticIaq, co2Equivalent, breathVocEquivalent, compGasValue, gasPercentage;
  uint8_t iaqAccuracy, staticIaqAccuracy, co2Accuracy, breathVocAccuracy, compGasAccuracy, gasPercentageAcccuracy;

} bsec_BME680_I2C_obj_t;



void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
  mp_hal_delay_us(period);
}


BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  bsec_BME680_I2C_obj_t *self = intf_ptr;
  uint8_t dev_addr = self->address;

  DEBUG_printf("i2c_read(reg_addr=%x, reg_data=%x, len=%x, intf_ptr=%x)\n", reg_addr, reg_data, len, intf_ptr);

  // let's call self.i2c_obj.readfrom_mem() python method

  mp_obj_t args[2 + 3]; // 2 for calling a method on i2c_obj; + 3 normal args to the method
  mp_load_method(self->i2c_obj, MP_QSTR_readfrom_mem, args); // load 2 obj needed to call a method, on args[0] and args[1]
  // add pass our 3 args
  // :addr
  args[2] = MP_OBJ_NEW_SMALL_INT(dev_addr);
  // :memaddr
  args[3] = MP_OBJ_NEW_SMALL_INT(reg_addr);
  // :nbytes
  args[4] = mp_obj_new_int_from_uint(len);
  // finally, call
  mp_obj_t bytes_obj = mp_call_method_n_kw(3, 0, args); // 3 args, 0 kwargs

  // copy bytes_obj data to reg_data
  mp_buffer_info_t bufinfo;
  mp_get_buffer_raise(bytes_obj, &bufinfo, MP_BUFFER_READ);
  memcpy(reg_data, bufinfo.buf, len < bufinfo.len ? len : bufinfo.len);
  // TODO v2 with readfrom_mem_into, with a mp_obj_new_bytearray_by_ref(len, reg_data) passed, hopefully it just works

  #if DEBUG_PRINT
  for (int i=0; i<(len < bufinfo.len ? len : bufinfo.len); i++) {
    DEBUG_printf("%02x ", reg_data[i]);
    if ((i+1)%16 == 0) DEBUG_printf("\n");
  }
  DEBUG_printf("\n");
  #endif

  // TODO error handling?
  return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  bsec_BME680_I2C_obj_t *self = intf_ptr;
  uint8_t dev_addr = self->address;

  DEBUG_printf("i2c_write(reg_addr=%x, reg_data=%x, len=%x, intf_ptr=%x)\n", reg_addr, reg_data, len, intf_ptr);

  #if DEBUG_PRINT
  for (int i=0; i<len; i++) {
    DEBUG_printf("%02x ", reg_data[i]);
    if ((i+1)%16 == 0) DEBUG_printf("\n");
  }
  DEBUG_printf("\n");
  #endif

  // let's call self.i2c_obj.writeto_mem() python method
  mp_obj_t args[2 + 3]; // 2 for calling a method on i2c_obj; + 3 normal args to the method
  mp_load_method(self->i2c_obj, MP_QSTR_writeto_mem, args); // load 2 obj needed to call a method, on args[0] and args[1]
  // add pass our 3 args
  // :addr
  args[2] = MP_OBJ_NEW_SMALL_INT(dev_addr);
  // :memaddr
  args[3] = MP_OBJ_NEW_SMALL_INT(reg_addr);
  // :buf
  args[4] = mp_obj_new_bytes(reg_data, len);
  // finally, call
  mp_obj_t bytes_obj = mp_call_method_n_kw(3, 0, args); // 3 args, 0 kwargs

  // TODO error handling?
  return 0;
}



// just a definition
mp_obj_t bsec_BME680_I2C_make_new(const mp_obj_type_t *type,
                            size_t n_args,
                            size_t n_kw,
                            const mp_obj_t *args);
STATIC void bsec_BME680_I2C_print(const mp_print_t *print,
                            mp_obj_t self_in,
                            mp_print_kind_t kind) {
  (void)kind;
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  mp_printf(print, "<BME680_I2C i2c=%p address=%x>", self->i2c_obj, self->address);
}

/* methods start */

void _bsec_update_subscription(mp_obj_t self_in, bsec_virtual_sensor_t sensorList[], uint8_t nSensors, float sampleRate)
{
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);

  for (uint8_t i = 0; i < nSensors; i++)
  {
    for (uint8_t j = 0; j < BSEC_NUMBER_OUTPUTS; j++)
    {
      if (self->virtualSensors[j].sensor_id == sensorList[i])
      {
        self->virtualSensors[j].sample_rate = sampleRate;
      }
    }
  }

  self->nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;
  self->status = bsec_update_subscription(self->virtualSensors, BSEC_NUMBER_OUTPUTS, self->sensorSettings, &self->nSensorSettings);
  _bsec_check_status("bsec_update_subscription", self->status);

  return;
}

STATIC mp_obj_t bsec_BME680_I2C_init(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;

  // setup and disable every sensor
  self->virtualSensors[0].sensor_id = BSEC_OUTPUT_IAQ;
  self->virtualSensors[1].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
  self->virtualSensors[2].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
  self->virtualSensors[3].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
  self->virtualSensors[4].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
  self->virtualSensors[5].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
  self->virtualSensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
  self->virtualSensors[7].sensor_id = BSEC_OUTPUT_RAW_GAS;
  self->virtualSensors[8].sensor_id = BSEC_OUTPUT_STABILIZATION_STATUS;
  self->virtualSensors[9].sensor_id = BSEC_OUTPUT_RUN_IN_STATUS;
  self->virtualSensors[10].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
  self->virtualSensors[11].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
  self->virtualSensors[12].sensor_id = BSEC_OUTPUT_COMPENSATED_GAS;
  self->virtualSensors[13].sensor_id = BSEC_OUTPUT_GAS_PERCENTAGE;

  self->virtualSensors[0].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[1].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[2].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[3].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[4].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[5].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[6].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[7].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[8].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[9].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[10].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[11].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[12].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
  self->virtualSensors[13].sample_rate = BSEC_SAMPLE_RATE_DISABLED;

  self->status = bsec_init();
  _bsec_check_status("bsec_init", self->status);

  self->status = bsec_get_version(&self->version);
  _bsec_check_status("bsec_get_version", self->status);

  self->bme.read = bme68x_i2c_read;
  self->bme.write = bme68x_i2c_write;
  self->bme.intf = BME68X_I2C_INTF;
  self->bme.delay_us = bme68x_delay_us;
  self->bme.intf_ptr = self;
  self->bme.amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */

  rslt = bme68x_init(&self->bme);
  bme68x_check_rslt("bme68x_init", rslt);

  self->conf.filter = BME68X_FILTER_OFF;
  self->conf.odr = BME68X_ODR_NONE; /* This parameter defines the sleep duration after each profile */
  self->conf.os_hum = BME68X_OS_16X;
  self->conf.os_pres = BME68X_OS_1X;
  self->conf.os_temp = BME68X_OS_2X;
  rslt = bme68x_set_conf(&self->conf, &self->bme);
  bme68x_check_rslt("bme68x_set_conf", rslt);

  self->heatr_conf.enable = BME68X_ENABLE;
  // TODO maybe use custom profile from sequential_mode.c?
  self->heatr_conf.heatr_temp = 300;
  self->heatr_conf.heatr_dur = 100;
  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &self->heatr_conf, &self->bme);
  bme68x_check_rslt("bme68x_set_heatr_conf", rslt);


  // initalize some sensors
  // TODO expose that as python update_subscription()
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  _bsec_update_subscription(self_in, sensorList, 10, BSEC_SAMPLE_RATE_LP);

  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_init_obj, bsec_BME680_I2C_init);


STATIC mp_obj_t bsec_BME680_I2C_get_state(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);

  uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
  uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];
  uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;

  self->status = bsec_get_state(0,
                                bsecState,
                                BSEC_MAX_STATE_BLOB_SIZE,
                                workBuffer,
                                BSEC_MAX_STATE_BLOB_SIZE,
                                &n_serialized_state);
  _bsec_check_status("bsec_get_state", self->status);

  return mp_obj_new_bytes(bsecState, n_serialized_state);
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_get_state_obj, bsec_BME680_I2C_get_state);

STATIC mp_obj_t bsec_BME680_I2C_set_state(mp_obj_t self_in, mp_obj_t state_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);

  mp_buffer_info_t state_buf_info;
  mp_get_buffer_raise(state_in, &state_buf_info, MP_BUFFER_READ);
  if (state_buf_info.len > BSEC_MAX_STATE_BLOB_SIZE) {
    mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("Too large BSEC state buffer lenght: max expected=%d, actual=%d"), BSEC_MAX_STATE_BLOB_SIZE, state_buf_info.len);
  }

  uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];

  self->status = bsec_set_state(state_buf_info.buf,
                                state_buf_info.len,
                                workBuffer,
                                BSEC_MAX_STATE_BLOB_SIZE);
  _bsec_check_status("bsec_set_state", self->status);

  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(bsec_BME680_I2C_set_state_obj, bsec_BME680_I2C_set_state);


void bsec_zero_outputs(bsec_BME680_I2C_obj_t *self)
{
  self->temperature = 0.0f;
  self->pressure = 0.0f;
  self->humidity = 0.0f;
  self->gasResistance = 0.0f;
  self->rawTemperature = 0.0f;
  self->rawHumidity = 0.0f;
  self->stabStatus = 0.0f;
  self->runInStatus = 0.0f;
  self->iaq = 0.0f;
  self->iaqAccuracy = 0;
  self->staticIaq = 0.0f;
  self->staticIaqAccuracy = 0;
  self->co2Equivalent = 0.0f;
  self->co2Accuracy = 0;
  self->breathVocEquivalent = 0.0f;
  self->breathVocAccuracy = 0;
  self->compGasValue = 0.0f;
  self->compGasAccuracy = 0;
  self->gasPercentage = 0.0f;
  self->gasPercentageAcccuracy = 0;
}


STATIC mp_obj_t bsec_BME680_I2C_force_measurement(mp_obj_t self_in, mp_obj_t current_time_ns_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;

  int64_t current_timestamp_ns = mpz_to_64bit_int(MP_OBJ_TO_PTR(current_time_ns_in), false);

  bsec_virtual_sensor_t sensorList[0];
  _bsec_update_subscription(self_in, sensorList, 0, BSEC_SAMPLE_RATE_LP);

  // get settings from control
  self->status = bsec_sensor_control(current_timestamp_ns, &self->bme680Settings);
  self->next_call_timestamp_ns = self->bme680Settings.next_call;

  // apply settings
  self->conf.filter = BME68X_FILTER_OFF; // in arduino code it doesn't seem to be set, relying on default zero maybe?; still, BME680_FILTER_SEL is set, not sure why.
  self->conf.odr = BME68X_ODR_NONE; //TODOFIX? /* This parameter defines the sleep duration after each profile */
  self->conf.os_hum = self->bme680Settings.humidity_oversampling;
  self->conf.os_pres = self->bme680Settings.pressure_oversampling;
  self->conf.os_temp = self->bme680Settings.temperature_oversampling;
  rslt = bme68x_set_conf(&self->conf, &self->bme);
  bme68x_check_rslt("bme68x_set_conf", rslt);

  self->heatr_conf.enable = self->bme680Settings.run_gas;
  self->heatr_conf.heatr_temp = self->bme680Settings.heater_temperature;
  self->heatr_conf.heatr_dur = self->bme680Settings.heating_duration;
  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &self->heatr_conf, &self->bme);
  bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

  // finally force measurement
  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &self->bme);
  bme68x_check_rslt("bme68x_set_op_mode", rslt);

  //TODOFIX implement this after read_data, or not? https://github.com/BoschSensortec/BSEC-Arduino-library/blob/master/src/bsec.cpp#L244-L252

  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(bsec_BME680_I2C_force_measurement_obj, bsec_BME680_I2C_force_measurement);


STATIC mp_obj_t bsec_BME680_I2C_get_read_data_delay_us(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;

  /* Calculate delay period in microseconds */
  int delay_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &self->conf, &self->bme) + (self->heatr_conf.heatr_dur * 1000);

  return mp_obj_new_int(delay_us);
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_get_read_data_delay_us_obj, bsec_BME680_I2C_get_read_data_delay_us);


STATIC mp_obj_t bsec_BME680_I2C_read_data(mp_obj_t self_in, mp_obj_t measurement_time_ns_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;
  uint8_t n_fields;

  // read data
  rslt = bme68x_get_data(BME68X_FORCED_MODE, &self->data, &n_fields, &self->bme);
  bme68x_check_rslt("bme68x_get_data", rslt);

  if (! n_fields)
  {
    return mp_const_none;
  }

  self->sample_count++;

  /* mp_obj_t tuple[6] = { */
  /*   mp_obj_new_int(self->sample_count), */
  /*   mp_obj_new_float(self->data.temperature), */
  /*   mp_obj_new_float(self->data.pressure), */
  /*   mp_obj_new_float(self->data.humidity), */
  /*   mp_obj_new_float(self->data.gas_resistance), */
  /*   mp_obj_new_int(self->data.status), */
  /* }; */
  /* return mp_obj_new_tuple(6, tuple); */


  // process it with BSEC, finally!
  bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temperature, Pressure, Humidity & Gas Resistance */
  uint8_t nInputs = 0, nOutputs = 0;

  int64_t measurement_timestamp_ns = mpz_to_64bit_int(MP_OBJ_TO_PTR(measurement_time_ns_in), false);

  if (self->data.status & BME68X_NEW_DATA_MSK)
  {
    if (self->bme680Settings.process_data & BSEC_PROCESS_TEMPERATURE)
    {
      inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
      inputs[nInputs].signal = self->data.temperature;
      inputs[nInputs].time_stamp = measurement_timestamp_ns;
      nInputs++;

      /* Temperature offset from the real temperature due to external heat sources */
      inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
      inputs[nInputs].signal = 4.6;
      inputs[nInputs].time_stamp = measurement_timestamp_ns;
      nInputs++;
    }
    if (self->bme680Settings.process_data & BSEC_PROCESS_HUMIDITY)
    {
      inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
      inputs[nInputs].signal = self->data.humidity;
      inputs[nInputs].time_stamp = measurement_timestamp_ns;
      nInputs++;
    }
    if (self->bme680Settings.process_data & BSEC_PROCESS_PRESSURE)
    {
      inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
      inputs[nInputs].signal = self->data.pressure;
      inputs[nInputs].time_stamp = measurement_timestamp_ns;
      nInputs++;
    }
    if (self->bme680Settings.process_data & BSEC_PROCESS_GAS)
    {
      inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
      inputs[nInputs].signal = self->data.gas_resistance;
      inputs[nInputs].time_stamp = measurement_timestamp_ns;
      nInputs++;
    }
  }

  if (! nInputs)
  {
    return mp_const_none;
  }

  nOutputs = BSEC_NUMBER_OUTPUTS;
  bsec_output_t _outputs[BSEC_NUMBER_OUTPUTS];

  self->status = bsec_do_steps(inputs, nInputs, _outputs, &nOutputs);
  _bsec_check_status("bsec_do_steps", self->status);

  bsec_zero_outputs(self);

  mp_obj_t ret_val = mp_obj_new_dict(0);

  if (! nOutputs)
  {
    return mp_const_none;
  }

  // TODO expose timestamps, proper timestmap next_call instead of next_call_delay too
  int64_t outputTimestamp = _outputs[0].time_stamp;

  for (uint8_t i = 0; i < nOutputs; i++)
  {
    switch (_outputs[i].sensor_id)
    {
      case BSEC_OUTPUT_IAQ:
        self->iaq = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("iaq", 3), mp_obj_new_float(self->iaq));
        self->iaqAccuracy = _outputs[i].accuracy;
        mp_obj_dict_store(ret_val, mp_obj_new_str("iaqAccuracy", 11), mp_obj_new_int(self->iaqAccuracy));
        break;
      case BSEC_OUTPUT_STATIC_IAQ:
        self->staticIaq = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("staticIaq", 9), mp_obj_new_float(self->staticIaq));
        self->staticIaqAccuracy = _outputs[i].accuracy;
        mp_obj_dict_store(ret_val, mp_obj_new_str("staticIaqAccuracy", 17), mp_obj_new_int(self->staticIaqAccuracy));
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        self->co2Equivalent = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("co2Equivalent", 13), mp_obj_new_float(self->co2Equivalent));
        self->co2Accuracy = _outputs[i].accuracy;
        mp_obj_dict_store(ret_val, mp_obj_new_str("co2Accuracy", 11), mp_obj_new_int(self->co2Accuracy));
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        self->breathVocEquivalent = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("breathVocEquivalent", 19), mp_obj_new_float(self->breathVocEquivalent));
        self->breathVocAccuracy = _outputs[i].accuracy;
        mp_obj_dict_store(ret_val, mp_obj_new_str("breathVocAccuracy", 17), mp_obj_new_int(self->breathVocAccuracy));
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        self->rawTemperature = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("rawTemperature", 14), mp_obj_new_float(self->rawTemperature));
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        self->pressure = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("pressure", 8), mp_obj_new_float(self->pressure));
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
        self->rawHumidity = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("rawHumidity", 11), mp_obj_new_float(self->rawHumidity));
        break;
      case BSEC_OUTPUT_RAW_GAS:
        self->gasResistance = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("gasResistance", 13), mp_obj_new_float(self->gasResistance));
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        self->stabStatus = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("stabStatus", 10), mp_obj_new_float(self->stabStatus));
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        self->runInStatus = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("runInStatus", 11), mp_obj_new_float(self->runInStatus));
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        self->temperature = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("temperature", 11), mp_obj_new_float(self->temperature));
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        self->humidity = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("humidity", 8), mp_obj_new_float(self->humidity));
        break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
        self->compGasValue = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("compGasValue", 12), mp_obj_new_float(self->compGasValue));
        self->compGasAccuracy = _outputs[i].accuracy;
        mp_obj_dict_store(ret_val, mp_obj_new_str("compGasAccuracy", 15), mp_obj_new_int(self->compGasAccuracy));
        break;
      case BSEC_OUTPUT_GAS_PERCENTAGE:
        self->gasPercentage = _outputs[i].signal;
        mp_obj_dict_store(ret_val, mp_obj_new_str("gasPercentage", 13), mp_obj_new_float(self->gasPercentage));
        self->gasPercentageAcccuracy = _outputs[i].accuracy;
        mp_obj_dict_store(ret_val, mp_obj_new_str("gasPercentageAcccuracy", 22), mp_obj_new_int(self->gasPercentageAcccuracy));
        break;
      default:
        break;
    }
  }

  return ret_val;
}
MP_DEFINE_CONST_FUN_OBJ_2(bsec_BME680_I2C_read_data_obj, bsec_BME680_I2C_read_data);



STATIC mp_obj_t bsec_BME680_I2C_get_next_call_timestamp_ns(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);

  return mp_obj_new_int_from_ull(self->next_call_timestamp_ns);
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_get_next_call_timestamp_ns_obj, bsec_BME680_I2C_get_next_call_timestamp_ns);

STATIC const mp_rom_map_elem_t bsec_BME680_I2C_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&bsec_BME680_I2C_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&bsec_BME680_I2C_get_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_state), MP_ROM_PTR(&bsec_BME680_I2C_set_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_measurement), MP_ROM_PTR(&bsec_BME680_I2C_force_measurement_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_read_data_delay_us), MP_ROM_PTR(&bsec_BME680_I2C_get_read_data_delay_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_data), MP_ROM_PTR(&bsec_BME680_I2C_read_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_next_call_timestamp_ns), MP_ROM_PTR(&bsec_BME680_I2C_get_next_call_timestamp_ns_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bsec_BME680_I2C_locals_dict, bsec_BME680_I2C_locals_dict_table);
/* methods end */


/* BME680_I2C class */
const mp_obj_type_t bsec_BME680_I2C_type = {
  { &mp_type_type },
  .name = MP_QSTR_BME680_I2C,
  .print = bsec_BME680_I2C_print,
  .make_new = bsec_BME680_I2C_make_new,
  .locals_dict = (mp_obj_dict_t*)&bsec_BME680_I2C_locals_dict,
};


mp_obj_t bsec_BME680_I2C_make_new(const mp_obj_type_t *type,
                            size_t n_args,
                            size_t n_kw,
                            const mp_obj_t *all_args ) {
    enum {
        ARG_i2c, ARG_address
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_address, MP_ARG_INT, {.u_int = BME68X_I2C_ADDR_HIGH} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // create new object
    bsec_BME680_I2C_obj_t *self = m_new_obj(bsec_BME680_I2C_obj_t);
    self->base.type = &bsec_BME680_I2C_type;

    // set parameters
    mp_obj_base_t *i2c_obj = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[ARG_i2c].u_obj);
    self->i2c_obj = i2c_obj;
    self->address = args[ARG_address].u_int;

    self->sample_count = 0;

    self->version.major = 0;
    self->version.minor = 0;
    self->version.major_bugfix = 0;
    self->version.minor_bugfix = 0;
    self->next_call_timestamp_ns = 0;
    self->status = BSEC_OK;
    self->nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;

    bsec_zero_outputs(self);

    return MP_OBJ_FROM_PTR(self);
}


// Define all properties of the module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t bsec_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_bsec) },
    { MP_ROM_QSTR(MP_QSTR_BME680_I2C), (mp_obj_t)&bsec_BME680_I2C_type },
};
STATIC MP_DEFINE_CONST_DICT(bsec_module_globals, bsec_module_globals_table);

// Define module object.
const mp_obj_module_t bsec_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bsec_module_globals,
};

// Register the module to make it available in Python.
// Note: the "1" in the third argument means this module is always enabled.
// This "1" can be optionally replaced with a macro like MODULE_BSEC_ENABLED
// which can then be used to conditionally enable this module.
MP_REGISTER_MODULE(MP_QSTR_bsec, bsec_user_cmodule, 1);
