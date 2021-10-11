// Include MicroPython API.
#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"

/* Use the following bme680 driver: https://github.com/BoschSensortec/BME68x-Sensor-API/blob/v4.4.6/bme68x.h */
#include "bme68x.h"
/* BSEC header files are available in the inc/ folder of the release package */
#include "bsec_interface.h"
#include "bsec_datatypes.h"


void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Error [%d] : Null pointer"), api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Error [%d] : Communication failure"), api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Error [%d] : Incorrect length parameter"), api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Error [%d] : Device not found"), api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Error [%d] : Self test error"), api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Warning [%d] : No new data found"), api_name, rslt);
            break;
        default:
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("API name [%s]  Error [%d] : Unknown error code"), api_name, rslt);
            break;
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
} bsec_BME680_I2C_obj_t;


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

STATIC mp_obj_t bsec_BME680_I2C_init(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;

  rslt = bme68x_init(&self->bme);
  bme68x_check_rslt("bme68x_init", rslt);


  self->conf.filter = BME68X_FILTER_OFF;
  self->conf.odr = BME68X_ODR_NONE;
  self->conf.os_hum = BME68X_OS_16X;
  self->conf.os_pres = BME68X_OS_1X;
  self->conf.os_temp = BME68X_OS_2X;
  rslt = bme68x_set_conf(&self->conf, &self->bme);
  bme68x_check_rslt("bme68x_set_conf", rslt);

  self->heatr_conf.enable = BME68X_ENABLE;
  self->heatr_conf.heatr_temp = 300;
  self->heatr_conf.heatr_dur = 100;
  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &self->heatr_conf, &self->bme);
  bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_init_obj, bsec_BME680_I2C_init);


STATIC mp_obj_t bsec_BME680_I2C_force_measurement(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;

  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &self->bme);
  bme68x_check_rslt("bme68x_set_op_mode", rslt);

  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_force_measurement_obj, bsec_BME680_I2C_force_measurement);


STATIC mp_obj_t bsec_BME680_I2C_get_delay_us(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;

  /* Calculate delay period in microseconds */
  int delay_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &self->conf, &self->bme) + (self->heatr_conf.heatr_dur * 1000);

  return mp_obj_new_int(delay_us);
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_get_delay_us_obj, bsec_BME680_I2C_get_delay_us);


STATIC mp_obj_t bsec_BME680_I2C_read_data(mp_obj_t self_in) {
  bsec_BME680_I2C_obj_t *self = MP_OBJ_TO_PTR(self_in);
  int8_t rslt;
  uint8_t n_fields;

  rslt = bme68x_get_data(BME68X_FORCED_MODE, &self->data, &n_fields, &self->bme);
  bme68x_check_rslt("bme68x_get_data", rslt);

  if (! n_fields)
  {
    return mp_const_none;
  }

  self->sample_count++;

  mp_obj_t tuple[6] = {
    mp_obj_new_int(self->sample_count),
    mp_obj_new_float(self->data.temperature),
    mp_obj_new_float(self->data.pressure),
    mp_obj_new_float(self->data.humidity),
    mp_obj_new_float(self->data.gas_resistance),
    mp_obj_new_int(self->data.status),
  };
  return mp_obj_new_tuple(6, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_1(bsec_BME680_I2C_read_data_obj, bsec_BME680_I2C_read_data);


STATIC const mp_rom_map_elem_t bsec_BME680_I2C_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&bsec_BME680_I2C_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_measurement), MP_ROM_PTR(&bsec_BME680_I2C_force_measurement_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_delay_us), MP_ROM_PTR(&bsec_BME680_I2C_get_delay_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_data), MP_ROM_PTR(&bsec_BME680_I2C_read_data_obj) },
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
    self->sample_count = 0;
    mp_obj_base_t *i2c_obj = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[ARG_i2c].u_obj);
    self->i2c_obj = i2c_obj;
    self->address = args[ARG_address].u_int;

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
