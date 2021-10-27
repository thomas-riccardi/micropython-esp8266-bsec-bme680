// Include MicroPython API.
#include "py/runtime.h"

/* Use the following bme680 driver: https://github.com/BoschSensortec/BME68x-Sensor-API/blob/v4.4.6/bme68x.h */
#include "bme68x.h"
/* BSEC header files are available in the inc/ folder of the release package */
#include "bsec_interface.h"
#include "bsec_datatypes.h"


// This is the function which will be called from Python as cbsec.add_ints(a, b).
STATIC mp_obj_t bsec_add_ints(mp_obj_t a_obj, mp_obj_t b_obj) {
    // Extract the ints from the micropython input objects.
    int a = mp_obj_get_int(a_obj);
    int b = mp_obj_get_int(b_obj);

    struct bme68x_dev bme;
    int8_t rslt;
    rslt = bme68x_init(&bme);


    // Calculate the addition and convert to MicroPython object.
    return mp_obj_new_int(a + b);
}
// Define a Python reference to the function above.
STATIC MP_DEFINE_CONST_FUN_OBJ_2(bsec_add_ints_obj, bsec_add_ints);

// Define all properties of the module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t bsec_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cbsec) },
    { MP_ROM_QSTR(MP_QSTR_add_ints), MP_ROM_PTR(&bsec_add_ints_obj) },
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
MP_REGISTER_MODULE(MP_QSTR_cbsec, bsec_user_cmodule, 1);
