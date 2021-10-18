BSEC_MOD_DIR := $(USERMOD_DIR)

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(BSEC_MOD_DIR)/bsecmodule.c
SRC_USERMOD += $(BSEC_MOD_DIR)/API/bme68x.c

CFLAGS_USERMOD += -Wno-unused-but-set-variable -Wno-unused-variable

CFLAGS_USERMOD += -I$(BSEC_MOD_DIR)
CFLAGS_USERMOD += -I$(BSEC_MOD_DIR)/API
CFLAGS_USERMOD += -I$(BSEC_MOD_DIR)/algo/bin

# Link with libalgobsec.a from Bosch
LDFLAGS_USERMOD += -L$(BSEC_MOD_DIR)/algo/bin -lalgobsec
# libalgobsec.a requires both C99 math functions such as fminf, not present in micropython/lib/libm/, as well as double precision math functions such as pow
# => link with compiler libm.a to solve these few missing symbols, the rest of math.h still comes from micropython
# next line doesn't work, probably because this file is loaded too early; so hardcode the path instead, assuming build from docker with esp-sdk wrapper; adapt according to your local toolchain setup
#LDFLAGS_USERMOD += -L$(ESP_SDK)/../lib -lm
LDFLAGS_USERMOD += -L/tools/xtensa-lx106-elf/xtensa-lx106-elf/sysroot/lib -lm
