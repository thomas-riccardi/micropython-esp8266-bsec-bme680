BSEC_MOD_DIR := $(USERMOD_DIR)

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(BSEC_MOD_DIR)/bsecmodule.c
SRC_USERMOD += $(BSEC_MOD_DIR)/API/bme68x.c

CFLAGS_USERMOD += -Wno-unused-but-set-variable -Wno-unused-variable

CFLAGS_USERMOD += -I$(BSEC_MOD_DIR)
CFLAGS_USERMOD += -I$(BSEC_MOD_DIR)/API
CFLAGS_USERMOD += -I$(BSEC_MOD_DIR)/algo/bin

# replace that by a copy, adapted to write to micropython I2C
#SRC_USERMOD += API/example/bsec_integration.c

CFLAGS_USERMOD += -L$(BSEC_MOD_DIR)/algo/bin -lalgobsec
#CFLAGS_USERMOD += -lm -lrt
