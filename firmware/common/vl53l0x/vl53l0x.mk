VL53L0XSRC := \
              $(USER_LIBS)/vl53l0x/platform/src/vl53l0x_platform.c \
              $(USER_LIBS)/vl53l0x/platform/src/vl53l0x_i2c_platform.c \
              $(USER_LIBS)/vl53l0x/core/src/vl53l0x_api.c \
              $(USER_LIBS)/vl53l0x/core/src/vl53l0x_api_calibration.c \
              $(USER_LIBS)/vl53l0x/core/src/vl53l0x_api_core.c \
              $(USER_LIBS)/vl53l0x/core/src/vl53l0x_api_ranging.c \
              $(USER_LIBS)/vl53l0x/core/src/vl53l0x_api_strings.c 

              #$(USER_LIBS)/vl53l0x/platform/src/vl53l0x_platform_log.c

VL53L0XINC := $(USER_LIBS)/vl53l0x/platform/inc \
              $(USER_LIBS)/vl53l0x/core/inc
