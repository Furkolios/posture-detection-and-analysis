# Phase 2.5 Makefile.
#
# HOST TARGETS (default — same as Phase 2):
#   make host             - host firmware loop with dummy generator
#   make test             - Phase 1 C harness
#   make test2            - Phase 2 C harness
#   make pytest           - Phase 1 Python validator
#   make pytest2          - Phase 2 Python validator
#
# DEVICE TARGET:
#   make TARGET=msp432    - cross-compile firmware.elf and firmware.hex
#                           for the MSP432P401R LaunchPad.
#
# REQUIREMENTS for TARGET=msp432:
#   - arm-none-eabi-gcc on PATH (apt: gcc-arm-none-eabi)
#   - TI MSP432 SDK installed; point MSP432_SDK at its root, e.g.:
#       export MSP432_SDK=$HOME/ti/simplelink_msp432p4_sdk_3_40_01_02
#     The default below assumes this layout.
#
# FLASHING (after `make TARGET=msp432`):
#   MSP432Flasher -n MSP432P401R -w build/firmware.hex -v -g
# Add `-z VCC` to power the target from the debugger.
# Or open build/firmware.hex in Code Composer Studio and click Flash.

TARGET  ?= host
BUILD   := build

# ============================================================
# Source lists — same on every target except IMU source + HALs.
# ============================================================
DOMAIN_SRCS := \
    src/fusion_filter.c \
    src/classifier.c \
    src/alert.c \
    src/telemetry.c

ifeq ($(TARGET),msp432)
  IMU_SRCS := src/imu_source_mpu6050.c
  HAL_SRCS := \
      src/hal_buzzer_msp432.c \
      src/hal_uart_msp432.c \
      src/hal_time_msp432.c \
      src/hal_i2c_msp432.c \
      src/hal_platform_msp432.c
else
  IMU_SRCS := src/dummy_generator.c
  HAL_SRCS := \
      src/hal_buzzer_host.c \
      src/hal_uart_host.c \
      src/hal_time_host.c \
      src/hal_i2c_host.c \
      src/hal_platform_host.c
endif

# ============================================================
# Host build configuration (default).
# ============================================================
HOST_CC      := gcc
HOST_CSTD    := -std=c99
HOST_WARN    := -Wall -Wextra -Werror -Wshadow -Wpointer-arith -Wcast-align
HOST_INC     := -Iinclude
HOST_CFLAGS  := $(HOST_CSTD) $(HOST_WARN) $(HOST_INC) -O2
HOST_LIBS    := -lm

# ============================================================
# MSP432 cross-compile configuration.
# ============================================================
MSP432_SDK   ?= $(HOME)/ti/simplelink_msp432p4_sdk_3_40_01_02
ARM_CC       := arm-none-eabi-gcc
ARM_OBJCOPY  := arm-none-eabi-objcopy
ARM_SIZE     := arm-none-eabi-size

ARM_CPU      := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARM_DEFS     := -D__MSP432P401R__ -DTARGET_IS_MSP432P4XX -DDeviceFamily_MSP432P401x -DTARGET_MSP432
ARM_INC      := -Iinclude \
                -I$(MSP432_SDK)/source \
                -I$(MSP432_SDK)/source/ti/devices/msp432p4xx \
                -I$(MSP432_SDK)/source/third_party/CMSIS/Include
ARM_CSTD     := -std=c99
ARM_WARN     := -Wall -Wextra -Wshadow -Wpointer-arith
ARM_CFLAGS   := $(ARM_CPU) $(ARM_CSTD) $(ARM_WARN) $(ARM_DEFS) $(ARM_INC) \
                -ffunction-sections -fdata-sections -Os -g
ARM_LIBPATH  := $(MSP432_SDK)/source/ti/devices/msp432p4xx/driverlib/gcc
ARM_LDSCRIPT := vendor/msp432p401r.lds
ARM_STARTUP  := vendor/startup_msp432p401r_gcc.c \
                vendor/system_msp432p401r.c
ARM_LDFLAGS  := $(ARM_CPU) -T $(ARM_LDSCRIPT) \
                -Wl,--gc-sections -Wl,-Map=$(BUILD)/firmware.map \
                -L$(ARM_LIBPATH) --specs=nosys.specs --specs=nano.specs
ARM_LIBS     := -ldriver -lm

# ============================================================
# Targets dispatch on $(TARGET).
# ============================================================
ifeq ($(TARGET),msp432)
all: device-build
else
all: host test2
endif

# ============================================================
# DEVICE BUILD
# ============================================================
.PHONY: device-build msp432-toolcheck

device-build: msp432-toolcheck $(BUILD)/firmware.hex
	@$(ARM_SIZE) $(BUILD)/firmware.elf
	@echo ""
	@echo "Device build complete: $(BUILD)/firmware.hex"
	@echo "Flash with one of:"
	@echo "  MSP432Flasher -n MSP432P401R -w $(BUILD)/firmware.hex -v -g"
	@echo "  MSP432Flasher -n MSP432P401R -w $(BUILD)/firmware.hex -v -g -z VCC"
	@echo ""
	@echo "Use -z VCC if you want the debugger to power the target;"
	@echo "omit -z if the target is powered separately (battery or USB)."
	@echo "Or open $(BUILD)/firmware.hex in Code Composer Studio."

msp432-toolcheck:
	@command -v $(ARM_CC) >/dev/null 2>&1 || { \
	    echo "ERROR: $(ARM_CC) not found on PATH."; \
	    echo "  Install with: sudo apt install gcc-arm-none-eabi"; \
	    echo "  Or download from https://developer.arm.com/downloads/-/gnu-rm"; \
	    exit 1; }
	@test -d $(MSP432_SDK) || { \
	    echo "ERROR: MSP432_SDK directory not found: $(MSP432_SDK)"; \
	    echo "  Download SimpleLink MSP432P4 SDK from TI and either:"; \
	    echo "    - install to the default path above, or"; \
	    echo "    - export MSP432_SDK=/path/to/simplelink_msp432p4_sdk_X_YY_ZZ_AA"; \
	    exit 1; }
	@test -f $(ARM_LDSCRIPT) || { \
	    echo "ERROR: linker script not found: $(ARM_LDSCRIPT)"; \
	    echo "  See vendor/README.md for how to drop it in from the SDK."; \
	    exit 1; }

ARM_OBJS := $(patsubst src/%.c,$(BUILD)/arm/%.o,$(DOMAIN_SRCS) $(IMU_SRCS) $(HAL_SRCS) src/main.c) \
            $(patsubst vendor/%.c,$(BUILD)/arm/vendor/%.o,$(ARM_STARTUP))

$(BUILD)/arm/%.o: src/%.c | $(BUILD)/arm
	@mkdir -p $(dir $@)
	$(ARM_CC) $(ARM_CFLAGS) -c $< -o $@

$(BUILD)/arm/vendor/%.o: vendor/%.c | $(BUILD)/arm
	@mkdir -p $(dir $@)
	$(ARM_CC) $(ARM_CFLAGS) -c $< -o $@

$(BUILD)/firmware.elf: $(ARM_OBJS) | $(BUILD)
	$(ARM_CC) $(ARM_LDFLAGS) $(ARM_OBJS) $(ARM_LIBS) -o $@

$(BUILD)/firmware.hex: $(BUILD)/firmware.elf
	$(ARM_OBJCOPY) -O ihex $< $@

$(BUILD)/arm:
	@mkdir -p $(BUILD)/arm

# ============================================================
# HOST BUILDS — unchanged from Phase 2.
# ============================================================
.PHONY: host test test2 pytest pytest2 clean

host: $(BUILD)/firmware_host

$(BUILD)/firmware_host: $(DOMAIN_SRCS) $(IMU_SRCS) $(HAL_SRCS) src/main.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) $^ -o $@ $(HOST_LIBS)

# Phase 1 regression — links Phase 1's Phase-1-shaped sources only.
PHASE1_TEST_DOMAIN_SRCS := \
    src/dummy_generator.c \
    src/fusion_filter.c \
    src/classifier.c \
    src/alert.c \
    src/telemetry.c

PHASE1_TEST_HAL_SRCS := \
    src/hal_buzzer_host.c \
    src/hal_time_host.c

$(BUILD)/test_phase1: $(PHASE1_TEST_DOMAIN_SRCS) $(PHASE1_TEST_HAL_SRCS) src/test_phase1_main.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) $^ -o $@ $(HOST_LIBS)

test: $(BUILD)/test_phase1
	./$(BUILD)/test_phase1

# Phase 2 harness — links MPU-6050 driver against host I2C HAL.
PHASE2_TEST_HAL_SRCS := \
    src/hal_buzzer_host.c \
    src/hal_time_host.c \
    src/hal_i2c_host.c

$(BUILD)/test_phase2: $(DOMAIN_SRCS) src/imu_source_mpu6050.c $(PHASE2_TEST_HAL_SRCS) src/test_phase2_main.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) $^ -o $@ $(HOST_LIBS)

test2: $(BUILD)/test_phase2
	./$(BUILD)/test_phase2

$(BUILD)/firmware_host_finite: $(DOMAIN_SRCS) src/dummy_generator.c $(HAL_SRCS) src/main.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -DPHASE1_MAIN_ITERATIONS=60 -DPHASE2_MAIN_ITERATIONS=60 -DUSE_TELEMETRY_V1 $^ -o $@ $(HOST_LIBS)

pytest: $(BUILD)/firmware_host_finite
	python3 tools/test_phase1.py $(BUILD)/firmware_host_finite

$(BUILD)/firmware_host_finite_v2: $(DOMAIN_SRCS) src/dummy_generator.c $(HAL_SRCS) src/main.c | $(BUILD)
	$(HOST_CC) $(HOST_CFLAGS) -DPHASE2_MAIN_ITERATIONS=200 $^ -o $@ $(HOST_LIBS)

pytest2: $(BUILD)/firmware_host_finite_v2
	python3 tools/test_phase2.py $(BUILD)/firmware_host_finite_v2

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)