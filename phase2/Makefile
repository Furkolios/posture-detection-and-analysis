# Phase 2 Makefile.
#
# Targets:
#   make host             - host firmware loop with dummy generator
#   make test             - Phase 1 C harness (regression net)
#   make test2            - Phase 2 C harness (CP6/CP7/CP8/CP9 host paths)
#   make pytest           - Phase 1 Python validator
#   make pytest2          - Phase 2 Python validator (V2 packets)
#   make TARGET=msp432    - device firmware: MPU-6050 source + MSP432 HALs
#                           (placeholder rule — fails until DriverLib added)
#   make clean            - remove build artefacts
#
# Pass TARGET=msp432 to switch IMU source and HAL set.

CC      ?= gcc
CSTD    := -std=c99
WARN    := -Wall -Wextra -Werror -Wshadow -Wpointer-arith -Wcast-align
INC     := -Iinclude
CFLAGS  := $(CSTD) $(WARN) $(INC) -O2

BUILD   := build
TARGET  ?= host

# Domain modules — same on every target except for the IMU source, which
# is selectable via $(TARGET).
DOMAIN_SRCS := \
    src/fusion_filter.c \
    src/classifier.c \
    src/alert.c \
    src/telemetry.c

# IMU source selection.
ifeq ($(TARGET),msp432)
  IMU_SRCS := src/imu_source_mpu6050.c
  HAL_SRCS := \
      src/hal_buzzer_msp432.c \
      src/hal_uart_msp432.c \
      src/hal_time_msp432.c \
      src/hal_i2c_msp432.c
else
  # Host build links BOTH the dummy generator (it owns imu_source.h)
  # AND the host I2C HAL (so test_phase2_main.c can also exercise the
  # MPU-6050 driver path). The MPU-6050 source itself is NOT linked
  # into the host firmware to avoid duplicate imu_source_init/read
  # symbols. test_phase2 links a different combination — see below.
  IMU_SRCS := src/dummy_generator.c
  HAL_SRCS := \
      src/hal_buzzer_host.c \
      src/hal_uart_host.c \
      src/hal_time_host.c \
      src/hal_i2c_host.c
endif

LIBS := -lm

.PHONY: all host test test2 pytest pytest2 msp432 clean

all: host test2

# ------------------------------------------------------------------
# Host firmware: dummy generator + V2 telemetry.
# ------------------------------------------------------------------
host: $(BUILD)/firmware_host

$(BUILD)/firmware_host: $(DOMAIN_SRCS) $(IMU_SRCS) $(HAL_SRCS) src/main.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# ------------------------------------------------------------------
# Phase 1 C harness — regression net.
# Excludes hal_uart_host.c (the harness defines its own).
# ------------------------------------------------------------------
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
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

test: $(BUILD)/test_phase1
	./$(BUILD)/test_phase1

# ------------------------------------------------------------------
# Phase 2 C harness — exercises the MPU-6050 driver path through the
# host I2C HAL, so we link imu_source_mpu6050.c + hal_i2c_host.c and
# DROP the dummy_generator's own imu_source impl. The dummy generator
# is still linked because hal_i2c_host.c uses dummy_set_scenario /
# dummy_get_target_angle to choose what to fake.
#
# To avoid duplicate symbols (both dummy_generator.c and
# imu_source_mpu6050.c define imu_source_init/read), we compile a
# trimmed dummy_generator that exposes ONLY the scenario knob — done
# by passing -DDUMMY_NO_IMU_SOURCE_IMPL. dummy_generator.c is already
# host-only test code; this define is a Phase 2 addition just for
# this target.
#
# The harness also defines its own uart_init/uart_write_bytes, so we
# exclude hal_uart_host.c here.
# ------------------------------------------------------------------
PHASE2_TEST_HAL_SRCS := \
    src/hal_buzzer_host.c \
    src/hal_time_host.c \
    src/hal_i2c_host.c

$(BUILD)/test_phase2: $(DOMAIN_SRCS) src/imu_source_mpu6050.c $(PHASE2_TEST_HAL_SRCS) src/test_phase2_main.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

test2: $(BUILD)/test_phase2
	./$(BUILD)/test_phase2

# ------------------------------------------------------------------
# Phase 1 pytest. Reuses the original validator and binary.
# ------------------------------------------------------------------
$(BUILD)/firmware_host_finite: $(DOMAIN_SRCS) src/dummy_generator.c $(HAL_SRCS) src/main.c | $(BUILD)
	$(CC) $(CFLAGS) -DPHASE1_MAIN_ITERATIONS=60 -DPHASE2_MAIN_ITERATIONS=60 -DUSE_TELEMETRY_V1 $^ -o $@ $(LIBS)

pytest: $(BUILD)/firmware_host_finite
	python3 tools/test_phase1.py $(BUILD)/firmware_host_finite

# ------------------------------------------------------------------
# Phase 2 pytest. Same firmware build but with V2 telemetry (which is
# already what main.c emits in Phase 2). Iteration count covers >4 sec
# at 20 Hz so the dummy scenario cycle visits all three states.
# ------------------------------------------------------------------
$(BUILD)/firmware_host_finite_v2: $(DOMAIN_SRCS) src/dummy_generator.c $(HAL_SRCS) src/main.c | $(BUILD)
	$(CC) $(CFLAGS) -DPHASE2_MAIN_ITERATIONS=200 $^ -o $@ $(LIBS)

pytest2: $(BUILD)/firmware_host_finite_v2
	python3 tools/test_phase2.py $(BUILD)/firmware_host_finite_v2

# ------------------------------------------------------------------
# MSP432 device build. Phase 2 keeps this as a structural placeholder.
# Phase 3 replaces the rule with arm-none-eabi-gcc + linker script +
# DriverLib path. The source list itself is correct already.
# ------------------------------------------------------------------
msp432:
	@echo "MSP432 flash build not implemented in Phase 2."
	@echo "Sources are ready: \$(DOMAIN_SRCS) + \$(IMU_SRCS) + \$(HAL_SRCS) + src/main.c"
	@echo "To enable: replace this rule with arm-none-eabi-gcc + ti DriverLib + linker script."
	@false

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
