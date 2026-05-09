# Phase 1 Makefile.
#
# Targets:
#   make host        - build the firmware on the host with stub HALs
#   make test        - build and run the in-process C test harness
#   make pytest      - build host firmware (with finite iterations) and run test_phase1.py
#   make msp432      - placeholder: real flash build (see comments)
#   make clean       - remove build artefacts
#
# C99, pedantic warnings, -Werror.

CC      ?= gcc
CSTD    := -std=c99
WARN    := -Wall -Wextra -Werror -Wshadow -Wpointer-arith -Wcast-align
INC     := -Iinclude
CFLAGS  := $(CSTD) $(WARN) $(INC) -O2

BUILD   := build

# Domain modules — same on every target.
DOMAIN_SRCS := \
    src/dummy_generator.c \
    src/fusion_filter.c \
    src/classifier.c \
    src/alert.c \
    src/telemetry.c

# Host HAL implementations.
HOST_HAL_SRCS := \
    src/hal_buzzer_host.c \
    src/hal_uart_host.c \
    src/hal_time_host.c

# MSP432 HAL implementations (placeholders; fleshed out in Phase 2).
MSP432_HAL_SRCS := \
    src/hal_buzzer_msp432.c \
    src/hal_uart_msp432.c \
    src/hal_time_msp432.c

# Math lib needed for sinf/atan2f/fabsf.
LIBS := -lm

.PHONY: all host test pytest msp432 clean

all: host test

# ------------------------------------------------------------------
# Host firmware: real main loop, host stubs, runs forever by default.
# Override iteration count via PHASE1_MAIN_ITERATIONS=<N>.
# ------------------------------------------------------------------
host: $(BUILD)/firmware_host

$(BUILD)/firmware_host: $(DOMAIN_SRCS) $(HOST_HAL_SRCS) src/main.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# ------------------------------------------------------------------
# Test harness binary. test_phase1_main.c provides its own
# uart_init / uart_write_bytes definitions, so we deliberately
# exclude hal_uart_host.c from this target's link to avoid
# duplicate symbols.
# ------------------------------------------------------------------
TEST_HAL_SRCS := \
    src/hal_buzzer_host.c \
    src/hal_time_host.c

$(BUILD)/test_phase1: $(DOMAIN_SRCS) $(TEST_HAL_SRCS) src/test_phase1_main.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

test: $(BUILD)/test_phase1
	./$(BUILD)/test_phase1

# ------------------------------------------------------------------
# pytest: host firmware compiled with finite iterations + run the
# Python validator against it.
# ------------------------------------------------------------------
$(BUILD)/firmware_host_finite: $(DOMAIN_SRCS) $(HOST_HAL_SRCS) src/main.c | $(BUILD)
	$(CC) $(CFLAGS) -DPHASE1_MAIN_ITERATIONS=60 $^ -o $@ $(LIBS)

pytest: $(BUILD)/firmware_host_finite
	python3 tools/test_phase1.py $(BUILD)/firmware_host_finite

# ------------------------------------------------------------------
# MSP432 target — placeholder. Phase 2 will fill this in with real
# arm-none-eabi-gcc invocations against the DriverLib. We keep a
# rule here as a checkpoint so anyone trying it sees a clear message
# instead of a confusing build error.
# ------------------------------------------------------------------
msp432:
	@echo "MSP432 flash build not implemented in Phase 1."
	@echo "Sources are ready: \$(DOMAIN_SRCS) + \$(MSP432_HAL_SRCS) + src/main.c"
	@echo "Phase 2: drop in arm-none-eabi-gcc + DriverLib + linker script."
	@false

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
