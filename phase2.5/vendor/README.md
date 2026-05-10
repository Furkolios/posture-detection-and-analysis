# vendor/

Three files belong here, all copied verbatim from your TI MSP432 SDK
install. They're not in this repo because they're TI's licensed code,
not ours.

After installing the SimpleLink MSP432P4 SDK and setting `MSP432_SDK`,
copy the following into this directory (paths are relative to
`$MSP432_SDK`):

| Copy from | Copy to |
|---|---|
| `source/ti/devices/msp432p4xx/startup_system_files/gcc/startup_msp432p401r_gcc.c` | `vendor/startup_msp432p401r_gcc.c` |
| `source/ti/devices/msp432p4xx/startup_system_files/gcc/system_msp432p401r.c` | `vendor/system_msp432p401r.c` |
| `source/ti/devices/msp432p4xx/startup_system_files/gcc/msp432p401r.lds` | `vendor/msp432p401r.lds` |

(Older SDK versions have the same files under `startup_system_files/`
without the `gcc` subdirectory — the GCC variant is what we need.)

The Makefile's `make TARGET=msp432` rule will refuse to build until
these are in place and emit an error pointing back to this README.

## What each file does

- **`startup_msp432p401r_gcc.c`** — the interrupt vector table and the
  default `Reset_Handler` that copies `.data` from flash to RAM, zeroes
  `.bss`, calls `SystemInit()`, and jumps to `main()`. It also defines
  weak-default handlers for every IRQ; our `SysTick_Handler` in
  `hal_time_msp432.c` overrides the weak default automatically because
  GCC prefers the strong symbol.

- **`system_msp432p401r.c`** — defines `SystemInit()`, called by the
  startup code before `main`. The default implementation does the
  minimum (sets up the FPU). Our `hal_platform_init()` does the rest
  of the clock tree configuration.

- **`msp432p401r.lds`** — GCC linker script. Tells `arm-none-eabi-gcc`
  where flash (256 KB at 0x00000000) and RAM (64 KB at 0x20000000)
  live, and how to lay out `.text`, `.data`, `.bss`, the vector table,
  and the heap/stack regions.

## Why aren't these committed?

TI ships them under the BSD license, but they belong to TI rather
than this project. Keeping them out of git makes the repo clean and
avoids re-distribution questions. Each developer pulls them from
their own SDK install at build time.
