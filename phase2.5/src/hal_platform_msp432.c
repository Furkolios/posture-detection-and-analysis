/*
 * hal_platform_msp432.c — Device-specific bring-up that runs before
 * any HAL peripheral is touched.
 *
 * Responsibilities:
 *   1. Disable the watchdog so it doesn't reset us during init.
 *   2. Configure the clock tree: DCO at 48 MHz, MCLK = DCO, SMCLK = 12 MHz.
 *   3. Set up SysTick at 1 ms.
 *   4. Enable interrupts globally.
 *
 * Why these exact frequencies: the UART baud divisors in
 * hal_uart_msp432.c assume SMCLK = 12 MHz; the SysTick reload in
 * hal_time_msp432.c assumes MCLK = 48 MHz. Change one, change the
 * other, or both will silently break.
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void hal_time_msp432_init(void);   /* defined in hal_time_msp432.c */

void hal_platform_init(void);
void hal_platform_init(void) {
    /* 1. Disable watchdog. */
    WDT_A_holdTimer();

    /* 2. Clock tree.
     *    - VCORE1 needed for >24 MHz CPU clock.
     *    - Flash wait states: 1 wait state for 48 MHz, both banks.
     *    - DCO frequency range = 48 MHz.
     *    - MCLK = DCO (48 MHz), HSMCLK/SMCLK = DCO/4 (12 MHz),
     *      ACLK = REFOCLK (32.768 kHz). */
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK,    CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_4);
    CS_initClockSignal(CS_SMCLK,   CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_4);
    CS_initClockSignal(CS_ACLK,    CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* 3. SysTick. */
    hal_time_msp432_init();

    /* 4. Global interrupts (SysTick handler depends on this). */
    Interrupt_enableMaster();
}
