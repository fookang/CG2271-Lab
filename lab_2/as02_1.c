/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/*insert other include files here. */

/*insert other definitions and declarations here. */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

typedef enum tl {
    RED, GREEN, BLUE
} TLED;

void setMCGIRClk()
{
    // Clear clk
    MCG->C1 &= ~MCG_C1_CLKS_MASK;

    // Set clock source to LIRC
    // Set IRCLKEN to enable LIRC
    MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

    // Set IRCS to choose 2 MHz clock
    MCG->C2 &= ~MCG_C2_IRCS_MASK;

    // Set FRCDIV
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
    MCG->SC |= MCG_SC_FCRDIV(0b0);

    // Set LIRC_DIV2 for scaling to 1 MHz
    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
    MCG->MC |= MCG_MC_LIRC_DIV2(0b1);

}

void initTimer()
{
    // Disbale TPM0 interrupt
    NVIC_DisableIRQ(TPM0_IRQn);

    setMCGIRClk();

    // Turn on clock gating for TMPO
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Clear TPM clock source
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;

    // Select MCGICLK
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);

    // Turn off TPM0 and clear Prescale counter
    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

    // Set prescalar to 8 and TOIE mask
    TPM0->SC |= (TPM_SC_TOIE_MASK | TPM_SC_PS(0b11));

    // Initialize count to 0
    TPM0->CNT = 0;

    // Initialize modulo
    TPM0->MOD = 62500;

    // Set priority to highest
    NVIC_SetPriority(TPM0_IRQn, 0);

    // Enable IRQ
    NVIC_EnableIRQ(TPM0_IRQn);
}

void startTimer()
{
    TPM0->SC |= TPM_SC_CMOD(0b1);
}

void stopTimer()
{
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

//count needs to be volatile.
volatile int count = 0;

void TPM0_IRQHandler()
{
    // Clear pending IRQ
    NVIC_ClearPendingIRQ(TPM0_IRQn);

    // Check if TOF is set
    if (TPM0->STATUS & TPM_STATUS_TOF_MASK) 
    {
        count = (count + 1) % 6;

        // Reset counter
        TPM0->CNT = 0;

        // Clear TOF flag
        TPM0->STATUS |= TPM_STATUS_TOF_MASK;
    }
}

void initGPIO()
{
    // Set clock for GPIO D E
    SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    // GPIO for PTE31
    PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[RED_PIN] |= PORT_PCR_MUX(1);

    // GPIO for PTE29
    PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[BLUE_PIN] |= PORT_PCR_MUX(1);

    // GPIO for PTD5
    PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(1);

    // Set PTE29 and PTE31 as output
    GPIOE->PDDR |= (1 << RED_PIN) | (1 << BLUE_PIN);

    // Set PTD5 as output
    GPIOD->PDDR |= (1 << GREEN_PIN);

    // Switch off LED
    GPIOE->PSOR |= (1 << RED_PIN) | (1 << BLUE_PIN);
    GPIOD->PSOR |= (1 << GREEN_PIN);

}

void ledOn(TLED led) {
    switch(led) {
        case RED:
            GPIOE->PCOR |= (1 << RED_PIN);
            break;

        case GREEN:
            GPIOD->PCOR |= (1 << GREEN_PIN);
            break;

        case BLUE:
            GPIOE->PCOR |= (1 << BLUE_PIN);
            break;
    }
}

void ledOff(TLED led) {
    switch(led) {
        case RED:
            GPIOE->PSOR |= (1 << RED_PIN);
            break;

        case GREEN:
            GPIOD->PSOR |= (1 << GREEN_PIN);
            break;

        case BLUE:
            GPIOE->PSOR |= (1 << BLUE_PIN);
            break;
    }
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    initGPIO();
    initTimer();
    PRINTF("TIMER DEMO\r\n");
    startTimer();

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        switch(count) {
            case 0:
                ledOn(RED);
                break;
            case 1:
                ledOff(RED);
                break;
            case 2:
                ledOn(GREEN);
                break;
            case 3:
                ledOff(GREEN);
                break;
            case 4:
                ledOn(BLUE);
                break;
            case 5:
                ledOff(BLUE);
                break;
            default:
                count=0;
        }
    }
    return 0 ;
}
