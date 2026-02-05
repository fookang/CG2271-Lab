/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_2.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

//LED pin numbers
#define RED_PIN		31	//PTE31
#define GREEN_PIN	5	//PTD5
#define BLUE_PIN	29	//PTE29

#define SWITCH_PIN 4 //PTA4

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

//TODO
void setMCGIRClk() {
    // Clear clk
    MCG->C1 &= ~MCG_C1_CLKS_MASK;

    // Set clock source to LIRC
    // Set IRCLKEN to enable LIRC
    MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

    // Set IRCS to choose 8 MHz clock
    MCG->C2 |= MCG_C2_IRCS_MASK;

    // Set FRCDIV for divisor 1
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
    MCG->SC |= MCG_SC_FCRDIV(0b0);

    // Set LIRC_DIV2 for divisor 1
    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
    MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
}

void initTimer() {
	//disable TPM1 interrupt
	NVIC_DisableIRQ(TPM1_IRQn);

	setMCGIRClk();

	//turn on clock gating to TPM1
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

	//Choose 8MHz MCGIRCLK
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);

	//Turn off TPM1 and mask the prescalar
    TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

	//Set prescalar of 128, and enable TOIE
    TPM1->SC |= (TPM_SC_TOIE_MASK | TPM_SC_PS(0b111));

	//Initialize the counter to 0
    TPM1->CNT = 0;

	//Initialize modulo of 625
    TPM1->MOD = 625;

	//Set the priority to the highest & enable IRQ
    NVIC_SetPriority(TPM1_IRQn, 0);
    NVIC_EnableIRQ(TPM1_IRQn);
}

void startTimer() {
    TPM1->SC |= TPM_SC_CMOD(0b1);
}

void stopTimer() {
    TPM1->SC &= ~TPM_SC_CMOD_MASK;
}

void initPWM() {
	//turn on clock gating to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	//turn on clock gating to PORTD and PORTE
	SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK);

	//configure the RGB LED MUX to be for PWM
	PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_PIN] |= PORT_PCR_MUX(0b11);

	PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[BLUE_PIN] |= PORT_PCR_MUX(0b11);

	PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(0b11);

	//set pins to output (good practice)
	GPIOE->PDDR |= ((1 << RED_PIN) | (1 << BLUE_PIN));
    GPIOD->PDDR |= (1 << GREEN_PIN);

	//the following code is used to setup TPM0
	//turn off TPM0 by clearing the clock mode
    TPM0->SC &= ~TPM_SC_CMOD_MASK;

	//clear and set the prescalar 128
    TPM0->SC &= ~TPM_SC_PS_MASK;
    TPM0->SC |= (TPM_SC_TOIE_MASK | TPM_SC_PS(0b111));

	//set centre-aligned PWM mode
    TPM0->SC |= TPM_SC_CPWMS_MASK;

	//initialize count to 0
    TPM0->CNT = 0;

	//choose and initialize modulo 125
    TPM0->MOD = 125;

	//Configure TPM0 channels.
	//IMPORTANT: Configure a REVERSE PWM signal!!!
	//i.e. it sets when counting up and clears when counting
	//down. This is because the LEDs are active low.
    // MS = 10, ELS = 01
    TPM0->CONTROLS[2].CnSC &= ~(TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK);
    TPM0->CONTROLS[2].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_MSA(0) | TPM_CnSC_ELSB(0) | TPM_CnSC_ELSA(1));
    TPM0->CONTROLS[4].CnSC &= ~(TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK);
    TPM0->CONTROLS[4].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_MSA(0) | TPM_CnSC_ELSB(0) | TPM_CnSC_ELSA(1));
    TPM0->CONTROLS[5].CnSC &= ~(TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK);
    TPM0->CONTROLS[5].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_MSA(0) | TPM_CnSC_ELSB(0) | TPM_CnSC_ELSA(1));
}

void startPWM() {
	//set CMOD for TPM0
    TPM0->SC |= TPM_SC_CMOD(0b1);
}

void stopPWM() {
	//mask CMOD for TPM0
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

void setPWM(int LED, int percent) {
	// convert percent into a value
	int value = (int) (percent / 100.0 * (double) TPM0->MOD);
	switch(LED) {
	    case(RED):
		//	set TPM0 control value
		//	repeat for BLUE and GREEN
			TPM0->CONTROLS[4].CnV = value;
	    	break;
	    case(GREEN):
			TPM0->CONTROLS[5].CnV = value;
	    	break;
	    case(BLUE):
			TPM0->CONTROLS[2].CnV = value;
	    	break;
	    default: printf("invalid LED.\r\n");
	}
}

volatile int val = 0;

void TPM1_IRQHandler(){

	NVIC_ClearPendingIRQ(TPM1_IRQn);

	static int dir = 1;

	if(TPM1->STATUS & TPM_STATUS_TOF_MASK) {
		val += dir;
		if (val > 0 && val <= 100) {
			setPWM(RED, val);
		} else if (val > 100 && val <= 200) {
			setPWM(GREEN, val - 100);
		} else if (val > 200 && val <= 300) {
			setPWM(BLUE, val - 200);
		}
		if (val >= 300 || val <= 0) {
			dir *= -1;
		}

//			PRINTF("redVal=%d, greenVal=%d, blueVal=%d\r\n",
//					redVal, greenVal, blueVal);
			PRINTF("val=%d\r\n", val);

			TPM1->CNT=0;
			TPM1->STATUS |= TPM_STATUS_TOF_MASK;
		}
}

void initButton() {
    //Disable interrupts
    NVIC_DisableIRQ(PORTA_IRQn);

    //Enable clock gating to PORTA
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK);

    //Configure MUX of PTA4
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_MUX(1);

	//Set pullup resistor
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PS(1);
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PE_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PE(1);

	//Set as input
	GPIOA->PDDR &= ~(1 << SWITCH_PIN);

	//Configure the interrupt for falling edge
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_IRQC(0b1010);

	//Set NVIC priority to 0,
	//highest priority
	NVIC_SetPriority(PORTA_IRQn, 0);


	//Clear pending interrupts and enable interrupts
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void PORTA_IRQHandler() {
    NVIC_ClearPendingIRQ(PORTA_IRQn);

	//We don't actually have to test that it is
	//PTA4 since there is only one interrupt
	//configured, but this shows how to do it.
	if(PORTA->ISFR & (1 << SWITCH_PIN)) {
        stopPWM();
        TPM0->CNT = 0;
            // TODO: write modulo for 20Hz
        if (TPM0->MOD == 1563) {
            // TODO: write modulo for 250Hz
        	TPM0->MOD = 125;
        } else {
            // TODO: write modulo for 20Hz
        	TPM0->MOD = 1563; // rounded up from 1562.5
        }
        startPWM();
	}

    //Write a 1 to clear the ISFR bit.
    PORTA->ISFR |= (1 << SWITCH_PIN);

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

    //Set the TPM clock
    initTimer();
    initPWM();
    initButton();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    PRINTF("PWM DEMO\r\n");
    startPWM();
    startTimer();

    /* Enter an infinite loop, just incrementing a counter. */
    int i = 0;
    while(1) {
    	i++;
    }
    return 0 ;
}

