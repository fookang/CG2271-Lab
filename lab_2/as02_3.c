/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_3.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/*
 * @brief   Application entry point.
 */

// LED pin numbers22
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

#define SWITCH_PIN   4  // PTA4

#define ADC_PIN1 30
#define ADC_PIN1_CHANNEL 23
#define ADC_PIN2 2
#define ADC_PIN2_CHANNEL 12

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

/*
 * @brief   Application entry point.
 */
// Configure ADC_SE0 on PTE20 and ADC_SE4a on PTE21

#define ADC_SE0			0
#define ADC_SE0_PIN	20
#define ADC_SE4a			4
#define ADC_SE4_PIN	21

//TODO
void initADC() {
	// Disable & clear interrupt
	NVIC_DisableIRQ(ADC0_IRQn);
	NVIC_ClearPendingIRQ(ADC0_IRQn);

	// Enable clock gating to relevant configurations
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Set pins from Q3 to ADC
	PORTE->PCR[ADC_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[ADC_PIN1] |= PORT_PCR_MUX(0b0);

	PORTB->PCR[ADC_PIN2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[ADC_PIN2] |= PORT_PCR_MUX(0b0);

	// Configure the ADC
	// Enable ADC interrupt
	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

	// Select single-ended ADC
	ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
	ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);

	// Set 12 bit conversion
	ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
	ADC0->CFG1 |= ADC_CFG1_MODE(0b01);

	// Select software conversion trigger
	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

	// Configure alternate voltage reference
	ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
	ADC0->SC2 |= ADC_SC2_REFSEL(0b01);

	// Don't use averaging
	ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
	ADC0->SC3 |= ADC_SC3_AVGE(0);

	// Switch off continuous conversion.
	ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
	ADC0->SC3 |= ADC_SC3_ADCO(0);

	// Set highest priority
	NVIC_SetPriority(ADC0_IRQn, 0);
	NVIC_EnableIRQ(ADC0_IRQn);
}

int result[2];

void startADC(int channel) {
	//mask and set the channel
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(channel);
}

//TODO! This is NOT going to work.
void ADC0_IRQHandler(){
	static int turn=0;

	NVIC_ClearPendingIRQ(ADC0_IRQn);

	if(ADC0->SC1[0] & ADC_SC1_COCO_MASK) {
		result[turn] = ADC0->R[0];
		//PRINTF("Turn = %d, Result = %d\r\n", turn, result[turn]);
		turn = 1 - turn;
		if(turn == 0) {
			startADC(ADC_PIN1_CHANNEL);
		} else {
			startADC(ADC_PIN2_CHANNEL);
		}
	}
}

// Configure the MCG Internal Reference Clock
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

void setTPMClock(){
	//Choose 8MHz MCGIRCLK
	setMCGIRClk();

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
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
	PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(0b100);

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
	int value = (int) (percent / 255.0 * (double) TPM0->MOD);
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
		//TODO
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

    PRINTF("Hello World\r\n");
    setTPMClock();
    initPWM();
	initButton();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    startPWM();

    initADC();
    startADC(ADC_PIN1_CHANNEL);
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;

        // Sample way to convert values
        int red = (int) (result[0] / 4096.0 * 255.0);
        int green = (int) (result[1] / 4096.0 * 255.0);
        int blue = (red + green) / 2;

        PRINTF("RED = %d GREEN = %d BLUE = %d\r\n", red, green, blue);
        setPWM(RED, red);
        setPWM(GREEN, green);
        setPWM(BLUE, blue);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}


