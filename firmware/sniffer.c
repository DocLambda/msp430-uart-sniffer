/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
/******************************************************************************/

/******************************************************************************
 * This code is based on the following examples provided by Texas Instruments:
 *
 * *  file:		msp430g2xx3_uscia0_uart_01_115k.c
 *    description:	MSP430G2xx3 Demo - USCI_A0, 115200 UART Echo ISR, DCO SMCLK
 *    author:		D. Dang (Texas Instruments Inc) -- February 2011
 *
 * *  file:		msp430g2xx3_ta_uart9600.c
 *    description:	MSP430G2xx3 Demo - Timer_A, Ultra-Low Pwr UART 9600 Echo, 32kHz ACLK
 *    author:		D. Dang (Texas Instruments Inc) -- December 2010
 ******************************************************************************/

#include <msp430.h>

/* LEDs */
#define LED_RED				BIT0
#define LED_GREEN			BIT6

/* Settings for 9600 baud @ 1MHz */
#define UART0_RXD			BIT1		// P1.1 using USCI
#define UART0_TXD			BIT2		// P1.2 using USCI
#define UART0_DIVIDER			104		// according to table 15.4 in users manual
#define UART0_DIVIDER_BR0		(UART0_DIVIDER & 0xFF)
#define UART0_DIVIDER_BR1		(UART0_DIVIDER >> 8)
#define UART0_DIVIDER_MCTL		UCBRS0		// according to table 15.4 in users manual

/* Settings for 9600 baud @ 1MHz */
#define UART1_RXD			BIT1		// P2.1 using Timer1_CCI1A
#define UART1_DIVIDER			104		// 9600baud @ 1MHz = 104.16
#define UART1_DIVIDER_HALF		52		// half-bit duration to sample in the middle of the bit = 52.08

/* Error blink codes */
#define ERR_LED_CYCLES_ON		200UL  * 1200UL // 200ms @ 1.2MHz
#define ERR_LED_CYCLES_OFF		1000UL * 1200UL // 100ms @ 1.2MHz
#define ERR_LED_BLINKS_CALIB		1

/* Variables for the TX FIFO */
#define TX_CAPACITY			16
volatile unsigned char tx_lock  = 0;
volatile unsigned char tx_buffer[TX_CAPACITY];
volatile unsigned char tx_count = 0;
volatile unsigned char tx_head  = 0;
volatile unsigned char tx_tail  = 0;

void blink_led_and_trap(unsigned int blinks)
{
	unsigned int b;

	/* Flash the led and trap */
	while (1)
	{
		P1OUT &= ~LED_RED;
		__delay_cycles(ERR_LED_CYCLES_OFF);
		for (b = 0; b < blinks; b++)
		{
			P1OUT |= LED_RED;
			__delay_cycles(ERR_LED_CYCLES_ON);
			P1OUT &= ~LED_RED;
			__delay_cycles(ERR_LED_CYCLES_ON);
		}
	}
}

void setup_hwuart(void)
{
	/* Setup pins for RX and TX */
	P1SEL  = UART0_RXD + UART0_TXD;			// Select special function for RXD and TXD
	P1SEL2 = UART0_RXD + UART0_TXD;

	/* Setup UART clock and state machine */
	UCA0CTL1 |= UCSSEL_2;				// SMCLK
	UCA0BR0   = UART0_DIVIDER_BR0;    	        // Baud rate generator divider settings
	UCA0BR1   = UART0_DIVIDER_BR1;
	UCA0MCTL  = UART0_DIVIDER_MCTL;          	// Modulation setting for baud rate generator
	UCA0CTL1 &= ~UCSWRST;				// **Initialize USCI state machine**

	IE2 |= UCA0RXIE;				// Enable USCI_A0 RX interrupt
}

void setup_swuart(void)
{
	/* Setup timer 1 CCR1 */
	TA1CCTL1 = SCS + CM1 + CAP + CCIE;		// Sync, high-low edge, capture, interrupt enabled
	TA1CTL   = TASSEL_2 + MC_2;			// SMCLK, continuous mode

	/* Setup the receiver pin */
	P2OUT &= ~UART1_RXD;				// Clear pin
	P2SEL |=  UART1_RXD;				// Timer1_CCI1A special function
	P2DIR &= ~UART1_RXD;				// Input
}

int main(void)
{
	const char *readymsg = "UART SNIFFER READY\r\n";
	int i;

	/* Disable watchdog */
	WDTCTL = WDTPW + WDTHOLD;

	/* Initialize LEDs */
	P1DIR  =  (LED_RED + LED_GREEN);		// Set LEDs to output
	P1OUT &= ~(LED_RED + LED_GREEN);		// LEDs off

	/* Calibrate DCO with factory calibration data.
	 * Catch deleted calibration data and error out.
	 */
	if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)
	{
		blink_led_and_trap(ERR_LED_BLINKS_CALIB);
	}
	DCOCTL  = 0;					// Select lowest DCOx and MODx settings
	DCOCTL  = CALDCO_1MHZ;				// Use the factory calibration data for DCO
	BCSCTL1 = CALBC1_1MHZ;

	/* Setup the hardware UART as the first uart (RX and TX).
	 * This uart will be used as a first receiver, but also
	 * for transmitting the gathered data to the host.
	 */
	setup_hwuart();

	/* Setup additional receiver */
	setup_swuart();

	/* Initialize TX FIFO */
	for (i = 0; i < TX_CAPACITY; i++)
	{
		tx_buffer[i] = 0xFF;
	}

	/* Signal Readyness */
	P1OUT |= LED_RED; 				// Red LED on to signal READY
	P1OUT |= LED_GREEN; 				// Green LED on to signal READY

	__enable_interrupt();

	/* Send ready message to the host */
	for (i = 0; i < 21; i++)
	{
		while (!(IFG2 & UCA0TXIFG));		// USCI_A0 TX buffer ready?
		UCA0TXBUF = readymsg[i];		// TX -> RXed character
	}

	/* Main loop for scheduling received data for transmission */
	while (1)
	{
		__bis_SR_register(LPM0_bits + GIE);	// Enter LPM0, interrupts enabled
		P1OUT &= ~LED_GREEN;			// LED off for start of transmission
		while (tx_count > 0)
		{
			while (!(IFG2 & UCA0TXIFG));	// USCI_A0 TX buffer ready?

			UCA0TXBUF = tx_buffer[tx_tail];	// TX character
			tx_buffer[tx_tail] = 0xFF;	// Clear slot
			tx_tail = (tx_tail + 1) % TX_CAPACITY; // Move on with write pointer

			/* Lock the tx-queue to remove a byte */
			while (tx_lock);		// FIXME: This should be atomic test-and-set
			tx_lock = 1;

			tx_count--;

			/* Release lock to tx-queue */
			tx_lock = 0;
		}
		P1OUT |= LED_GREEN;			// LED on for end of transmission
	}

	/* We should never end up here... */
	return 0;
}

/* Interrupt service routine for first receiver */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void UART0_RX_ISR(void)
{
	unsigned char rx0_byte = UCA0RXBUF;

	/* Lock the tx-queue to add one more byte */
	while (tx_lock);				// FIXME: This should be atomic test-and-set
	tx_lock = 1;

	/* Store received byte in transmission buffer WITH LOCK HELD */
	tx_buffer[tx_head] = rx0_byte;			// Add received byte
	tx_head = (tx_head + 1) % TX_CAPACITY;		// Switch to next free slot
	tx_count++;					// We have one more to tx
	if (tx_count >= TX_CAPACITY)
		P1OUT &= ~LED_RED;			// Turn red led off for overflow

	/* Release lock to tx-queue */
	tx_lock = 0;

	/* Schedule transmission in main loop */
	__bic_SR_register_on_exit(LPM0_bits);
}

/* Interrupt service routine for second receiver */
#pragma vector = TIMER1_A1_VECTOR
__interrupt void UART1_RX_ISR(void)
{
	static unsigned char rx1_byte     = 0x00;
	static unsigned char rx1_bitcount = 8;

	/* Handle the capture / compare interrupts of CCR1A*/
	if (TA1IV & TA1IV_TA1CCR1)
	{
		TA1CCR1 += UART1_DIVIDER;		// Wait for next bit
		if (TA1CCTL1 & CAP) 			// Capture mode --> start bit
		{
			TA1CCTL1 &= ~CAP;		// Switch capture to compare mode
			TA1CCR1  += UART1_DIVIDER_HALF; // Point CCRx to middle of D0
		}
		else
		{
			rx1_byte >>= 1;
			if (TA1CCTL1 & SCCI)		// Get bit waiting in receive latch
				rx1_byte |= 0x80;	// Set highest bit
			rx1_bitcount--;
			if (rx1_bitcount == 0)		// Got all bits
			{
				TA1CCTL1 |= CAP;	// Switch back from compare to capture mode
				rx1_bitcount = 8;	// Re-load bit counter

				/* Lock the tx-queue to add one more byte */
				while (tx_lock);	// FIXME: This should be atomic test-and-set
				tx_lock = 1;

				/* Store received byte in transmission buffer WITH LOCK HELD */
				tx_buffer[tx_head] = rx1_byte;		// Add received byte
				tx_head = (tx_head + 1) % TX_CAPACITY;	// Switch to next free slot
				tx_count++;				// We have one more to tx
				if (tx_count >= TX_CAPACITY)
					P1OUT &= ~LED_RED;		// Turn red led off for overflow

				/* Release lock to tx-queue */
				tx_lock = 0;

				/* Schedule transmission in main loop */
				__bic_SR_register_on_exit(LPM0_bits);
			}
		}
	}
}
