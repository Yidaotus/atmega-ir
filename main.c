/*
 * POT_PWM.c
 *
 * Author : Daniel Voigt
 */ 

#define F_CPU 16000000UL
#define UART_BAUD_RATE 60000

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "uart.h"

char buffer[64];
int count_ir_up, count_ir_down = 0;
unsigned char ir_prev = 0;
typedef enum IR_State
{
	IR_IDLE,
	IR_RECIEVING,
	IR_WAKEUP,
	IR_ACKNOWLEDGE,
	IR_UNKNOWN,
	IR_TIMEOUT,
	IR_HANDSHAKE_FAIL
} IR_STATE;

IR_STATE volatile ir_state = IR_IDLE;

char* ir_str = "Down: %d us\tUp: %d us\r\n";

int ir_result[2][32];
int out_results[2][32];
int volatile ir_result_i = 0;
int volatile ir_results = 0;

IR_STATE volatile ir_state_prev = IR_IDLE;
IR_STATE ir_history[50];
int ir_history_i = 0;

/**
 * A simple in software implementation of a simple IR protocol.
 * Works by sampling an IR diode with a timed interrupt of the atmega128
 * Very unoptimized and mainly used for educational purpose for learning
 * interrupt routines on the atmega128.
 */
int main(void)
{
	PORTD |= (1 << PORTD2);								//Port D 2 = Input + internal PullUp

	EICRA |= (1 << ISC01);								// Falling edge on PD2 = Interrupt
	EIMSK |= (1 << INT0);								// Enable INT0
	sei();															// Enable global Interrupts
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );				// Set Baudrate

    while (1) 
    {
		if (ir_state == IR_IDLE && ir_results > 0)
		{
			cli();
			int result_count = ir_results;
			memcpy(out_results, ir_result, 2*32*sizeof(int));
			ir_results = 0;
			sei();
			
			for (int i = 0; i < result_count; i++)
			{
				sprintf(buffer, "U:%4d D:%5d\t", out_results[0][i] * 10, out_results[1][i] * 10);
				uart_puts(buffer);
				if (!((i+1)%8)) {
					uart_puts("\r\n");
				} else {
					uart_puts(" | ");
				}
			}
			uart_puts("\r\n\r\n");
		} else {
			sei();	
		}
		IR_STATE state;
		for(int i = 0; i < ir_history_i; i++) {
			state = ir_history[i];
			switch (state)
			{
				case IR_IDLE:
					sprintf(buffer, "IR_IDLE\r\n");
					break;
				case IR_ACKNOWLEDGE:
					sprintf(buffer, "IR_ACKNOWLEDGE\r\n");
					break;
				case IR_WAKEUP:
					sprintf(buffer, "IR_WAKEUP\r\n");
					break;
				case IR_UNKNOWN:
					sprintf(buffer, "IR_UNKNOWN\r\n");
					break;
				case IR_RECIEVING:
					sprintf(buffer, "IR_RECIEVING\r\n");
					break;				
				case IR_TIMEOUT:
					sprintf(buffer, "IR_TIMEOUT\r\n");
					break;
				case IR_HANDSHAKE_FAIL:
					sprintf(buffer, "IR_HANDSHAKE_FAIL\r\n");
					break;
			}
			uart_puts(buffer);
		}
		_delay_ms(5000);
	}
}

/**
 *	Interrupt routine to sample the IR Diode.
 *
 */
ISR(TIMER0_COMPA_vect) {
	unsigned char ir = PIND & (1 << PIND2);
	if (ir_state == IR_RECIEVING || ir_state == IR_WAKEUP || ir_state == IR_ACKNOWLEDGE) {
		if(!ir) {
			count_ir_up++;
		} else {
			count_ir_down++;
		}
		if (ir_state == IR_RECIEVING && (count_ir_up > (600 / 20) || count_ir_down > (600 * 2 / 20))) {
			ir_results = ir_result_i;
			ir_result_i = 0;
			ir_state = IR_TIMEOUT;
			ir_history[ir_history_i++] = ir_state;
			TIMSK0 &= ~(1 << OCIE0A);	//Output Compare A Match Interrupt Disable
			TCCR0A = 0x00;
			EIMSK |= (1 << INT0);		//Enable INT0
			return;
		} else if ((ir_state == IR_WAKEUP && count_ir_up > 9100 / 20)	
				|| (ir_state == IR_ACKNOWLEDGE && count_ir_down > 4500 / 20)) {
			ir_state = IR_HANDSHAKE_FAIL;
			ir_history[ir_history_i++] = ir_state;
			TIMSK0 &= ~(1 << OCIE0A);	//Output Compare A Match Interrupt Disable
			TCCR0A = 0x00;
			EIMSK |= (1 << INT0);		//Enable INT0
			return;
		}
	}

	if (!ir && ir_prev) {
		count_ir_up = 0;
		if (ir_state == IR_RECIEVING) {
			ir_result[0][ir_result_i++] = count_ir_down;
		} else if (ir_state == IR_ACKNOWLEDGE && count_ir_down >= (4000 / 20)) { 
			ir_state = IR_RECIEVING;
			ir_history[ir_history_i++] = ir_state;
			count_ir_down = 0;
		} else {
			ir_state = IR_UNKNOWN;
			ir_history[ir_history_i++] = ir_state;
			ir_history[ir_history_i++] = ir_state;
			TIMSK0 &= ~(1 << OCIE0A);	//Output Compare A Match Interrupt Disable
			TCCR0A = 0x00;
			EIMSK |= (1 << INT0);		//Enable INT0
		}
	} else if(ir && !ir_prev) {
		count_ir_down = 0;
		if (ir_state == IR_RECIEVING) {
			ir_result[1][ir_result_i] = count_ir_up;
			count_ir_up = 0;
		} else if (ir_state == IR_WAKEUP && count_ir_up >= (8000 / 20)) {
			ir_state = IR_ACKNOWLEDGE;
			ir_history[ir_history_i++] = ir_state;
			count_ir_up = 0;
		} else {
			ir_state = IR_UNKNOWN;
			ir_history[ir_history_i++] = ir_state;
			TIMSK0 &= ~(1 << OCIE0A);	//Output Compare A Match Interrupt Disable
			TCCR0A = 0x00;
			EIMSK |= (1 << INT0);		//Enable INT0
		}
	}
	ir_prev = ir;
}


ISR(INT0_vect) {
	EIMSK &= ~(1 << INT0);		//Disable INT0
	EIFR &= ~(1 << INTF0);

	ir_state = IR_WAKEUP;
	ir_history[ir_history_i++] = ir_state;
	count_ir_up = 0;
	count_ir_down = 0;
	ir_prev = PIND & (1 << PIND2);

	//Setup the interrupt
	OCR0A = 40;									// 16000000 / (8* 40) ~ 50kH
	TCCR0B |= (1 << CS01);								// 8 prescaler
	TIMSK0 |= (1 << OCIE0A);							// Output Compare A Match Interrupt Enable
	TCCR0A |=  (1 << WGM01);							// CTC Top = OCR0A
	TCNT0 = 0x00;
}
