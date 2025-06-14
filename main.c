#include "parser.h"
#include "pwm.h"
#include "timer.h"
#include "uart.h"

#include <string.h>
#include <xc.h>

#define CLOCK_LD1_TOGGLE 250

// TODO: size correctly and explain why
#define INPUT_BUFF_LEN 100
#define OUTPUT_BUFF_LEN 100

volatile struct circular_buffer UART_input_buff = {
	.len = INPUT_BUFF_LEN,
};

volatile struct circular_buffer UART_output_buff = {
	.len = OUTPUT_BUFF_LEN,
};

void timers_init() {
	IEC0bits.T2IE = 1; // enabling the timer 2 interrupt

	IFS0bits.T2IF = 0;
}

void button_init(void) {
	RPINR0bits.INT1R = 0x58; // remapping the interrupt 1 to the T2 button pin
	IFS1bits.INT1IF = 0;	 // resetting flag of interrupt 1
	IEC1bits.INT1IE = 1;	 // enabling interrupt 1
}

int main(void) {
	TRISA = TRISG = ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG =
		0x0000;

	char output_str[100]; // TODO: change with correct value
	char input_buff[INPUT_BUFF_LEN];
	char output_buff[OUTPUT_BUFF_LEN];

	UART_input_buff.buff = input_buff;
	UART_output_buff.buff = output_buff;

	pwm_init();
	button_init();
	timers_init();
	init_uart();

	move(RIGHT);

	int count_ld1_toggle = 0;

	const int main_hz = 500;
	tmr_setup_period(TIMER1, 1000 / main_hz); // 100 Hz frequency
	parser_state pstate = {.state = STATE_DOLLAR};
	while (1) {
		if (++count_ld1_toggle >= CLOCK_LD1_TOGGLE) {
			count_ld1_toggle = 0;
			LATA = !LATA;
		}

		while (UART_input_buff.read != UART_input_buff.write) {
			const int status =
				parse_byte(&pstate, UART_input_buff.buff[UART_input_buff.read]);
			if (status == NEW_MESSAGE) {
				if (strcmp(pstate.msg_type, "PCREF") == 0) {
					const int speed = extract_integer(pstate.msg_payload);
					const int next_arg = next_value(pstate.msg_payload, 1);
					const int yaw =
						extract_integer(&pstate.msg_payload[next_arg]);
					sprintf(output_str, "S:%d,Y:%d", speed, yaw);
					print_to_buff(output_str, &UART_output_buff);
				}
			}
			UART_input_buff.read = (UART_input_buff.read + 1) % INPUT_BUFF_LEN;
		}
		tmr_wait_period(TIMER1);
	}
	return 0;
}

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(void) {
	IFS1bits.INT1IF = 0;
	tmr_setup_period(TIMER2, 10);
}

int wait_state = 0;
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
	IFS0bits.T2IF = 0;
	T2CONbits.TON = 0;

	if (wait_state) {
		pwm_start();
	} else {
		pwm_stop();
	}
	wait_state = !wait_state;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
	IFS0bits.U1TXIF = 0; // clear TX interrupt flag

	if (UART_output_buff.read == UART_output_buff.write) {
		UART_INTERRUPT_TX_MANUAL_TRIG = 1;
	}

	while (!U1STAbits.UTXBF &&
		   UART_output_buff.read != UART_output_buff.write) {
		U1TXREG = UART_output_buff.buff[UART_output_buff.read];
		UART_output_buff.read = (UART_output_buff.read + 1) % OUTPUT_BUFF_LEN;
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
	IFS0bits.U1RXIF = 0; // resetting the interrupt flag to 0

	while (U1STAbits.URXDA) {
		const char read_char = U1RXREG;

		const int new_write_index =
			(UART_input_buff.write + 1) % INPUT_BUFF_LEN;
		if (new_write_index != UART_input_buff.read) {
			UART_input_buff.buff[UART_input_buff.write] = read_char;
			UART_input_buff.write = new_write_index;
		}
	}
}
