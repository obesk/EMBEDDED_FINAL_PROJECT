#include "parser.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"

#include <string.h>
#include <xc.h>

#define CLOCK_LD1_TOGGLE 250
#define CLOCK_BATT_PRINT 500
#define CLOCK_IR_PRINT 50
#define CLOCK_ACC_READ 50

// TODO: size correctly and explain why
#define INPUT_BUFF_LEN 100
#define OUTPUT_BUFF_LEN 100

#define OBSTACLE_THRESHOLD_CM 0

enum RobotState { WAIT, MOVING, EMERGENCY };

volatile struct circular_buffer UART_input_buff = {
	.len = INPUT_BUFF_LEN,
};

volatile struct circular_buffer UART_output_buff = {
	.len = OUTPUT_BUFF_LEN,
};
struct AccReading {
	int x;
	int y;
	int z;
};

enum Axis {
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
};

void timers_init() {
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1; // enabling the timer 2 interrupt
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 1; // enabling the timer 3 interrupt
}

void button_init(void) {
	RPINR0bits.INT1R = 0x58; // remapping the interrupt 1 to the T2 button pin
	IFS1bits.INT1IF = 0;	 // resetting flag of interrupt 1
	IEC1bits.INT1IE = 1;	 // enabling interrupt 1
}

void activate_accelerometer() {
	// Selecting the accelerometer and disabling magnetometer and gyroscope
	CS_MAG = 1;
	CS_GYR = 1;

	CS_ACC = 0;
	spi_write(0x11);
	spi_write(0x00); // changing the accelerometer to active state
	CS_ACC = 1;
	tmr_wait_ms(TIMER4,
				3); // waiting for the accelerometer to go into active state
}

int read_acc_axis(enum Axis axis) {
	CS_ACC = 0;
	// Startin from a specific register, I read twice
	spi_write((0x02 + axis * 2) |
			  0x80); // 2 registers for each axis -> starting form 0x02 for x,
					 // 0x04 for y and 0x06 for z
	uint8_t axis_lsb = spi_write(0x00);
	uint8_t axis_msb = spi_write(0x00);
	CS_ACC = 1;

	// Combining readings to reconstruct the real value of accelerations
	int axis_value = (((axis_msb << 4) | (axis_lsb & 0x0F)));
	if (axis_value & 0x800) {
		axis_value |= 0xF000;
	}

	return axis_value;
}

enum RobotState robot_state = WAIT;

int main(void) {
	// TODO: is this ok ?
	// FIXME: interrupts still triggering
	// IEC1bits.INT1IE = 0;
	TRISA = TRISG = ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG =
		0x0000;

	// TODO: check if needed (emergency)
	TRISBbits.TRISB8 = 0;
	TRISFbits.TRISF1 = 0;
	TRISBbits.TRISB9 = 0;
	LATBbits.LATB9 = 1; // IR enable

	struct AccReading acc_reading = {0};

	char output_str[100]; // TODO: change with correct value
	char input_buff[INPUT_BUFF_LEN];
	char output_buff[OUTPUT_BUFF_LEN];

	UART_input_buff.buff = input_buff;
	UART_output_buff.buff = output_buff;

	pwm_init();
	button_init();
	timers_init();
	init_uart();
	init_spi();
	init_adc();

	int dist;
	double v_adc_batt;
	// IEC1bits.INT1IE = 1;

	int count_ld1_toggle = 0;
	int count_acc_read = 0;
	int count_ir_print = 0;
	int count_batt_print = 0;

	const int main_hz = 500;
	tmr_setup_period(TIMER1, 1000 / main_hz); // 100 Hz frequency
	parser_state pstate = {.state = STATE_DOLLAR};

	int distance = 0;

	activate_accelerometer();

	while (1) {

		if (++count_acc_read >= CLOCK_ACC_READ) {
			count_acc_read = 0;

			// Binding readings to reconstruct the values of accelerations along
			// axis
			acc_reading.x = read_acc_axis(X_AXIS);
			acc_reading.y = read_acc_axis(Y_AXIS);
			acc_reading.z = read_acc_axis(Z_AXIS);

			sprintf(output_str, "$MACC,%d,%d,%d*", acc_reading.x, acc_reading.y,
					acc_reading.z);
			print_to_buff(output_str, &UART_output_buff);
		}

		if (++count_ld1_toggle >= CLOCK_LD1_TOGGLE) {
			count_ld1_toggle = 0;
			LATAbits.LATA0 = !LATAbits.LATA0;
			if (robot_state == EMERGENCY) {
				LATBbits.LATB8 = !LATBbits.LATB8;
				LATFbits.LATF1 = !LATFbits.LATF1;
			}
		}

		if (distance < OBSTACLE_THRESHOLD_CM) {
			robot_state = EMERGENCY;
			print_to_buff("$MEMRG,1*", &UART_output_buff);
			tmr_setup_period(TIMER3, 5);
		}

		if (robot_state == MOVING) {
			pwm_start();
		} else {
			pwm_stop();
		}

		if (++count_ir_print >= CLOCK_IR_PRINT) {
			count_ir_print = 0;

			sprintf(output_str, "$MDIST,%d*", dist);
			print_to_buff(output_str, &UART_output_buff);
		}

		if (++count_batt_print >= CLOCK_BATT_PRINT) {
			count_batt_print = 0;

			sprintf(output_str, "$MBATT,%.2f*", v_adc_batt);
			print_to_buff(output_str, &UART_output_buff);
		}

		while (!AD1CON1bits.DONE) {
			;
		}

		AD1CON1bits.SAMP = 0;
		int adcb = ADC1BUF0;
		double v_adc = (adcb / 1023.0) * 3.3; // assuming Vref+ = 3.3 V
		v_adc_batt = v_adc * 3;

		int adcff = ADC1BUF2;
		double v_adc_ir = (adcff / 1023.0) * 3.3; // assuming Vref+ = 3.3 V
		dist = (2.34 - 4.74 * v_adc_ir + 4.06 * pow(v_adc_ir, 2) -
				1.6 * pow(v_adc_ir, 3) + 0.24 * pow(v_adc_ir, 4)) *
			   100;
		AD1CON1bits.SAMP = 1;

		while (UART_input_buff.read != UART_input_buff.write) {
			const int status =
				parse_byte(&pstate, UART_input_buff.buff[UART_input_buff.read]);
			if (status == NEW_MESSAGE) {
				if (strcmp(pstate.msg_type, "PCREF") == 0) {
					const int speed = extract_integer(pstate.msg_payload);
					const int next_arg = next_value(pstate.msg_payload, 1);
					const int yaw =
						extract_integer(&pstate.msg_payload[next_arg]);
					pwm_set_velocities(speed, yaw);
				} else if (strcmp(pstate.msg_type, "PCSTP") == 0) {
					if (robot_state != EMERGENCY) {
						print_to_buff("$MACK,1*", &UART_output_buff);
						robot_state = WAIT;
					} else {
						print_to_buff("$MACK,0*", &UART_output_buff);
					}
				} else if (strcmp(pstate.msg_type, "PCSTT") == 0) {
					if (robot_state != EMERGENCY) {
						print_to_buff("$MACK,1*", &UART_output_buff);
						robot_state = MOVING;
					} else {
						print_to_buff("$MACK,0*", &UART_output_buff);
					}
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

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
	IFS0bits.T2IF = 0;
	T2CONbits.TON = 0;

	if (robot_state == EMERGENCY) {
		return;
	}

	if (robot_state == WAIT) {
		robot_state = MOVING;
	} else if (robot_state == MOVING) {
		robot_state = WAIT;
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
	robot_state = WAIT;
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
