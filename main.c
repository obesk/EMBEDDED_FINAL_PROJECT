#include "adc.h"
#include "parser.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <xc.h>

#define CLOCK_LD1_TOGGLE 250 // 1 Hz (.5 high, .5 low)
#define CLOCK_BATT_PRINT 500 // 1 Hz 
#define CLOCK_IR_PRINT 50 // 10 Hz
#define CLOCK_ACC_READ 50 // 10 Hz

#define COUNT_T3_CALLS 500

/* Since we use a 10 bit UART transmission we use 10 bits for a byte of data.
Given the baud rate at 9600, and with the main executed at 500 Hz, we have 48 bytes per cycle */

#define INPUT_BUFF_LEN 48

/*  Considerations on fixed messages:

	- $MEMRG, 1* or $MEMRG, 0* -> same number of bytes; cannot occur simultneously 
	
	- $MACK, 1* or $MACK, 0* -> same number of bytes; cannot occur simultneously
		NB: this message is printed both for the $PCSTP,* message and for the $PCSTT,* one; these 2 messages cannot occur simultaneously

	Considerations on MBATT message:
	- $MBATT,* -> 8 bytes
	- vbatt is always positive and 2 decimals are printed (x.xx) -> 4 bytes

	Cosiderations on MDIST message:
	- $MDIST,* -> 8 bytes
	- dist is always positive and an integer of maximum 3 digits -> 3 bytes

	Considerations on MACC message:
	- $MACC,,,* -> 9 bytes
	- x values can assume either positive and negative values, 
		and at maximum requiring 4 bytes fot the absolute values -> 5 bytes (4 + 1 for the sign) 
	- same reasoning can be done for y and z

	NB: for each message, we have to add 1 byte to consider the \0 (end of string)

	Considering the worst case scenario, all the messages listed above can occur at the same time. 
	In this case, the buffer should be able to contain all of them. Its size should then be given by: 
	- $MEMRG, 1* or $MEMRG, 0* -> (9 + 1) = 10 bytes
	- $MACK, 1* or $MACK, 0* -> (8 + 1) = 9 bytes
	- $MBATT,* + vbatt -> (8 + 4 + 1) = 13 bytes 
	- $MDIST,* + dist -> (8 + 3 + 1) = 12 bytes 
	- $MACC,,,* + x + y + z -> (9 + 5 * 3 + 1) = 25 bytes 
	TOTAL : 69 bytes */

#define OUTPUT_BUFF_LEN 64

#define OBSTACLE_THRESHOLD_CM 30 // threshold for emergecy state

#define N_ADC_READINGS 10 // used to compute the mean value for sensors

enum RobotState { WAIT, MOVING, EMERGENCY }; // WAIT = 0, MOVING = 1, EMERGENCY = 2

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

struct ADCReading {
	float distance;
	float vbatt;
};

struct ADCReadings {
	int w;
	struct ADCReading readings[N_ADC_READINGS];
};

enum Axis {
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
};

void timers_init() {
	IFS0bits.T2IF = 0; // resetting the timer 2 flag
	IEC0bits.T2IE = 1; // enabling the timer 2 interrupt
	IFS0bits.T3IF = 0; // resetting the timer 3 flag
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
	tmr_wait_ms(TIMER4, 3); // waiting for the accelerometer to go into active state
}

int read_acc_axis(enum Axis axis) {
	CS_ACC = 0;
	// Starting from 0x02 + axis * 2, I read two consecutive registers
	spi_write((0x02 + axis * 2) | 0x80); // 2 registers for each axis -> starting form 0x02 for x, 0x04 for y and 0x06 for z
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

// Function to read distance and battery values using ADC
struct ADCReading read_adc(void) {
	struct ADCReading reading;

	while (!AD1CON1bits.DONE) {
		; // waiting until the conversion is ended
	}

	AD1CON1bits.SAMP = 0; // stopping the sampling and beginning the ADC conversion
	int adc0_raw_reading = ADC1BUF0; 
	double v_adc = (adc0_raw_reading / 1023.0) * 3.3; // assuming Vref+ = 3.3 V
	reading.vbatt = v_adc * 3;

	int adc2_raw_reading = ADC1BUF2; 
	double v_adc_ir = (adc2_raw_reading / 1023.0) * 3.3; // assuming Vref+ = 3.3 V
	reading.distance = (2.34 - 4.74 * v_adc_ir + 4.06 * pow(v_adc_ir, 2) - 1.6 * pow(v_adc_ir, 3) + 0.24 * pow(v_adc_ir, 4)) * 100;
	AD1CON1bits.SAMP = 1; // restarting the sampling 

	return reading;
}

enum RobotState robot_state = WAIT; // starting state of the system 
int count_t3_calls = 0;

int main(void) {
	// TODO: is this ok ?
	// FIXME: interrupts still triggering
	// IEC1bits.INT1IE = 0;
	TRISA = TRISG = ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

	const int main_hz = 500; 
	tmr_setup_period(TIMER1, 1000 / main_hz); // 100 Hz frequency for main function
	parser_state pstate = {.state = STATE_DOLLAR};

	// enabling the led for emergency state
	TRISBbits.TRISB8 = 0;
	TRISFbits.TRISF1 = 0;

	// enabling for IR sensor
	TRISBbits.TRISB9 = 0;
	LATBbits.LATB9 = 1; // IR enable

	// struct initialization
	struct ADCReadings ADC_readings;
	struct AccReading acc_reading = {0};

	// buffer and string initialization
	char output_str[25]; // 25 bytes = max lenght between all output messages
	char input_buff[INPUT_BUFF_LEN];
	char output_buff[OUTPUT_BUFF_LEN];

	UART_input_buff.buff = input_buff;
	UART_output_buff.buff = output_buff;

	// peripheral inizialization 
	pwm_init();
	button_init();
	timers_init();
	init_uart();
	init_spi();
	init_adc();

	activate_accelerometer();
	
	// counters and variables 
	int count_ld1_toggle = 0;
	int count_acc_read = 0;
	int count_ir_print = 0;
	int count_batt_print = 0;
	
	int distance = 0;

	// filling array readings of ADC_readings to correctly compute the mean afterwards
	for (int i = 0; i < N_ADC_READINGS; ++i) {
		ADC_readings.readings[i] = read_adc();
	}

	while (1) {

		// acquiring and printing the values read on the accelerometer
		if (++count_acc_read >= CLOCK_ACC_READ) {
			count_acc_read = 0;

			// Binding readings to reconstruct the values of accelerations along axis
			acc_reading.x = read_acc_axis(X_AXIS);
			acc_reading.y = read_acc_axis(Y_AXIS);
			acc_reading.z = read_acc_axis(Z_AXIS);

			sprintf(output_str, "$MACC,%d,%d,%d*", acc_reading.x, acc_reading.y, acc_reading.z);
			print_to_buff(output_str, &UART_output_buff);
		}

		// control of led toggoling (A0 and emergency ones)
		if (++count_ld1_toggle >= CLOCK_LD1_TOGGLE) {
			count_ld1_toggle = 0;
			LATAbits.LATA0 = !LATAbits.LATA0;
			if (robot_state == EMERGENCY) {
				LATBbits.LATB8 = !LATBbits.LATB8;
				LATFbits.LATF1 = !LATFbits.LATF1;
			}
		}

		// emercency state activation
		if (distance < OBSTACLE_THRESHOLD_CM) {
			if (robot_state != EMERGENCY) {
				print_to_buff("$MEMRG,1*", &UART_output_buff);
			}
			robot_state = EMERGENCY;
			count_t3_calls = 0;
			tmr_setup_period(TIMER3, 10);
		}

		// PWM management
		if (robot_state == MOVING) {
			pwm_start();
		} else {
			pwm_stop();
		}

		// adding the newest value of distance and battery readings from ADC to ADC_readings struct
		ADC_readings.readings[ADC_readings.w] = read_adc();
		ADC_readings.w = (ADC_readings.w + 1) % N_ADC_READINGS;

		// printing dist value after computing the mean with the last N_ADC_READINGS values read
		if (++count_ir_print >= CLOCK_IR_PRINT) {
			count_ir_print = 0;

			double dist = 0;
			for (int i = 0; i < N_ADC_READINGS; ++i) {
				dist += ADC_readings.readings[i].distance;
			}

			int idist = dist / N_ADC_READINGS;
			sprintf(output_str, "$MDIST,%d*", idist);
			print_to_buff(output_str, &UART_output_buff);
		}

		// printing v_batt value after computing the mean with the last N_ADC_READINGS values read
		if (++count_batt_print >= CLOCK_BATT_PRINT) {
			count_batt_print = 0;

			double v_batt = 0;
			for (int i = 0; i < N_ADC_READINGS; ++i) {
				v_batt += ADC_readings.readings[i].vbatt;
			}

			v_batt /= N_ADC_READINGS;

			sprintf(output_str, "$MBATT,%.2f*", v_batt);
			print_to_buff(output_str, &UART_output_buff);
		}

		// managing messages the microcontroller should recieve (in all states)
		while (UART_input_buff.read != UART_input_buff.write) {
			const int status = parse_byte(&pstate, UART_input_buff.buff[UART_input_buff.read]);
			if (status == NEW_MESSAGE) {
				if (strcmp(pstate.msg_type, "PCREF") == 0) { 
					// setting new speed to the system
					const int speed = extract_integer(pstate.msg_payload);
					const int next_arg = next_value(pstate.msg_payload, 1);
					const int yaw = extract_integer(&pstate.msg_payload[next_arg]);
					pwm_set_velocities(speed, yaw);
				} else if (strcmp(pstate.msg_type, "PCSTP") == 0) {  
					// switching the robot to the WAIT state + generating ACK msg
					if (robot_state != EMERGENCY) {
						print_to_buff("$MACK,1*", &UART_output_buff);
						robot_state = WAIT;
					} else {
						print_to_buff("$MACK,0*", &UART_output_buff);
					}
				} else if (strcmp(pstate.msg_type, "PCSTT") == 0) { 
					// switching the robot to the MOVING state + generating ACK msg
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

// interrupt for timer1 -> main
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(void) {
	IFS1bits.INT1IF = 0;
	tmr_setup_period(TIMER2, 10); // to manage the bouncing 
}

// interrupt for timer2 -> debouncing
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
	IFS0bits.T2IF = 0;
	T2CONbits.TON = 0; // stopping the timer

	if (robot_state == EMERGENCY) {
		return;
	}

	if (robot_state == WAIT) {
		robot_state = MOVING;
	} else if (robot_state == MOVING) {
		robot_state = WAIT;
	}
}

// interrupt when the emergency is ended -> going back to WAIT state
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
	IFS0bits.T3IF = 0; // resetting the flag

	if (++count_t3_calls > COUNT_T3_CALLS) {
		print_to_buff("$MEMRG,0*", &UART_output_buff);
		count_t3_calls = 0;
		T3CONbits.TON = 0;
		robot_state = WAIT;
	}
}

// interrupt when the U1TXREG is ready to send a new byte
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
	IFS0bits.U1TXIF = 0; // clear TX interrupt flag

	// control on the circular buffer indexes
	if (UART_output_buff.read == UART_output_buff.write) {
		UART_INTERRUPT_TX_MANUAL_TRIG = 1; // reset manual transmission flag
	}

	// until the buffer is full and there are still characters to be sent in the circular buffer
	while (!U1STAbits.UTXBF && UART_output_buff.read != UART_output_buff.write) {
		U1TXREG = UART_output_buff.buff[UART_output_buff.read]; // write the character on the UART register
		UART_output_buff.read = (UART_output_buff.read + 1) % OUTPUT_BUFF_LEN; // increment the read index 
	}
}

// interrupt generated when a new byte to the UART1 (new character available to be read)
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
	IFS0bits.U1RXIF = 0; // resetting the interrupt flag to 0

	while (U1STAbits.URXDA) { // until characters are available
		const char read_char = U1RXREG; // read from UART register

		// control and incrementation of the circular buffer write index
		const int new_write_index =	(UART_input_buff.write + 1) % INPUT_BUFF_LEN;
		if (new_write_index != UART_input_buff.read) {
			UART_input_buff.buff[UART_input_buff.write] = read_char;
			UART_input_buff.write = new_write_index;
		}
	}
}
