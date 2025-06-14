#include "pwm.h"
#include "timer.h"

#include <xc.h>

#define CLOCK_LD1_TOGGLE 250

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

	pwm_init();
	button_init();
	timers_init();

	move(RIGHT);

	int count_ld1_toggle = 0;

	const int main_hz = 500;
	tmr_setup_period(TIMER1, 1000 / main_hz); // 100 Hz frequency
	while (1) {
		if (++count_ld1_toggle >= CLOCK_LD1_TOGGLE) {
			count_ld1_toggle = 0;
			LATA = !LATA;
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
