#include "xc.h"
#include "pwm.h"

#define FCY 72000000UL
#define PWM_FREQ 10000UL
#define PWM_PERIOD_TICKS (FCY/PWM_FREQ)

void pwm_init(void) {
	// No sync or trigger source for input caputure
	OC1CON2bits.SYNCSEL = 0x1F;
	OC2CON2bits.SYNCSEL = 0x1F;
	OC3CON2bits.SYNCSEL = 0x1F;
	OC4CON2bits.SYNCSEL = 0x1F;

	// Setting peripheral clock 1/2 of FOSC
	OC1CON1bits.OCTSEL = 7;
	OC2CON1bits.OCTSEL = 7;
	OC3CON1bits.OCTSEL = 7;
	OC4CON1bits.OCTSEL = 7;

	// Setting the period of the PWM
	OC1RS = PWM_PERIOD_TICKS;
	OC2RS = PWM_PERIOD_TICKS; 
	OC3RS = PWM_PERIOD_TICKS; 
	OC4RS = PWM_PERIOD_TICKS; 

	// remapping pins for output compare
    RPOR0bits.RP65R = 0b010000;
    RPOR1bits.RP66R = 0b010001;
    RPOR1bits.RP67R = 0b010010;
    RPOR2bits.RP68R = 0b010011;

	// setting the PWM pins as output
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;
}

void pwm_set_velocities(int forward_speed, int yaw_rate) {
	if (forward_speed < -100 || forward_speed > 100 || yaw_rate < -100 || yaw_rate > 100) {
		return;
	}
	
	int right_speed = forward_speed + yaw_rate;
	int left_speed = forward_speed - yaw_rate;

	if (right_speed > 100) {
		left_speed -= right_speed % 101;
		right_speed = 100;
	} else if (right_speed < -100) {
		left_speed += right_speed % 101;
		right_speed = -100;
	}

	if (left_speed > 100) {
		right_speed -= left_speed % 101;
		left_speed = 100;
	} else if (left_speed < -100) {
		right_speed += left_speed % 101;
		left_speed = -100;
	}

	//TODO: use abs function
	int abs_right = right_speed > 0 ? right_speed : -right_speed;
	int abs_left = left_speed > 0 ? left_speed : -left_speed;

	if (right_speed > 0) {
		OC4R = comp_duty(abs_right);
		OC3R = 0;
	} else {
		OC3R = comp_duty(abs_right);
		OC4R = 0;
	}
	if (left_speed > 0) {
		OC2R = comp_duty(abs_left);
		OC1R = 0;
	} else {
		OC1R = comp_duty(abs_left);
		OC2R = 0;
	}
}

void pwm_start(void) {
	// edge-aligned pwm mode
    OC1CON1bits.OCM = 0b110;
    OC2CON1bits.OCM = 0b110;
    OC3CON1bits.OCM = 0b110;
    OC4CON1bits.OCM = 0b110;
}

void pwm_stop(void) {
    OC1CON1bits.OCM = 0;
    OC2CON1bits.OCM = 0;
    OC3CON1bits.OCM = 0;
    OC4CON1bits.OCM = 0;
}

int comp_duty(int percentage) {  
	// computing real velocity value
    return (PWM_PERIOD_TICKS * percentage) / 100;
}

