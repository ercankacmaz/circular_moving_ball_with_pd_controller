#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


/*
 * Parameter
 */



/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define M_PI 3.14159265359
#define FC 3.0     				// Cutoff frequency in Hz
#define FS 100.0   				// Sampling frequency in Hz
 
#define RADIUS 80
#define CENTER_X 680 
#define CENTER_Y 320
#define CIRCLE_PERIOD 4

#define PWM_0_DEGREE        3800
#define PWM_90_DEGREE_X     3720
#define PWM_90_DEGREE_Y     3700
#define PWM_180_DEGREE      3600

#define KP_X	0.12
#define KP_Y	0.1
#define KD_X	3
#define KD_Y	2
/*
 * Global Variables
 */

 volatile uint16_t task_pending_counter = 0;
 volatile uint16_t deadline_misses = 0;
 volatile uint8_t current_axis = 0;
 volatile uint32_t time_elapsed = 0;
 volatile uint32_t circle_counter = 0;


/*
 * Timer Code
 */
void initialize_timer()
{
	CLEARBIT(T1CONbits.TON);
	CLEARBIT(T1CONbits.TCS);
	CLEARBIT(T1CONbits.TGATE);
	TMR1 = 0x00;
	T1CONbits.TCKPS = TCKPS_256;
	PR1 = 50;
	IPC0bits.T1IP = 0x01;
	CLEARBIT(IFS0bits.T1IF);
	SETBIT(IEC0bits.T1IE);
	SETBIT(T1CONbits.TON);
}


void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	task_pending_counter++;
    if (task_pending_counter >= 10){
        deadline_misses++;
    }

	time_elapsed++;
	if(time_elapsed % 20 == 0)
		circle_counter = time_elapsed;

    IFS0bits.T1IF = 0;
}

/*
 * Servo Code
 */

void servo_initialize()
{
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = TCKPS_64; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    PR2 = 4000; // Set timer period 20 ms:
    // 4000 = 20*10^-3 * 12.8*10^6 * 1/64
    // Setup OC8
    CLEARBIT(TRISDbits.TRISD8); // Set OC8 as output
    OC8R = PWM_90_DEGREE_X; // Set the initial duty cycle to 1 ms
    OC8RS = PWM_90_DEGREE_X; // Load OCRS: next pwm duty cycle
    OC8CON = 0x0006; // Set OC8: PWM, no fault check, Timer2
    SETBIT(T2CONbits.TON); // Turn Timer 2 on

    CLEARBIT(T3CONbits.TON); // Disable Timer
    CLEARBIT(T3CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T3CONbits.TGATE); // Disable Gated Timer mode
    TMR3 = 0x00; // Clear timer register
    T3CONbits.TCKPS = TCKPS_64; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T3IF); // Clear Timer3 interrupt status flag
    CLEARBIT(IEC0bits.T3IE); // Disable Timer3 interrupt enable control bit
    PR3 = 4000; // Set timer period 20 ms:
    // 4000 = 20*10^-3 * 12.8*10^6 * 1/64
    // Setup OC7
    CLEARBIT(TRISDbits.TRISD7); // Set OC7 as output
    OC7R = PWM_90_DEGREE_Y; // Set the initial duty cycle to 1 ms
    OC7RS = PWM_90_DEGREE_Y; // Load OCRS: next pwm duty cycle
    OC7CON = 0x0006; // Set OC7: PWM, no fault check, Timer3
    SETBIT(T3CONbits.TON); // Turn Timer 3 on

}

typedef enum {X_DIR,  Y_DIR} dir_t;

void set_pwm(dir_t dir, uint16_t duty_cycle)
{
	if (duty_cycle > PWM_0_DEGREE) 
		duty_cycle = PWM_0_DEGREE;
	if (duty_cycle < PWM_180_DEGREE) 
		duty_cycle = PWM_180_DEGREE;

    if (dir == X_DIR)
    {
        OC8RS = duty_cycle; // Load OCRS: next pwm duty cycle
    }
        
    if (dir == Y_DIR)
    {        
        OC7RS = duty_cycle; // Load OCRS: next pwm duty cycle
    }    
}


/*
 * Touch screen code
 */

void initialize_touchscreen()
{
    // Disable ADC
    CLEARBIT(AD1CON1bits.ADON);     // Disable ADC2
    //initialize pin
    SETBIT(TRISBbits.TRISB15);
    SETBIT(TRISBbits.TRISB9);
    CLEARBIT(AD1PCFGLbits.PCFG15);
    CLEARBIT(AD1PCFGLbits.PCFG9);
    CLEARBIT(AD1CON1bits.AD12B);
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0x7;
    AD1CON2 = 0; 
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F; 
    AD1CON3bits.ADCS = 0x2;
    SETBIT(AD1CON1bits.ADON);
    
    // Set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1); // I/O pin set to output
    CLEARBIT(TRISEbits.TRISE2); // I/O pin set to output
    CLEARBIT(TRISEbits.TRISE3); // I/O pin set to output
}

void set_direction(dir_t dir)
{
    if(dir == X_DIR)
    {
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop(); 
    }
    else if(dir == Y_DIR){
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();
    }
    //__delay_ms(10);
}


uint16_t read_positions()
{
    if (PORTEbits.RE1 == 0) {
        AD1CHS0bits.CH0SA = 0x0F; // AN15 for X
    } else if (PORTEbits.RE1 == 1){
        AD1CHS0bits.CH0SA = 0x09; // AN9 for Y
    }

    SETBIT(AD1CON1bits.SAMP);
    while (!AD1CON1bits.DONE);
    CLEARBIT(AD1CON1bits.DONE);
    return ADC1BUF0;
}



/*
 * PD Controller
 */

typedef struct {
	float Kp;				// Propotional gain
	float Kd;				// Derivative gain
	float prev_error;		// Previous error value for Derivative gain
} PD_CONTROLLER;

void PD_Init(PD_CONTROLLER *pd_val, float kp, float kd)
{
	pd_val->Kp = kp;
	pd_val->Kd = kd;
	pd_val->prev_error = 0.f;
}

float PD_Calculate(PD_CONTROLLER *pd_val, float ref_val, float read_val)
{
	float error, proportional, derivative;

	// Calculate the error
	error = ref_val - read_val;

	// Calculate the proportional term
	proportional = pd_val->Kp * error;

	// Calculate the derivative term
	derivative = pd_val->Kd * (error - pd_val->prev_error);

	// Update the previous error for the next iteration
	pd_val->prev_error = error;

	return proportional + derivative;
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */

// Structure to hold filter state and coefficients
typedef struct {
	float alpha;
    float beta0;
    float beta1;
	float state;			// filter state (previous output)
} Butterworth1P_t;

// Function to initialize the filter
void Butterworth1P_Init(Butterworth1P_t *filter, float cutoff_freq, float sample_rate)
{	// Calculate coefficients based on cutoff frequency and sample rate
	filter->alpha = 4.242 / 6.242;
    filter->beta0 = 1 / 6.242;
    filter->beta1 = 1 / 6.242;
	filter->state = 0.;
}

float apply_filter(Butterworth1P_t *filter, float input)
{
	float output = filter->beta0 * input + filter->beta1 * input + filter->alpha * filter->state;
	filter->state = output;

	return output;
}

void create_circle(uint32_t circle_val, float *x_ref, float *y_ref)
{
	float omega = 2. * M_PI / CIRCLE_PERIOD;
	*x_ref = CENTER_X + RADIUS * cos(omega * circle_val / 1000);
	*y_ref = CENTER_Y + RADIUS * sin(omega * circle_val / 1000);
}

/*
 * main loop
 */
void main_loop()
{
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: GroupName 12");
    lcd_locate(0, 2);
    
	initialize_timer();
	servo_initialize();
    initialize_touchscreen();

	PD_CONTROLLER x_pd_val; 
	PD_CONTROLLER y_pd_val;

	PD_Init(&x_pd_val, KP_X, KD_X);
	PD_Init(&y_pd_val, KP_Y, KD_Y);

	Butterworth1P_t filter_x;
	Butterworth1P_t filter_y;

	Butterworth1P_Init(&filter_x, FC, FS);
	Butterworth1P_Init(&filter_y, FC, FS);
    
	float raw_x = 0, raw_y = 0;
	float filtered_x = 0, filtered_y = 0;
	float ref_x = 0, ref_y = 0;
	float x_calculated = 0, y_calculated = 0;
    while(TRUE) {
		if(task_pending_counter < 10)
        {
			//continue;
        }
		
        else{
            task_pending_counter = 0;

            if(!current_axis) {
                raw_x = (float)read_positions();
                filtered_x = apply_filter(&filter_x, raw_x);
                set_direction(Y_DIR);
            }
            else if(current_axis) {
                raw_y = (float)read_positions();
                filtered_y = apply_filter(&filter_y, raw_y);
                set_direction(X_DIR);
            }
            current_axis = !current_axis;
            create_circle(circle_counter, &ref_x, &ref_y);

            x_calculated = PD_Calculate(&x_pd_val, ref_x, filtered_x);
            y_calculated = PD_Calculate(&y_pd_val, ref_y, filtered_y);

            set_pwm(X_DIR, PWM_90_DEGREE_X - (int)(x_calculated));
            set_pwm(Y_DIR, PWM_90_DEGREE_Y - (int)(y_calculated));
            
            lcd_locate(0, 7);
            lcd_printf("Miss: %u    ", deadline_misses);
            deadline_misses = 0;
        }
    }
}
