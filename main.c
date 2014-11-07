//
//  acrobat_temp.h
//
//
//  Created by Max Wasserman on 11/1/14.
//
//


#include <avr/io.h>
#include <math.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_imu.h"
#include "m_bus.h"
#include "m_rf.h"
#define BETA 0.75 //smoothing parameter: higher BETA make old value more significant
#define G_GAIN 16.4 // gryo scale---> sensitivity..... 0 --> 131 , 1--> 65.5, 2-->32.8, 3-->16.4 [LSB/dps] from data sheet of imu
#define A_GAIN 4096 //accel scale---> sensitivity.... 0 --> 16384 , 1--> 8192, 2-->4096, 3-->2048 [LSB/g] from data sheet of imu
#define x_omega_offset 3
#define y_omega_offset -1
#define x_acc_offset 410
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5


#include <string.h>
#include <stdio.h>





void init(void);
void motor_init (void);
void init_timer0(void);
void init_timer1(void);
void smooth(void);
void convertGyroToDps_print(void);
//void init_imu (void);
volatile int smooth_imu_values[9];
volatile int smooth_index = 0;
volatile float x_omega = 0.0;
volatile float y_omega = 0.0;
volatile float z_omega = 0.0;
volatile float x_acc = 0.0;
int imu_buffer[9];
float DT;
volatile int gyroXangle;



int main(void)
{
    init();
    m_bus_init();
    m_imu_init(2,3);
    motor_init();
    init_timer0();
    init_timer1();

    //  motors on
    // OCR0A = 200;

    while(1)
    {


    }
}


void init(void)
{
    //set system clock to 16MHz
    m_clockdivide(0);

    // enable global interrupts
    sei();

    //Disable Jtag
    m_disableJTAG();

    // initalize usb communiations
    m_usb_init();

    // init rf

}

void motor_init (void)
{  // set pin B7 as output  - note this problem is symettric. thus will only use one pin for both motors
    set(DDRB,7);   ///ENABLE PINS

    //MOTOR DIRECTION
    set(DDRB,3);

    // intially low
    clear(PORTB,3);



    // set speed of motors with pwm - adjusts time motor is in with enable to b7

    // slow down pwm: stay in hundreds Hz (~300)
}


void smooth(void)
{// digitally filters input according to BETA

 // for (smooth_index = 0; smooth_index < 9; smooth_index++)
 //   { //use old values in smooth_imu_values to smooth out new values in smooth_imu_values
 //       smooth_imu_values[smooth_index] = (float)BETA*smooth_imu_values[smooth_index] + (float)(1-BETA)*imu_buffer[smooth_index];
 //   }

   smooth_imu_values[AX] = (float)BETA*smooth_imu_values[AX] + (float)(1-BETA)*imu_buffer[AX];
   // smooth_imu_values[AY] = (float)BETA*smooth_imu_values[AY] + (float)(1-BETA)*imu_buffer[AY];
   smooth_imu_values[GX] = (float)BETA*smooth_imu_values[GX] + (float)(1-BETA)*imu_buffer[GX];

}

void init_timer0(void)

{  // set speed by adjusting OCR0A and OCR0B:  Vout = (OCR0n)/(255)*Vin

    // BE SURE D0 IS NOT MESSED UP BC OF THIS TIMER

    // set timer mode:UP to 0xFF, PWM mode
    clear(TCCR0B,WGM02);
    set(TCCR0A,WGM01);
    set(TCCR0A,WGM00);

    //  B7: OCR0A OUTPUT COMPARE: clear at OCR0A, set at 0xFF: Duty cycle - OCR0A/255
    set(TCCR0A, COM0A1);
    clear(TCCR0A, COM0A0);

    //initially duty cycle is 0 - motor is off
    OCR0A = 0;

    // enable global interrupts
    sei();

    // set prescaler to /8 = 2 Mhz
    clear(TCCR0B,CS02);
    set(TCCR0B,CS01);
    clear(TCCR0B,CS00);
}

void init_timer1(void)
{  // this timer will pull values from the imu when it overflows

    // set timer mode: (mode 4) UP to OCR1A
    clear(TCCR1B,WGM13);
    set(TCCR1B,WGM12);
    clear(TCCR1A,WGM11);
    clear(TCCR1A, WGM10);

    //update at  100Hz - clock/prescale
    OCR1A = 20000;

    // this is the DT that will be used
    // DT = OCR1A / Timer_Prescaler
    // DT = 20000 / 2*10^6;  // =  0.01 s = 10 ms  => should be loop  but i set it to timer period.  might be problem but prob not

    DT = (float) OCR1A/ (2*10^6);

    // enable global interrupts
    sei();

    // enable interrput when timer is OCR1A
    set(TIMSK1,OCIE1A);

    // set prescaler to /8 = 2 mhz
    clear(TCCR1B,CS12);
    set(TCCR1B,CS11);
    clear(TCCR1B,CS10);
}

char buf[100];
void send_float(float x) {
  sprintf(buf, "val: %f\n", ((float)123.45));
  int i;
  for (i=0; i< strlen(buf); i++) {
    m_usb_tx_char(*(buf+i));
  }
}

volatile float x_angle = 0;

void convertGyroToDps_print(void)
{ //convert gyro values to degrees per second

// smoothed
  /* x_omega = (float) smooth_imu_values[GX]/G_GAIN + x_omega_offset; */
  x_omega = (float) imu_buffer[GX]/G_GAIN + x_omega_offset;
/* x_acc = (float) smooth_imu_values[AX]/A_GAIN; */
  x_acc = imu_buffer[AX];

  /* send_float(x_acc); */
  x_angle = x_angle + ((int) x_omega * DT);

  /* send_float(x_angle); */

m_usb_tx_string(" Ax:  ");
m_usb_tx_int(x_omega);
/* m_usb_tx_int(x_acc); */
m_usb_tx_int(x_angle);
m_usb_tx_string("\n");

//git test

 // // track angle
 // gyroXangle += (float) x_omega * DT;  // way off

 // m_usb_tx_string(" Gx:  ");
 // m_usb_tx_int(x_omega);
 // m_usb_tx_string("      ");
 // m_usb_tx_string(" Theta:  ");
 // m_usb_tx_int(gyroXangle);
 // m_usb_tx_string("\n");




}


ISR(TIMER1_COMPA_vect)
{  // pull data form imu to buffer
    m_imu_raw(imu_buffer);
    smooth();
    convertGyroToDps_print();
    m_red(TOGGLE);


}


