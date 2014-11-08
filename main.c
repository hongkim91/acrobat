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
#define BETA 0.7 //smoothing parameter: higher BETA make old value more significant
#define BETA_COMP 0.1
#define G_GAIN 131 // gryo scale---> sensitivity..... 0 --> 131 , 1--> 65.5, 2-->32.8, 3-->16.4 [LSB/dps] from data sheet of imu
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
#define g 9.8
#define M_PI 3.14159265358979323846
#define RAD_TO_DEG 57.29578

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
volatile float y_acc = 0.0;
volatile float z_acc = 0.0;
volatile float dt_a = 0.0;
int imu_buffer[9];

float DT;
volatile int gyroXangle;
volatile float old_fst_x_angle = 0;
volatile float fst_x_angle = 0;
volatile float low_x_angle = 0;
volatile float high_x_angle = 0;
volatile float x_angle = 0;
volatile float CFangleX = 0;
volatile float CFangleY = 0;
volatile float AccXangle= 0;
volatile float AccYangle = 0;



int main(void)
{
    init();
    m_bus_init();
    m_imu_init(0,0);
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
    dt_a = OCR1A + 0.01;
    // this is the DT that will be used
    // DT = OCR1A / Timer_Prescaler
    // DT = 20000 / 2*10^6;  // =  0.01 s = 10 ms  => should be loop  but i set it to timer period.  might be problem but prob not

    DT = (float) dt_a/ ((2*10^6)+0.01);

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


void convertGyroToDps_print(void)
{ //convert gyro values to degrees per second

// smoothed

  x_omega = (float)imu_buffer[GX]/G_GAIN + x_omega_offset;
  y_omega = (float)imu_buffer[GY]/G_GAIN + y_omega_offset;

  x_acc = (float)imu_buffer[AX];
  y_acc = (float)imu_buffer[AY];
  z_acc = (float)imu_buffer[AZ];

  AccXangle = (float) (atan2(y_acc, z_acc) + M_PI) * RAD_TO_DEG;
  AccYangle = (float) (atan2(z_acc, x_acc) + M_PI) * RAD_TO_DEG;

  CFangleX= BETA_COMP*(CFangleX+x_omega*DT) +(1 - BETA_COMP) * AccXangle;
  CFangleY= BETA_COMP*(CFangleY+y_omega*DT) +(1 - BETA_COMP) * AccYangle;




//   old_fst_x_angle = fst_x_angle;
//   fst_x_angle = (old_fst_x_angle + (x_omega) * DT);


// //HIGH PASS
// // new changes in angle approximation via the angular velocity has a large influence
//                       // last value                   // diff in angle approx via ang velocity
//   high_x_angle = (BETA_COMP)*high_x_angle + BETA_COMP*(fst_x_angle - old_fst_x_angle);

// //LOW PASS

//    low_x_angle = BETA_COMP*(low_x_angle) + (1-BETA_COMP)*(-z_acc/g);

// // Theta ~= HIGH PASS EST + LOW PASS EST

//    x_angle = high_x_angle + low_x_angle;




  /* send_float(x_angle); */

//m_usb_tx_string(" X_angle:  ");
//m_usb_tx_int(x_angle);
// m_usb_tx_string(" x_omega:  ");
// m_usb_tx_int(x_omega);

// m_usb_tx_string(" fst_x_angle:  ");
// m_usb_tx_int(fst_x_angle);
// m_usb_tx_string(" x_omega:  ");
// m_usb_tx_int(x_omega);

// m_usb_tx_string(" z_acc :  ");
// m_usb_tx_int(z_acc);

// m_usb_tx_string(" x_acc :  ");
// m_usb_tx_int(x_acc);


m_usb_tx_string(" CFangleX:  ");
m_usb_tx_int(CFangleX);

m_usb_tx_string(" AccXangle:  ");
m_usb_tx_int(AccXangle);

m_usb_tx_string(" x_omega:  ");
m_usb_tx_int(x_omega);


// m_usb_tx_string(" Y_angle:  ");
// m_usb_tx_int(CFangleY);
/* m_usb_tx_int(x_acc); */
// m_usb_tx_int(x_angle);
m_usb_tx_string("\n");

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
