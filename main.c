//
//  main.c
//
//  Created by Max Wasserman on 11/1/14.
//

#include <avr/io.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_imu.h"

#define SYSTEM_CLOCK 16*pow(10,6)
#define POLLING_FREQ 100

#define ACC_SCALE 1
#define GYRO_SCALE 0

#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

#define ALPHA 0.98
#define OMEGA_X_OFFSET 3
#define RAD_TO_DEG 57.29578

#define Kp 0.25
#define Kd 0.1
#define Ki 0.01

#define DEBUG 1

//git test

void init();
void init_timer1();
void estimate_theta();
void send_float(char *label, float value);
void motor_init();
void init_timer0();
void set_motor_input_voltage(float voltage);

int imu_buf[9];
float G_GAIN[4] = {131, 65.5, 32.8, 16.4}; //[LSB/dps]
float A_GAIN[4] = {16384, 8192, 4096, 2048}; //[LSB/g]
float DT = 1.0/POLLING_FREQ;

float omega_x = 0;
float acc_angle_x = 0;
float gyro_angle_x = 0;
float cf_angle_x = 0;
float priv_cf_angle_x = 0;
float tgt_volt = 0;

volatile int new_imu_values_flag = 0;
int poll_count = 0;
int angle_sum = 0;
float init_angle = 0;
float integral_term = 0;

int main()
{
  init();
  m_imu_init(ACC_SCALE, GYRO_SCALE);
  motor_init();
  init_timer0();
  init_timer1();

  while(1) {
    if (new_imu_values_flag) {
      m_red(TOGGLE);
      m_imu_raw(imu_buf);
      estimate_theta();
      tgt_volt = Kp * cf_angle_x;
      send_float("V_prop", tgt_volt);
      tgt_volt += Kd * (cf_angle_x - priv_cf_angle_x)/DT;
      send_float("V_deriv", Kd * (cf_angle_x - priv_cf_angle_x)/DT);
      if (init_angle != 0) {
        integral_term += (cf_angle_x - init_angle)*DT;
        tgt_volt += Ki*integral_term;
        send_float("V_int", Ki*integral_term);
      }
      set_motor_input_voltage(tgt_volt);
      new_imu_values_flag = 0;
    }
  }
}

void init()
{
  //set system clock to 16MHz
  m_clockdivide(0);

  // enable global interrupts
  sei();

  //Disable JTAG
  m_disableJTAG();

  // initalize usb communiations
  if (DEBUG) {
    m_usb_init();
  }
}

// this timer will pull values from the imu when it overflows
void init_timer1()
{
  float timer_freq;

  // set prescaler to /8
  clear(TCCR1B,CS12);
  set(TCCR1B,CS11);
  clear(TCCR1B,CS10);

  // timer freq = 16MHz /8 = 2MHz
  timer_freq = SYSTEM_CLOCK/8;

  // set timer mode: (mode 4) UP to OCR1A
  clear(TCCR1B,WGM13);
  set(TCCR1B,WGM12);
  clear(TCCR1A,WGM11);
  clear(TCCR1A, WGM10);

  // set OCR1A so that interrupt would occur at POLLING_FREQ.
  OCR1A =  timer_freq/POLLING_FREQ;

  // enable global interrupts
  sei();

  // enable interrput when timer is OCR1A
  set(TIMSK1,OCIE1A);
}

void estimate_theta()
{
  acc_angle_x = atan2(imu_buf[AX], imu_buf[AZ]) * RAD_TO_DEG;

  // Gives same result as above for small angles.
  // theta_x = a_x/g
  // acc_angle_x = imu_buf[AX]/A_GAIN[ACC_SCALE] * RAD_TO_DEG);

  omega_x = imu_buf[GX]/G_GAIN[GYRO_SCALE] + OMEGA_X_OFFSET;

  // This value simply drifts away from zero.
  // gyro_angle_x = gyro_angle_x + omega_x*DT;

  // Store the previous cf angle for later derivative calculation.
  priv_cf_angle_x = cf_angle_x;

  // In the long term cf_angle converges to acc_angle, but in the short term
  // is highly influenced by omega_x * DT
  cf_angle_x = ALPHA * (cf_angle_x + omega_x*DT) + (1-ALPHA)*acc_angle_x;

  usb_tx_string("-----------------------\n");
  send_float("acc_angle", acc_angle_x);
  send_float("omega", omega_x);
  send_float("cf_angle", cf_angle_x);

  // Average the cf angle for two seconds to set the initial angle.
  if (poll_count < 2*POLLING_FREQ) {
    angle_sum += cf_angle_x;
    poll_count++;
  } else if (init_angle == 0) {
    init_angle = angle_sum/(2.0*POLLING_FREQ);
    send_float("init_angle", init_angle);
  }
}

// pull data from imu to buf
ISR(TIMER1_COMPA_vect)
{
  new_imu_values_flag = 1;
}

char buf[100];
void send_float(char *label, float value) {
  if (DEBUG) {
    sprintf(buf, "%s: %.3f\n", label, value);
    int i;
    for (i=0; i< strlen(buf); i++) {
      m_usb_tx_char(*(buf+i));
    }
  }
}

void motor_init()
{
  ///ENABLE PIN
  set(DDRB,7);

  //MOTOR DIRECTION PIN
  set(DDRB,3);

  // set speed of motors with pwm - adjusts time motor is in with enable to b7

  // slow down pwm: stay in hundreds Hz (~300)
}

// set motor input voltage by adjusting OCR0A:  Vout = (OCR0n)/(255)*Vin
void init_timer0()
{
  // BE SURE D0 IS NOT MESSED UP BC OF THIS TIMER

  // set prescaler to /256
  // timer freq = 16MHz /256 = 62.5kHz
  set(TCCR0B,CS02);
  clear(TCCR0B,CS01);
  clear(TCCR0B,CS00);

  // set timer mode: UP to 0xFF, PWM mode
  // pwm freq = timer freq /256 = 244Hz
  clear(TCCR0B,WGM02);
  set(TCCR0A,WGM01);
  set(TCCR0A,WGM00);

  //  B7: OCR0A OUTPUT COMPARE: clear at OCR0A, set at 0xFF: Duty cycle - OCR0A/255
  set(TCCR0A, COM0A1);
  clear(TCCR0A, COM0A0);

  //initially duty cycle is 0 - motor is off
  OCR0A = 0;
}

void set_motor_input_voltage(float voltage) {
  int target_OCR0A = 51 * fabs(voltage);
  if (target_OCR0A > 255) {
    OCR0A = 255;
  } else {
    OCR0A = target_OCR0A;
  }
  if (voltage > 0) {
    set(PORTB, 3);
  } else {
    clear(PORTB, 3);
  }
  send_float("motor voltage", voltage);
  send_float("OCR0A", OCR0A);
  send_float("PORTB", check(PORTB,3));
}

