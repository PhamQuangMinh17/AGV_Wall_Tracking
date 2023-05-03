/*
 Pham Quang Minh - Mobile robot - wall following.
*/
// Define a macro for the delay.h header file. F_CPU is the microcontroller frequencyvalue for the delay.h header file
// Default value of F_CPU in delay.h header file is 1000000(1MHz)*/
#ifndef F_CPU
#define F_CPU 16000000UL // Set 16 MHz clock speed
#endif

#include <avr/io.h> // This header defines all the Input/Output Registers for all AVRmicrocontrollers*/
#include <util/delay.h> ///This header file defines two functions, _delay_ms (milliseconddelay) and _delay_us (microsecond delay)*/

void motor_control_PID(int control_number, int pwm,int PID_mode);
int PID_wall(int w_status ,float distance_right_2);
void motor_control(int control_number, int pwm);

// port set up for ultrasonic sensor 
#define trig_right_1 14
#define echo_right_1 15
#define trig_right_2 16
#define echo_right_2 17
#define trig_front_r 18
#define echo_front_r 19
#define trig_front_l 7
#define echo_front_l 6

void robot_sensor();

float distance_right_1, distance_right_2, distance_front_r, distance_front_l ;
float duration_right_1, duration_right_2, duration_front_r, duration_front_l;

// DC MOTOR
int ENA = 10, IN1 = 8, IN2 = 9;// motor right
int ENB = 11, IN3 = 12, IN4 = 13;// motor left

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define pulse_per_rev 225
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define Encoder_right_C1 2
#define Encoder_left_C2 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define Encoder_right_C2 4
#define Encoder_left_C1 5
 
// True = Forward; False = Reverse
boolean Direction_right = true;
boolean Direction_left = true;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;
  
// Counters for milliseconds during interval
long prev_ms = 0;
long current_ms = 0;
// One-second interval for measurements
int interval = 48;

// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;
 
const float d_wheel = 0.065; // 6.5cm = 0.065 m;

// Robot velocity:
float right_wheel_speed = 0;
float left_wheel_speed = 0;
float robot_speed = 0;
int constant_pwm = 70;

// Variable for PID set up
long PID_prev_time = 0;
long PID_current_time = 0;
float delta_T;

float e_wall;
float e_prev_wall = 0;
float e_int_wall = 0;
float derivative_wall;
float u_wall; // control signal

/// PID constant for wall tracking
float kp_w = 2;
float ki_w = 0.1;
float kd_w = 0;

float e_angle;
float e_prev_angle = 0;
float e_int_angle = 0;
float derivative_angle;
float u_angle; // control signal
/// PID constant for angle tracking
float kp_a = 1;
float ki_a = 0;
float kd_a = 0;

// PID constant for turn right tracking
float kp_r = 1;
float ki_r = 1;
float kd_r = 0;

const int target_wall = 15;
float wall_offset = 2;
float angle_Check;
float angle_offset = 1;

float error_wall_2;
float error_wall_1;
float wall_max = 60;
typedef enum {start_up,wall_following, manual_control}STATES ;
typedef enum {check_front, check_right,right_turn, right_follow_PID, obstacle_ahead}WALL_FOLLOWING;
STATES robot_mode;
WALL_FOLLOWING wall_mode;


void setup()
{
  robot_mode = start_up;
  wall_mode = check_front;

  // Set up trigger pin as output and echo pin input
  pinMode(trig_right_1, OUTPUT); // Sets the trigger_left as an Output
  pinMode(echo_right_1, INPUT); // Sets the echo_left as an Input
  pinMode(trig_right_2, OUTPUT); // Sets the trigger_left as an Output
  pinMode(echo_right_2, INPUT); // Sets the echo_left as an Input
  pinMode(trig_front_r, OUTPUT); // Sets the trigger_left as an Output
  pinMode(echo_front_r, INPUT); // Sets the echo_left as an Input
  pinMode(trig_front_l, OUTPUT); // Sets the trigger_left as an Output
  pinMode(echo_front_l, INPUT); // Sets the echo_left as an Input
  /*
   // set all the motor control pins to outputs and set all trigger 
   pin as output and echo pin as input
   */
  DDRB = 0xFF; // Port B as output
  Serial.begin(9600);
//  // Set pin states of the encoder
//  pinMode(Encoder_right_C1 , INPUT_PULLUP);
//  pinMode(Encoder_right_C2 , INPUT);
//  
//  pinMode(Encoder_left_C1 , INPUT_PULLUP);
//  pinMode(Encoder_left_C2 , INPUT);
// 
//  // Every time the pin goes high, this is a pulse
//  attachInterrupt(digitalPinToInterrupt(Encoder_right_C1), right_wheel_pulse, RISING);
//  attachInterrupt(digitalPinToInterrupt(Encoder_left_C2), left_wheel_pulse, RISING);
}
void loop()
{
  robot_sensor();
//  PID_wall(1, distance_right_2);
//  PID_angle(1, distance_right_2, distance_right_1);
//  Serial.print("PID_wall:  ");
//  Serial.println(PID_wall(1, distance_right_2));
//  Serial.print("PID_angle:  ");
//  Serial.println(PID_angle(1, distance_right_2, distance_right_1));
//  angle_Check = distance_right_2 - distance_right_1;


 switch(robot_mode)
  {
    case start_up:
      delay(1000);
      robot_mode = wall_following;
      break;
    case check_right:
      switch(wall_mode)
      {
        case check_front:
          if(distance_front_r > 10 && distance_front_l > 15)
          {
            wall_mode = right_follow_PID;
          }
          else if(distance_front_r <= 10 && distance_front_l > 15)
          {
            wall_mode = obstacle_ahead;
          }
          else if(distance_front_r > 10 && distance_front_l <= 15)
          {
             wall_mode = obstacle_ahead;
          }
          else
          {
            wall_mode = obstacle_ahead;
          }
          break;
        case check_right:

          break;
        case right_turn:
        
          break;
        case right_follow_PID:
          if(distance_right_2 <= target_wall)
          {
            motor_control_PID(2,constant_pwm, 1);
          }
          else if(distance_right_2 > target_wall)
          {
            motor_control_PID(3,constant_pwm, 1);
          }
          wall_mode = check_front;
          break;
        case obstacle_ahead:
          PID_wall(0, distance_right_2);
          motor_control_PID(0,0,1);
          delay(1000);
          motor_control(4,constant_pwm);
          delay(1000);
          motor_control(0,0);
          delay(500);
          motor_control(2,110);
          delay(500);
          
          wall_mode = check_front;
          break;
        default:
          wall_mode = check_front;
          break;
      }
      break;
    case manual_control:
      break;
    default:
      break;
  }
}

void motor_control(int control_number, int pwm)
{
  if (pwm > 255)
  {
    pwm = 255;
  }
  else if(pwm < 0)
  {
    pwm = 0;
  }
  
  if(control_number == 0)
  {
    // motor stop
    PORTB &= ~(3<<4);
    PORTB &= ~(3<<0);
  }
  else if(control_number == 1)
  {
    // go forward
        // Turn on motor right
    PORTB |= (1<<1);//  digitalWrite(IN1, HIGH);
    PORTB &= ~(1<<0); //  digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
   
     // turn on motor left
    PORTB |= (1<<4); //  digitalWrite(IN3, HIGH);
    PORTB &= ~ (1<<5); //  digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm );
  }
  else if(control_number == 2)
  {
    // turn left
    /*
    Motor left stop 
    Motor right run
    */
    // Motor right run
    PORTB |= (1<<1);//  digitalWrite(IN1, HIGH);
    PORTB &= ~(1<<0); //  digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
    
    // Motor left stop
    PORTB &= ~ (1<<4); //  digitalWrite(IN3, LOW);
    PORTB &= ~(1<<5); //  digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  }
  else if(control_number == 3)
  {
      // turn right
      /*
     Motor left run
     Motor right stop
     */
   // Motor right stop
    PORTB &= ~(1<<1);//  digitalWrite(IN3, HIGH);
    PORTB &= ~(1<<0); //  digitalWrite(IN4, LOW);
    analogWrite(ENA, pwm);
    // Motor left run
    PORTB &= ~(1<<5); //  digitalWrite(IN1, HIGH);
    PORTB |= (1<<4); //  digitalWrite(IN2, LOW);
    analogWrite(ENB, pwm);
  }
  else if(control_number == 4)
  {
    // go backward
    PORTB &= ~(1<<1);//  digitalWrite(IN1, LOW);
    PORTB |= (1<<0); //  digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwm);
    PORTB &= ~(1<<4); //  digitalWrite(IN3, LOW);
    PORTB  |= (1<<5); //  digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwm);
  }
}

void motor_control_PID(int control_number, int pwm,int PID_mode)
{
  int PID_value;
  if (pwm > 255)
  {
    pwm = 255;
  }
  else if(pwm < 0)
  {
    pwm = 0;
  }
  
  if(control_number == 0)
  {
    // motor stop
    PORTB &= ~(3<<4);
    PORTB &= ~(3<<0);
  }
  else if(control_number == 1)
  {
    // go forward
        // Turn on motor right
    PORTB |= (1<<1);//  digitalWrite(IN1, HIGH);
    PORTB &= ~(1<<0); //  digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
   
     // turn on motor left
    PORTB |= (1<<4); //  digitalWrite(IN3, HIGH);
    PORTB &= ~ (1<<5); //  digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm );
  }
  else if(control_number == 2)
  {
    if(PID_mode == 1)
    {
      PID_value = PID_wall(1,distance_right_2);
    }
    else
    {
      PID_value = PID_angle(1,distance_right_2, distance_right_1);
    }
    // go away from the way (go left) VL < VR)
     // motor right
    PORTB |= (1<<1);//  digitalWrite(IN1, HIGH);
    PORTB &= ~(1<<0); //  digitalWrite(IN2, LOW);
      analogWrite(ENA, pwm +  PID_value);
     // motor left
    PORTB |= (1<<4); //  digitalWrite(IN3, HIGH);
    PORTB &= ~ (1<<5); //  digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  }
  else if(control_number == 3)
  {
     // go close to the wall (go left) VL > VR)
    if(PID_mode == 1)
    {
      PID_value = PID_wall(1,distance_right_2);
    }
    else
    {
      PID_value = PID_angle(1,distance_right_2, distance_right_1);
    }
        // motor right
    PORTB |= (1<<1);//  digitalWrite(IN1, HIGH);
    PORTB &= ~(1<<0); //  digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
   
     // motor left
    PORTB |= (1<<4); //  digitalWrite(IN3, HIGH);
    PORTB &= ~ (1<<5); //  digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm + PID_value);
  }
}

void right_turn_PID(float distance_right_2, int pwm)
{
  int PID_value;
  PID_value = PID_wall(1,distance_right_2);
       // motor right
  PORTB |= (1<<1);//  digitalWrite(IN1, HIGH);
  PORTB &= ~(1<<0); //  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwm - PID_value);
 
   // motor left
  PORTB |= (1<<4); //  digitalWrite(IN3, HIGH);
  PORTB &= ~ (1<<5); //  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwm + PID_value);
}

// PID for keeping motor going parralel to the wall
int PID_angle(int a_status,float distance_right_2, float distance_right_1)
{
  PID_current_time = micros();

  delta_T = ((float)(PID_current_time - PID_prev_time))/1.0e6;
  PID_prev_time = PID_current_time;
  if(a_status == 1)
  {
    // error
    e_angle = distance_right_2 - distance_right_1;
  
    // derivative_wall
    derivative_angle = (e_angle - e_prev_angle)/delta_T;
  
     // intergral 
    e_int_angle = e_int_angle + e_angle*delta_T;
  }
  else
  {
      // error
    e_angle = 0;
  
    // derivative_wall
    derivative_angle = 0;
  
     // intergral 
    e_int_angle = 0;
  }

  if ((distance_right_2 >= target_wall) && (distance_right_2 <= (target_wall + wall_offset)))
  {
    // safe_zone
    u_angle = kp_a*e_angle + ki_a*e_int_angle + kd_a*derivative_angle; 
    return fabs(u_angle);
  }
  else
  {
    return 0;
  }
  // convert control signal to speed and direction
}

// PID value for adjusting distance with the wall
int PID_wall(int w_status ,float distance_right_2)
{
  PID_current_time = micros();

  delta_T = ((float)(PID_current_time - PID_prev_time))/1.0e6;
  PID_prev_time = PID_current_time;
  // w_status = 1: run normally, w_status = 0: reset PID value
  if(w_status == 1)
  {
    // error
    if (distance_right_2 < wall_max)
    {
      e_wall = distance_right_2 - target_wall;
    }
    else
    {
      e_wall = wall_max - target_wall;
    }
  
    // derivative_wall
    derivative_wall = (e_wall - e_prev_wall)/delta_T;
  
     // intergral 
    e_int_wall = e_int_wall + e_wall*delta_T;
  }
  else
  {
    // error
    e_wall = 0;
  
    // derivative_wall
    derivative_wall = 0;
  
     // intergral 
    e_int_wall = 0;
  }

  if (distance_right_2 == target_wall)
  {
    // safe_zone
    return 0; // go forward
  }
  else
  {
    // control signal
    u_wall = kp_w*e_wall + ki_w*e_int_wall + kd_w*derivative_wall; 
    return fabs(u_wall);
   }
}

// Interrupt to check rising pulse on encoder.: Right C1 and left C2
void right_wheel_pulse() 
{
  // Read the right_C2_inputue for the encoder for the right wheel
  int right_C2_input = digitalRead(Encoder_right_C2);
  if(right_C2_input == LOW)
  {
    Direction_right = false; // Reverse
  }
  else 
  {
    Direction_right = true; // Forward
  }
   
  if (Direction_right)
  {
    right_wheel_pulse_count++;
  }
  else 
  {
    right_wheel_pulse_count--;
  }
}
void left_wheel_pulse() 
{
  // Read the right_C2_inputue for the encoder for the right wheel
  int left_C1_input = digitalRead(Encoder_left_C1);
  if(left_C1_input == LOW)
  {
    Direction_left = false; // Reverse
  }
  else 
  {
    Direction_left = true; // Forward
  }
   
  if (Direction_left)
  {
    left_wheel_pulse_count++;
  }
  else 
  {
    left_wheel_pulse_count--;
  }
}
// Function for converting rpm to m/s
void rpm_to_m_over_s(float rpm_right, float rpm_left)
{
  /*
   velocity (m/s) = ((2*3.14*d_wheel/2)/60)*N = A*N
   A = ((2*3.14*d_wheel/2)/60)
   v: linear velocity in m/s.
   N is angular velocity (rpm)
   r: radius (m).
   */
   const float A = 0.0034017;//(2*3.14*d_wheel/2)/60
   right_wheel_speed = A*rpm_right;
   left_wheel_speed = A*rpm_left;
}

void robot_velocity(float right_wheel_speed,float left_wheel_speed)
{
  // Record the time
  current_ms = millis();
 
  // If one second has passed, print the number of pulses
  if (current_ms - prev_ms > interval) 
  {
    prev_ms = current_ms;
    // Calculate revolutions per minute
    rpm_right = (float)((right_wheel_pulse_count*1250/pulse_per_rev));
    rpm_left = (float)((left_wheel_pulse_count*1250/pulse_per_rev));
    
    rpm_to_m_over_s(rpm_right,rpm_left); // convert rpm to m/s
    robot_speed = (right_wheel_speed + left_wheel_speed)/2;
//    Serial.print("Right Wheel Pulses: ");
//    Serial.println(right_wheel_pulse_count);
//    Serial.print("Right Wheel Speed: ");
//    Serial.print(rpm_right);
//    Serial.print(" rpm");
//    Serial.println();
//    Serial.print("\n");
//
//    Serial.print("Left Wheel Pulses: ");
//    Serial.println(left_wheel_pulse_count);
//    Serial.print("Left Wheel Speed: ");
//    Serial.print(rpm_left);
//    Serial.print(" rpm");
//    Serial.println();
//    Serial.print("\n");

    Serial.print("Robot SPeed (m/s): ");
    Serial.println(robot_speed);
    Serial.println();
    Serial.print("\n");
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
  }
}
void sensor_right_1()
{
   // Clears the trigger_left
  digitalWrite(trig_right_1, LOW);
  delayMicroseconds(2);
  // Sets the trigger on HIGH state for 10 micro seconds
  digitalWrite(trig_right_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_right_1, LOW);
  // Reads the echo_left, returns the sound wave travel time in microseconds
  duration_right_1 = pulseIn(echo_right_1, HIGH);
  // Calculating the distance
  distance_right_1 = duration_right_1*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance right 1: ");
  Serial.println(distance_right_1);
}
void sensor_right_2()
{
  digitalWrite(trig_right_2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_right_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_right_2, LOW);
  // Reads the echo_left, returns the sound wave travel time in microseconds
  duration_right_2 = pulseIn(echo_right_2, HIGH);
  // Calculating the distance
  distance_right_2 = duration_right_2*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance right 2: ");
  Serial.println(distance_right_2);
}
void sensor_front_r()
{
  digitalWrite(trig_front_r, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_front_r, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_front_r, LOW);
  // Reads the echo_left, returns the sound wave travel time in microseconds
  duration_front_r = pulseIn(echo_front_r, HIGH);
  // Calculating the distance
  distance_front_r = duration_front_r*0.034/2;
  // Prints the distance on the Serial Monitor
//  Serial.print("Distance front right : ");
//  Serial.println(distance_front_r);
}

void sensor_front_l()
{
  digitalWrite(trig_front_l, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_front_l, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_front_l, LOW);
  // Reads the echo_left, returns the sound wave travel time in microseconds
  duration_front_l = pulseIn(echo_front_l, HIGH);
  // Calculating the distance
  distance_front_l = duration_front_l*0.034/2;
//  // Prints the distance on the Serial Monitor
  Serial.print("Distance front left : ");
  Serial.println(distance_front_l);
}
// Use all sensor at once reduce delay
void robot_sensor()
{
  sensor_right_1();
  sensor_right_2();
  sensor_front_r();
  sensor_front_l();
}
