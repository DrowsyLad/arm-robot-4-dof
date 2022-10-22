#include <Servo.h>
// using namespace servo_param;

/*
 * Servo HS5055MG Sudut: 20-165
 * Servo SG90 Sudut: 0-
 */

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int angle = 0;
float a = 0, b = 0, c = 0, d = 0;
float servo_1_current_angle = 0, servo_2_current_angle = 0, servo_3_current_angle = 0, servo_4_current_angle = 0;
float servo_1_offset_angle = 90, servo_2_offset_angle = 105, servo_3_offset_angle = 120, servo_4_offset_angle = 0;

const int servo_count = 4;

enum servo_param{
  pin_pwm,
  deg_range,
  min_pwm,
  max_pwm,
  max_to_min,
  param_count
};

int servo[servo_count][param_count] = {
  {10,  180,  540,  2400,   0},   //servo 1, base
  {9,   90,   750,  1650,   1},   //servo 2
  {6,   80,   800,  1750,   1},   //servo 3
  {7,   60,   540,  1200,   0},   //servo 4, gripper
};

Servo *servo_address[servo_count] = {&servo1, &servo2, &servo3, &servo4};

void set_pwm(Servo *servo_target, int output_pwm){
  servo_target->writeMicroseconds(output_pwm);
}

void move_servo_pwm(int servo_id, float target_angle){
  --servo_id;
  float deg_to_pwm = target_angle / servo[servo_id][deg_range] * (servo[servo_id][max_pwm] - servo[servo_id][min_pwm]);
  int output;
  if(servo[servo_id][max_to_min])
    output = servo[servo_id][max_pwm] - round(deg_to_pwm);
  else
    output = round(deg_to_pwm) - servo[servo_id][min_pwm];
  set_pwm(servo_address[servo_id], output);
}

//outdated
void move_servo(float servo_1_target_angle, float servo_2_target_angle, float servo_3_target_angle, float servo_4_target_angle){
  int move_time = 1000; //in miliseconds
  int delay_per_steps = 10; //
  int step_count = move_time / delay_per_steps;

  float servo_1_step_angle = (servo_1_target_angle - servo_1_current_angle) / step_count, 
        servo_2_step_angle = (servo_2_target_angle - servo_2_current_angle) / step_count, 
        servo_3_step_angle = (servo_3_target_angle - servo_3_current_angle) / step_count, 
        servo_4_step_angle = (servo_4_target_angle - servo_4_current_angle) / step_count;
  
  for(int steps = 0; steps < step_count; steps++){
    servo1.write(servo_1_offset_angle + (servo_1_current_angle += servo_1_step_angle)); //servo 1 angle = 90 - x
    servo2.write(servo_2_offset_angle - (servo_2_current_angle += servo_2_step_angle)); //servo 2 angle = 165 - x; max 110 min 20
    servo3.write(servo_3_offset_angle - (servo_3_current_angle += servo_3_step_angle)); //servo 3 angle = 90 - x
    servo4.write(servo_4_offset_angle + (servo_4_current_angle += servo_4_step_angle)); //servo 4 angle = 0 + x
    delay(delay_per_steps);
  }
}

void setup() {
  Serial.begin(9600);
  servo1.attach(10);
  servo2.attach(9);
  servo3.attach(6);
  servo4.attach(7);
//  move_servo(0, 0, 0, 0);
}

void loop(){
  move_servo_pwm(2, 0);
  move_servo_pwm(3, 0);
  delay(5000);
  move_servo_pwm(2, 45);
  move_servo_pwm(3, 45);
  delay(5000);
  move_servo_pwm(2, 90);
  move_servo_pwm(3, 90);
  delay(5000);
  move_servo_pwm(2, 45);
  move_servo_pwm(3, 45);
  delay(5000);
  // move_servo(0, 0, 0, 0);
  // delay(2000);
}
