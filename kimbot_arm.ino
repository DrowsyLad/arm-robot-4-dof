#include <Servo.h>

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



enum servo_param{
  min_pwm,
  max_pwm,
};

//servo[4]

void set_pwm(Servo *servo_target, int output_pwm){
  servo_target->writeMicroseconds(output_pwm);
}

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
  move_servo(0, 0, 0, 0);
}

void loop(){ 
//  set_pwm(&servo2, 750);
  for(int pwm = 540; pwm < 2400; pwm += 10){ //0->180
    set_pwm(&servo1, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 1600; pwm > 750; pwm -= 10){ //forward
    set_pwm(&servo2, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 540; pwm < 1200; pwm += 10){ //open
    set_pwm(&servo4, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 750; pwm < 1600; pwm += 10){ //backward
    set_pwm(&servo2, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 2400; pwm > 540; pwm -= 10){ //180->0
    set_pwm(&servo1, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 1600; pwm > 750; pwm -= 10){ //forward
    set_pwm(&servo2, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 1200; pwm > 540; pwm -= 10){ //close
    set_pwm(&servo4, pwm);
    Serial.println(pwm);
    delay(10);
  }
  for(int pwm = 750; pwm < 1600; pwm += 10){ //backward
    set_pwm(&servo2, pwm);
    Serial.println(pwm);
    delay(10);
  }
}
