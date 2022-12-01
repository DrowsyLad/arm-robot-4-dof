#include <Servo.h>
#include <math.h>

const float DEG2RAD = 0.017453292;
const float RAD2DEG = 57.29578049;

#define servo1_pin A0
#define servo2_pin A1
#define servo3_pin A2
#define servo4_pin A3

#define servo4_close 0
#define servo4_open 60

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

bool FK_debugPrint = true;
bool IK_debugPrint = true;
bool TEST_debugPrint = true;

float x_axis, y_axis, z_axis;
float current_angle[4] = {0,0,0,0};
float offset_angle[4] = {90,45,0,0};

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
  {servo1_pin,  180,  540,  2400,   0},   //servo 1, base
  {servo2_pin,   90,   750,  1650,   1},   //servo 2
  {servo3_pin,   80,   800,  1750,   1},   //servo 3
  {servo4_pin,   60,   540,  1200,   0},   //servo 4, gripper
};

Servo *servo_address[servo_count] = {&servo1, &servo2, &servo3, &servo4};

void set_pwm(Servo *servo_target, int output_pwm){
  servo_target->writeMicroseconds(output_pwm);
}

void moveServo_pwm(int servo_id, float target_angle){
  float deg_to_pwm = target_angle / servo[servo_id][deg_range] * (servo[servo_id][max_pwm] - servo[servo_id][min_pwm]);
  int output;
  if(servo[servo_id][max_to_min])
    output = servo[servo_id][max_pwm] - round(deg_to_pwm);
  else
    output = round(deg_to_pwm) + servo[servo_id][min_pwm];
  set_pwm(servo_address[servo_id], output);
  current_angle[servo_id] = target_angle;
//  Serial.print("Servo ");
//  Serial.print(servo_id);
//  Serial.print(" set to ");
//  Serial.println(output);
}

void moveAllServo_timed(int move_time, //in miliseconds
                        float* target_angle){
  int delay_per_steps = 10; //
  int step_count = move_time / delay_per_steps;

  float servo_step_angle[4]; 
  for(int servo_id = 0; servo_id < servo_count; servo_id++){
    servo_step_angle[servo_id] = (target_angle[servo_id] - current_angle[servo_id]) / step_count;
  }
  
  for(int steps = 0; steps < step_count; steps++){
    for(int servo_id = 0; servo_id < servo_count; servo_id++){
      moveServo_pwm(servo_id, (current_angle[servo_id] += servo_step_angle[servo_id]));
      Serial.print("Servo ");
      Serial.print(servo_id);
      Serial.print(" step angle: ");
      Serial.println(servo_step_angle[servo_id]);
    }
    delay(delay_per_steps);
  }
}

void moveAllServo(float* target_angle){
  for(int servo_id = 0; servo_id < servo_count; servo_id++){
    moveServo_pwm(servo_id, target_angle[servo_id] + offset_angle[servo_id]);
    // Serial.print("Servo ");
    // Serial.print(servo_id);
    // Serial.print(" moved to ");
    // Serial.println(current_angle[servo_id]);
    delay(50);
  }
}

void moveToPos(double x_input, double y_input, double z_input) { 
  if(IK_debugPrint == true){
    Serial.println("Input Coordinates:");
    Serial.print("X: ");
    Serial.println(x_input);
    Serial.print("Y: ");
    Serial.println(y_input);
    Serial.print("Z: ");
    Serial.println(z_input);
  }
  z_input -= 4;
  float target_angle[4];
  target_angle[0] = atan2(y_input,z_input) * (180 / 3.1415); // base angle
  double l = sqrt(z_input*z_input + y_input*y_input); // z_input and y_input extension
  double h = sqrt (l*l + x_input*x_input);
  double phi = atan(x_input/l) * (180 / 3.1415);
  double theta = acos((h/2)/8) * (180 / 3.1415);
  target_angle[1] = phi - theta; // angle for second part of the arm
  target_angle[2] = phi + theta - 90; // angle for first part of the arm
  target_angle[3] = servo4_close;
  moveAllServo(target_angle);

  float length[3] = {4, 8, 8};
  forwardKinematics(&x_axis, &y_axis, &z_axis, target_angle, length);
  if(FK_debugPrint == true){
    Serial.println("Predicted Coordinates:");
    Serial.print("X: ");
    Serial.println(x_axis);
    Serial.print("Y: ");
    Serial.println(y_axis);
    Serial.print("Z: ");
    Serial.println(z_axis);
  }
//  double target_angle[0] = 1;
//  double target_angle[1] = 2;
//  double target_angle[2] = 3;
  if(TEST_debugPrint == true){
    Serial.println("Predicted Angles:");
    Serial.print("A1: ");
    Serial.println(target_angle[0] * RAD2DEG);
    Serial.print("A2: ");
    Serial.println(target_angle[1] * RAD2DEG);
    Serial.print("A3: ");
    Serial.println(target_angle[2] * RAD2DEG);
  }
  // Serial.print("Phi: ");
  // Serial.println(phi);
  // Serial.print("Theta: ");
  // Serial.println(theta);
}

/*
A = Q*cos(X)*cos(Y) - R*cos(X)*sin(Y)*sin(Z) + R*cos(X)*cos(Y)*cos(Z)
B = Q*cos(Y)*sin(X) - R*sin(X)*sin(Y)*sin(Z) + R*cos(Y)*cos(Z)*sin(X)
C = P + Q*sin(Y) + R*cos(Y)*sin(Z) + R*cos(Z)*sin(Y)
*/

float forwardKinematics(float* x_axis, float* y_axis, float* z_axis, float* degree, float* length){
  degree[1] += 90;
  degree[2] -= 90;
  for(int servo_id = 0; servo_id < servo_count; ++servo_id){
    degree[servo_id] *= DEG2RAD;
  }
  *x_axis = length[1] * cos(degree[0]) * cos(degree[1])
          - length[2] * cos(degree[0]) * sin(degree[1]) * sin(degree[2]);
          + length[2] * cos(degree[0]) * cos(degree[1]) * cos(degree[2]);
  *y_axis = length[1] * cos(degree[1]) * sin(degree[0])
          - length[2] * sin(degree[0]) * sin(degree[1]) * sin(degree[2]);
          + length[2] * cos(degree[1]) * cos(degree[2]) * sin(degree[0]);
  *z_axis = length[2] * cos(degree[1]) * sin(degree[2])
          + length[2] * cos(degree[2]) * sin(degree[1])
          + length[1] * sin(degree[1])
          + length[0];
  degree[1] -= 90 * DEG2RAD;
  degree[2] += 90 * DEG2RAD;
}

float forwardKinematics_DHIYAA(float* x_axis, float* y_axis, float* z_axis, float* degree, float* length){
  *x_axis = length[1] * cos(degree[0]) * cos(degree[1]) * cos(degree[2])
          + length[1] * cos(degree[0]) * cos(degree[1])
          - length[2] * cos(degree[0]) * sin(degree[1]) * sin(degree[2]);
  *y_axis = length[1] * cos(degree[2]) * sin(degree[1])
          + length[1] * sin(degree[0]) * cos(degree[1])
          - length[2] * cos(degree[0]) * sin(degree[1]) * sin(degree[2]);
  *z_axis = length[1] * cos(degree[2]) * sin(degree[1])
          + length[1] * sin(degree[1])
          + length[2] * cos(degree[1]) * sin(degree[2])
          + length[0];
}

//outdated
void moveServo(float servo_1_target_angle, float servo_2_target_angle, float servo_3_target_angle, float servo_4_target_angle){  
  float servo_1_current_angle = 0, servo_2_current_angle = 0, servo_3_current_angle = 0, servo_4_current_angle = 0;
  float servo_1_offset_angle = 90, servo_2_offset_angle = 105, servo_3_offset_angle = 120, servo_4_offset_angle = 0;
  
  int move_time = 1000; //in miliseconds
  int delay_per_steps = 5; //
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
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
//  moveServo(0, 0, 0, 0);
}

void test_framework_2(){
  float angle_1[4] = {0, 0, 0, 0};
  moveAllServo_timed(100, angle_1);
  printCurrentAngle();
  delay(1000);
  float angle_2[4] = {45, 45, 45, 0};
  moveAllServo_timed(100, angle_2);
  printCurrentAngle();
  delay(1000);
  float angle_3[4] = {90, 90, 90, 0};
  moveAllServo_timed(100, angle_3);
  printCurrentAngle();
  delay(1000);
}

void printCurrentAngle(){
  for(int servo_id = 0; servo_id < servo_count; servo_id++){
    Serial.print("Servo ");
    Serial.print(servo_id);
    Serial.print(" currently at ");
    Serial.println(current_angle[servo_id]);
  }
}

void test_framework_1(){
  float angle_1[4] = {0, 45, 45, 0};
  float angle_2[4] = {0, 0, 0, 0};
  float angle_3[4] = {0, -45, -45, 0};
  moveAllServo(angle_1);
  delay(10000);
  moveAllServo(angle_2);
  delay(10000);
  moveAllServo(angle_3);
  delay(10000);
  moveAllServo(angle_2);
  delay(10000);
}

void test_framework_3(){
  float angle_1[4] = {0, 45, 45, 0};
  float angle_2[4] = {0, 0, 0, 0};
  float angle_3[4] = {0, -45, -45, 0};
  testFK(angle_1);
  delay(10000);
  testFK(angle_2);
  delay(10000);
  testFK(angle_3);
  delay(10000);
  testFK(angle_2);
  delay(10000);
}

void testFK(float* target_angle){
  float length[3] = {4, 8, 8};
  Serial.print("Predicted Coordinates from : ");
  Serial.print(target_angle[0]);
  Serial.print(", ");
  Serial.print(target_angle[1]);
  Serial.print(", ");
  Serial.print(target_angle[2]);
  Serial.print(", ");
  Serial.println(target_angle[3]);
  forwardKinematics(&x_axis, &y_axis, &z_axis, target_angle, length);
  Serial.print("X: ");
  Serial.println(x_axis);
  Serial.print("Y: ");
  Serial.println(y_axis);
  Serial.print("Z: ");
  Serial.println(z_axis);
}

void testIK(){
  float x = 8, y = 0, z = 12, step = 0.1;
  int delay_ = 10;
  for(; z >= 8;){ 
    moveToPos(x,y,z);
    z -= step;
    delay(delay_);
  }
  for(; x >= 6;){ 
    moveToPos(x,y,z);
    x -= step;
    delay(delay_);
  }
  for(; x <= 12;){
    moveToPos(x,y,z);
    x += step;
    delay(delay_);
  }
  for(; z <= 12;){ 
    moveToPos(x,y,z);
    z += step;
    delay(delay_);
  }
  for(; x >= 8;){
    moveToPos(x,y,z);
    x -= step;
    delay(delay_);
  }
}

void loop(){
//  float target_angle[4] = {0,0,0,0};
//  testFK(target_angle);
  IK_debugPrint = false;
  TEST_debugPrint = false;
//  FK_debugPrint = false;
  testIK();
//  delay(1000);
// test_framework_3();
//  return;
}

void loop_(){
//  moveServo_pwm(2, 0);
//  moveServo_pwm(3, 0);
//  delay(5000);
//  moveServo_pwm(2, 45);
//  moveServo_pwm(3, 45);
//  delay(5000);
//  moveServo_pwm(2, 90);
//  moveServo_pwm(3, 90);
//  delay(5000);
//  moveServo_pwm(2, 45);
//  moveServo_pwm(3, 45);
//  delay(5000);
  // moveServo(0, 0, 0, 0);
  // delay(2000);
}
