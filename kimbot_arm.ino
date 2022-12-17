#include <ESP32_Servo.h> //Servo library for ESP32
//#include <Servo.h> //Servo library for Arduino
#include <BluetoothSerial.h>
#include <math.h>

//------------------------ Bluetooth Serial ----------------------------
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
//------------------------ Bluetooth Serial ----------------------------

//--------------------------- Variables --------------------------------
const float DEG2RAD = 0.017453292;
const float RAD2DEG = 57.29578049;

#define servo1_pin 12//23
#define servo2_pin 13//22
#define servo3_pin 14//21
#define servo4_pin 19

#define servo4_close 0
#define servo4_open 60

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

bool FK_debugPrint = true;
bool IK_debugPrint = true;
bool TEST_debugPrint = true;

//coordinates for Inverse Kinematics
float x=8, y=0, z=12;
//angles for Forward Kinematics
float global_angles[4] = {0,0,0,0};
//running mode
int global_mode = 0;
int global_control_mode = 0;
int standby_count;

float x_axis, y_axis, z_axis;
float current_angle[4] = {0,0,0,0};
float offset_angle[4] = {90,45,0,0};

const int servo_count = 4;
//--------------------------- Variables --------------------------------

//--------------------------- Parameters -------------------------------
enum global_params{
  mode_Default,
  mode_FK,
  mode_IK,
  mode_IK_Move_FK,
  control_Bluetooth,
  control_SerialMonitor,
  control_Default,
  global_param_count
};

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
//--------------------------- Parameters -------------------------------

Servo *servo_address[servo_count] = {&servo1, &servo2, &servo3, &servo4};

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
//  moveServo(0, 0, 0, 0);
}

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
//  if(output < servo[servo_id][min_pwm])
//    output = servo[servo_id][min_pwm];
//  if(output > servo[servo_id][max_pwm])
//    output = servo[servo_id][max_pwm];
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
    delay(50);
  }
}

void moveToPos(float x_input, float y_input, float z_input) { 
  if(IK_debugPrint == true){
    Serial.println("Input Coordinates:");
    Serial.print("X: ");
    Serial.println(x_input);
    Serial.print("Y: ");
    Serial.println(y_input);
    Serial.print("Z: ");
    Serial.println(z_input);

    SerialBT.println("Input Coordinates:");
    SerialBT.print("X: ");
    SerialBT.println(x_input);
    SerialBT.print("Y: ");
    SerialBT.println(y_input);
    SerialBT.print("Z: ");
    SerialBT.println(z_input);
  }
  float target_angle[4] = {0,0,0,0};

  inverseKinematics(target_angle, x_input, y_input, z_input);
  
  moveAllServo(target_angle);

  float length[3] = {4, 8, 8};
  if(TEST_debugPrint == true){
    Serial.println("Predicted Angles:");
    Serial.print("A1: ");
    Serial.println(target_angle[0]);
    Serial.print("A2: ");
    Serial.println(target_angle[1]);
    Serial.print("A3: ");
    Serial.println(target_angle[2]);

    SerialBT.println("Predicted Angles:");
    SerialBT.print("A1: ");
    SerialBT.println(target_angle[0]);
    SerialBT.print("A2: ");
    SerialBT.println(target_angle[1]);
    SerialBT.print("A3: ");
    SerialBT.println(target_angle[2]);
  }
  target_angle[0] *= -1;
  target_angle[1] *= -1;
  target_angle[2] *= -1;
//  target_angle[1] *= -1;
  forwardKinematics(&x_axis, &y_axis, &z_axis, target_angle, length);

  if(FK_debugPrint == true){
    Serial.println("Predicted Coordinates:");
    Serial.print("X: ");
    Serial.println(x_axis);
    Serial.print("Y: ");
    Serial.println(y_axis);
    Serial.print("Z: ");
    Serial.println(z_axis);

    SerialBT.println("Predicted Coordinates:");
    SerialBT.print("X: ");
    SerialBT.println(x_axis);
    SerialBT.print("Y: ");
    SerialBT.println(y_axis);
    SerialBT.print("Z: ");
    SerialBT.println(z_axis);
  }
  // Serial.print("Phi: ");
  // Serial.println(phi);
  // Serial.print("Theta: ");
  // Serial.println(theta);
}

void inverseKinematics(float* target_angle, float x_input, float y_input, float z_input){
  z_input -= 4;
  target_angle[0] = - atan2(y_input,x_input) * RAD2DEG; // base angle
  double l = sqrt(x_input * x_input + y_input * y_input); // z_input and y_input extension
  double h = sqrt (l*l + z_input * z_input);
  double phi = atan2(z_input,l) * RAD2DEG;
  double theta = acos((h/2)/8) * RAD2DEG;
  target_angle[1] = - phi - theta + 90; // angle for second part of the arm
  target_angle[2] = - phi + theta;// - target_angle[1]; // angle for first part of the arm
  target_angle[3] = servo4_close;
}

/*
A = Q*cos(X)*cos(Y) - R*cos(X)*sin(Y)*sin(Z) + R*cos(X)*cos(Y)*cos(Z)
B = Q*cos(Y)*sin(X) - R*sin(X)*sin(Y)*sin(Z) + R*cos(Y)*cos(Z)*sin(X)
C = P + Q*sin(Y) + R*cos(Y)*sin(Z) + R*cos(Z)*sin(Y)
*/

void forwardKinematics(float* x_axis, float* y_axis, float* z_axis, float* degree, float* length){
  degree[2] -= degree[1];
  degree[1] += 90;
  degree[2] -= 90;
  for(int servo_id = 0; servo_id < servo_count; ++servo_id){
    degree[servo_id] *= DEG2RAD;
//    Serial.println(degree[servo_id]);
  }
  *x_axis = length[1] * cos(degree[0]) * cos(degree[1])
          - length[2] * cos(degree[0]) * sin(degree[1]) * sin(degree[2])
          + length[2] * cos(degree[0]) * cos(degree[1]) * cos(degree[2]);
  *y_axis = length[1] * cos(degree[1]) * sin(degree[0])
          - length[2] * sin(degree[0]) * sin(degree[1]) * sin(degree[2])
          + length[2] * cos(degree[1]) * cos(degree[2]) * sin(degree[0]);
  *z_axis = length[2] * cos(degree[1]) * sin(degree[2])
          + length[2] * cos(degree[2]) * sin(degree[1])
          + length[1] * sin(degree[1])
          + length[0];
  for(int servo_id = 0; servo_id < servo_count; ++servo_id){
    degree[servo_id] *= RAD2DEG;
  }
  degree[1] -= 90;
  degree[2] += 90;
}

void forwardKinematics_DHIYAA(float* x_axis, float* y_axis, float* z_axis, float* degree, float* length){
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

  SerialBT.print("Predicted Coordinates from : ");
  SerialBT.print(target_angle[0]);
  SerialBT.print(", ");
  SerialBT.print(target_angle[1]);
  SerialBT.print(", ");
  SerialBT.print(target_angle[2]);
  SerialBT.print(", ");
  SerialBT.println(target_angle[3]);

  forwardKinematics(&x_axis, &y_axis, &z_axis, target_angle, length);

  Serial.print("X: ");
  Serial.println(x_axis);
  Serial.print("Y: ");
  Serial.println(y_axis);
  Serial.print("Z: ");
  Serial.println(z_axis);

  SerialBT.print("X: ");
  SerialBT.println(x_axis);
  SerialBT.print("Y: ");
  SerialBT.println(y_axis);
  SerialBT.print("Z: ");
  SerialBT.println(z_axis);
}

void testIK(float x_input, float y_input, float z_input){
  float target_angle[4] = {0,0,0,0};

  Serial.println("Input Coordinates:");
  Serial.print("X: ");
  Serial.println(x_input);
  Serial.print("Y: ");
  Serial.println(y_input);
  Serial.print("Z: ");
  Serial.println(z_input);

  SerialBT.println("Input Coordinates:");
  SerialBT.print("X: ");
  SerialBT.println(x_input);
  SerialBT.print("Y: ");
  SerialBT.println(y_input);
  SerialBT.print("Z: ");
  SerialBT.println(z_input);

  inverseKinematics(target_angle, x_input, y_input, z_input);
  
  Serial.println("Predicted Angles:");
  Serial.print("A1: ");
  Serial.println(target_angle[0]);
  Serial.print("A2: ");
  Serial.println(target_angle[1]);
  Serial.print("A3: ");
  Serial.println(target_angle[2]);

  SerialBT.println("Predicted Angles:");
  SerialBT.print("A1: ");
  SerialBT.println(target_angle[0]);
  SerialBT.print("A2: ");
  SerialBT.println(target_angle[1]);
  SerialBT.print("A3: ");
  SerialBT.println(target_angle[2]);
}

void test_moveToPos(){
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

bool receiveXYZ(float* x, float* y, float* z){
  if(Serial.available()){
    *x = Serial.parseInt();
    *y = Serial.parseInt();
    *z = Serial.parseInt();
    return true;
  }
  if(SerialBT.available()){
    *x = SerialBT.parseInt();
    *y = SerialBT.parseInt();
    *z = SerialBT.parseInt();
    return true;
  }
  else return false;
}

bool receiveAngles(float* target_angle){
  if(Serial.available()){
    for(int servo_id = 0; servo_id < servo_count; ++servo_id)
      target_angle[servo_id] = Serial.parseInt();
    return true;
  }
  if(SerialBT.available()){
    for(int servo_id = 0; servo_id < servo_count; ++servo_id)
      target_angle[servo_id] = SerialBT.parseInt();
    return true;
  }
  else return false;
}

bool bluetoothController(float* x_input, float* y_input, float* z_input){
  if(SerialBT.available()){
    String BT_Button = SerialBT.readString();
    if(BT_Button == "x+") *x_input += 1;
    else if(BT_Button == "x-") *x_input -= 1;
    else if(BT_Button == "y+") *y_input += 1;
    else if(BT_Button == "y-") *y_input -= 1;
    else if(BT_Button == "z+") *z_input += 1;
    else if(BT_Button == "z-") *z_input -= 1;
    
    return true;
  }
  else return false;
}

bool receiveMode(int* mode){
  if(Serial.available()){
    *mode = Serial.parseInt();
    return true;
  }
  if(SerialBT.available()){
    *mode = SerialBT.parseInt();
    return true;
  }
  else return false;
}

void ESP32_Test(){
  Serial.println("ESP32 test");
  while (SerialBT.available()) {
    Serial.write(SerialBT.read());
    delay(20);
  }
}

void loop(){
  switch (global_mode){
  case 0:
    if(!receiveMode(&global_mode)){
      Serial.println("Choose mode:");
      Serial.println("1. Forward Kinematics");
      Serial.println("2. Inverse Kinematics");
      Serial.println("3. Move by Coords");
      Serial.println("4. Bluetooth Controller");

      SerialBT.println("Choose mode:");
      SerialBT.println("1. Forward Kinematics");
      SerialBT.println("2. Inverse Kinematics");
      SerialBT.println("3. IK -> FK");
      SerialBT.println("4. Bluetooth Controller");
    }
    break;
  
  case 1:
    if(receiveAngles(global_angles)){
      float target_angle[4] = {0,0,0,0};
      for(int i = 0; i < 4; ++i)
        target_angle[i] = global_angles[i];
      testFK(target_angle);
    }
    else{
      Serial.println("Forward Kinematics: A1, A2, A3, A4");
      SerialBT.println("Forward Kinematics: A1, A2, A3, A4");
    }
    break;
  
  case 2:
    if(receiveXYZ(&x, &y, &z)){
      testIK(x, y, z);
    }
    else{
      Serial.println("Inverse Kinematics: X, Y, Z");
      SerialBT.println("Inverse Kinematics: X, Y, Z");
    }
    break;
  
  case 3:
    if(receiveXYZ(&x, &y, &z))
      moveToPos(x, y, z);
    else{
      Serial.println("Move to Coordinates: X, Y, Z");
      SerialBT.println("Move to Coordinates: X, Y, Z");
    }
    break;

  case 4:
    if(bluetoothController(&x, &y, &z))
      moveToPos(x, y, z);
    else{
      Serial.println("Bluetooth Controller: Waiting for input");
      SerialBT.println("Bluetooth Controller: Waiting for input");
    }
    break;
  
  default:
    Serial.println("Mode not recognized");
    SerialBT.println("Mode not recognized");
    global_mode = 0;
    break;
  }

  delay(1000);
}
