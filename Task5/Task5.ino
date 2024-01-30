#include <MPU6050_light.h>
#include <Wire.h>
#include <Encoder.h>
#include <SPI.h>
#include <RF24.h>
#define DEADZONE 50
#define enA 6  
#define in1 A2
#define in2 A3
#define enB 5
#define in3 9
#define in4 4
RF24 radio(10, 8);
const int encoderPinA = 2;
const int encoderPinB = 3;
MPU6050 mpu(Wire);
Encoder DC_Encoder(encoderPinA,encoderPinB);
long old_ticks = -999; //to store ticks of encoder
const byte add_1 = 5;
const byte add_2 = 10;
float a[3] = {0, 0, 0} , g[3] = {0, 0, 0}; //stores accelerometer and gyro value
const float pi = 3.14159265359, f_cut = 5, dT = 0.003, comp_alpha = 0.01; //f_cut sampling frequency.
float Tau = 1 / (2 * pi*f_cut); //where tau is the desired time constant 
float alpha = Tau / (Tau + dT); // alpha is proportion
float roll_deg = 0, yaw_deg = 0 ; //tilt and yaw angle in degree 
float omega_roll = 0, omega_yaw = 0; // angularVelocity of roll and yaw
float lpx = 0, lpy = 0, lpz = 0, hpx = 0, hpy = 0, hpz = 0; // low pass filtered acceleration in the x, y, and z directions and high pass filtered gyroscope readings in the x, y, and z directions respectively
int m = 1, n = 1; 
float x1,x2,x3,x4; //errors
float k[] = {-1.003139 , -0.50000  , 0.953048 ,  6.973991}; //k matrix
float U,U_new; //pwm for dc motor 
float yaw_setpoint = 0;
typedef struct struct_message {
 int front; 
 int back;
 int left;
 int right;
 int sw1;
 int sw2;
 int sw3;
} struct_message;

struct_message data; //object of struct_message

////////////////////////////////////////////

void setup() {
      Serial.begin(9600);
      int result = radio.begin();
      if (result) {
        Serial.println("NRF module Working");
      } 
      else {
        Serial.println("NRF module ERROR ");
        
      }
      radio.openWritingPipe(add_2);
      radio.openReadingPipe(1, add_1);
      radio.setDataRate(RF24_2MBPS);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
      Wire.begin();
      mpu.begin();
      mpu.calcOffsets();
      dc_motor_init();
      bo_motor_init();

}

////////////////////////////////////////

void loop() {
  mpu_int();
  if (radio.available()) {
     radio.read((uint8_t *) &data, sizeof(struct_message));
  } 
  remote_control();
  encoder();
  x1= omega_yaw;
  x2 =  yaw_deg - yaw_setpoint ;
  x3 = omega_roll;
  x4 = roll_deg;
  U =  -k[0]*x1  + k[1]*x2 + k[2]*x3 + k[3]*x4;
  U_new = constrain(U*5,-255,255);
  motor_control(U_new);


}
///////////////////////////////////////////
// Function Name: low_pass_filter
// Input: Ax, Ay, Az - These are the input parameters representing the acceleration in the x, y, and z directions respectively.
// Output: This function does not return any value. However, it modifies the global variables lpx, lpy, and lpz.
// Logic: This function applies a low pass filter to the input acceleration values. 
//The filter is applied differently for the first input (n == 1) compared to subsequent inputs. For the first input, the filtered value is a fraction of the input value. For subsequent inputs, the filtered value is a weighted average of the input value and the previous filtered value.
// Example Call: low_pass_filter(1.0, 2.0, 3.0);
  
void low_pass_filter(float Ax, float Ay, float Az){
  // If this is the first input
  if (n == 1)
  {  // Apply the filter by taking a fraction of the input value
    lpx = (1 - alpha) * Ax;
    lpy = (1 - alpha) * Ay;
    lpz = (1 - alpha) * Az;
     // Increment the counter
    n++;
  }
  else
  {// For subsequent inputs, apply the filter by taking a weighted average of the input value and the previous filtered value
    lpx = (1 - alpha) * Ax + alpha * lpx;
    lpy = (1 - alpha) * Ay + alpha * lpy;
    lpz = (1 - alpha) * Az + alpha * lpz;
  }
}

////////////////////////////////////////////
// Function Name: high_pass_filter
// Input: Gx, Gy, Gz - These are the input parameters representing the gyroscope readings in the x, y, and z directions respectively.
// Output: This function does not return any value. However, it modifies the global variables hpx, hpy, hpz, and g.
// Logic: This function applies a high pass filter to the input gyroscope readings. The filter is applied differently for the first input (m == 1) compared to subsequent inputs. For the first input, the filtered value is a fraction of the input value. For subsequent inputs, the filtered value is a weighted difference of the current input, the previous input, and the previous filtered value.
// Example Call: high_pass_filter(1.0, 2.0, 3.0);
void high_pass_filter(float Gx, float Gy, float Gz){
   // If this is the first input
  if (m == 1)
  { // Apply the filter by taking a fraction of the input value
    hpx = (1 - alpha) * Gx;
    hpy = (1 - alpha) * Gy;
    hpz = (1 - alpha) * Gz;
     // Increment the counter
    m++;
  }
  else
  { 
     // For subsequent inputs, apply the filter by taking a weighted difference of the current input, the previous input, and the previous filtered value
    hpx = (1 - alpha) * (hpx + Gx - g[0]);
    hpy = (1 - alpha) * (hpy + Gy - g[1]);
    hpz = (1 - alpha) * (hpz + Gz - g[2]);
  }
   // Store the current input for use in the next iteration
  g[0] = Gx;
  g[1] = Gy;
  g[2] = Gz;
}

///////////////////////////////////////
// Function Name: getAcc
// Input: None
// Output: This function does not return any value. However, it modifies the global array 'a' and calls the function 'low_pass_filter'.
// Logic: This function gets the acceleration values in the x, y, and z directions from the MPU (Motion Processing Unit) sensor and stores them in the array 'a'. It then calls the 'low_pass_filter' function with these values.
// Example Call: getAcc();

void getAcc(){
    // Get the acceleration values in the x, y, and z directions from the MPU sensor
  a[0] = mpu.getAccX();
  a[1] = mpu.getAccY();
  a[2] = mpu.getAccZ();
    // Call the 'low_pass_filter' function with these values
  low_pass_filter(a[0], a[1], a[2]);
}

////////////////////////////////////////
// Function Name: getGyro
// Input: None
// Output: This function does not return any value. However, it modifies the global array 'g' and calls the function 'high_pass_filter'.
// Logic: This function gets the gyroscope readings in the x, y, and z directions from the MPU (Motion Processing Unit) sensor and stores them in the array 'g'. It then calls the 'high_pass_filter' function with these values.
// Example Call: getGyro();
void getGyro(){
  // Get the gyroscope readings in the x, y, and z directions from the MPU sensor
  g[0] = mpu.getGyroX();
  g[1] = mpu.getGyroY();
  g[2] = mpu.getGyroZ();
    // Call the 'high_pass_filter' function with these values
  high_pass_filter(g[0], g[1], g[2]);
}

////////////////////////////////////////////
// Function Name: complimentary_filter_roll
// Input: None
// Output: This function does not return any value. However, it modifies the global variables 'roll_deg', 'omega_roll', and 'omega_yaw'.
// Logic: This function applies a complimentary filter to calculate the roll angle in degrees. It uses both accelerometer and gyroscope readings. The roll angle is a weighted sum of the roll angle calculated from the gyroscope readings and the roll angle calculated from the accelerometer readings. The weights are determined by the variable 'comp_alpha'. The function also sets the values of 'omega_roll' and 'omega_yaw' based on the gyroscope readings.
// Example Call: complimentary_filter_roll();
void complimentary_filter_roll(){
    // Apply the complimentary filter to calculate the roll angle in degrees
  roll_deg = (1 - comp_alpha) * (roll_deg + g[0] * dT) + (comp_alpha) * (atan(a[1] / sqrt(pow(a[0], 2) + pow(a[2], 2)))) * (180 / pi);
    // Set the values of 'omega_roll' and 'omega_yaw' based on the gyroscope readings
  omega_roll = g[0];
  omega_yaw = g[2];
}

/////////////////////////////////////////////
// Function Name: mpu_int
// Input: None
// Output: This function does not return any value. However, it calls the functions 'mpu.update', 'getAcc', 'getGyro', and 'complimentary_filter_roll'.
// Logic: This function updates the MPU (Motion Processing Unit) sensor readings, gets the acceleration and gyroscope readings, and applies a complimentary filter to calculate the roll angle in degrees.
// Example Call: mpu_int();

void mpu_int(){
  // Update the MPU sensor readings
  mpu.update();
  
  // Get the acceleration readings
  getAcc();
  
  // Get the gyroscope readings
  getGyro();
  
  // Apply the complimentary filter to calculate the roll angle in degrees
  complimentary_filter_roll(); 
  }

/////////////////////////////////////////////
// Function Name: dc_motor_init
// Input: None
// Output: This function does not return any value. However, it initializes the pins for controlling a DC motor.
// Logic: This function sets the mode of the pins 'enA', 'in1', and 'in2' as OUTPUT. These pins are used for controlling a DC motor. The 'enA' pin is used for controlling the speed of the motor, while the 'in1' and 'in2' pins are used for controlling the direction of the motor. The function also sets the initial state of the 'in1' and 'in2' pins as LOW, which means the motor is initially stopped.
// Example Call: dc_motor_init();

  void dc_motor_init(){
  // Set the mode of the pins as OUTPUT
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set the initial state of the pins as LOW
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW); 
}

//////////////////////////////////////////////
// Function Name: motor_control
// Input: pwm - This is the input parameter representing the Pulse Width Modulation (PWM) value for controlling the speed of the motor.
// Output: This function does not return any value. However, it controls the direction and speed of the motor based on the input PWM value.
// Logic: This function controls the direction of the motor based on the sign of the PWM value. If the PWM value is negative, the motor is set to move in one direction. If the PWM value is positive, the motor is set to move in the other direction. The speed of the motor is controlled by the absolute value of the PWM value.
// Example Call: motor_control(-255); // This will set the motor to move in one direction at maximum speed.

void motor_control(int pwm) {
  // If the PWM value is negative
  if (pwm < 0) {
    // Set the motor to move in one direction
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Take the absolute value of the PWM value
    pwm = -pwm;
  }
  else {
    // Set the motor to move in the other direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  // Control the speed of the motor by the PWM value
  analogWrite(enA, pwm);
}

////////////////////////////////////////////////
// Function Name: encoder
// Input: None
// Output: This function does not return any value. However, it modifies the global variables 'old_ticks' and 'yaw_deg'.
// Logic: This function reads the number of ticks from the DC encoder. If the number of ticks has changed, it updates the 'old_ticks' variable. It then calculates the angle 'theta' and the arc length 'arc' based on the number of ticks. It uses these values to calculate the yaw angle in degrees 'yaw_deg'. The yaw angle is then normalized to the range [0, 360).
// Example Call: encoder();

void encoder(){
  // Read the number of ticks from the DC encoder
  long ticks = DC_Encoder.read();
  
  // If the number of ticks has changed
  if (ticks != old_ticks) {
    // Update the 'old_ticks' variable
    old_ticks = ticks;
  }
  
  // Calculate the angle 'theta' based on the number of ticks
  float theta  = 0.5 * ticks;
  
  // Calculate the arc length 'arc' based on the angle 'theta'
  float arc = (theta*2*pi*3)/360;
  
  // Calculate the yaw angle in degrees 'yaw_deg' based on the arc length 'arc'
  yaw_deg = 100*(arc/36);
  
}

/**
 * Function Name: bo_motor_init
 * Input: None
 * Output: None
 * Logic: This function initializes the motor control pins and sets them to LOW.
 * Example Call: bo_motor_init();
 */
void bo_motor_init(){
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/**
 * Function Name: forward
 * Input: None
 * Output: None
 * Logic: This function sets the motor to move forward.
 * Example Call: forward();
 */
void forward(){
  analogWrite(enB, 255);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}

/**
 * Function Name: backward
 * Input: None
 * Output: None
 * Logic: This function sets the motor to move backward.
 * Example Call: backward();
 */
void backward(){
  analogWrite(enB, 180);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  
}

/**
 * Function Name: stop
 * Input: None
 * Output: None
 * Logic: This function stops the motor.
 * Example Call: stop();
 */
void stop(){
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/**
 * Function Name: remote_control
 * Input: None
 * Output: None
 * Logic: This function controls the motor based on the input from the remote control.
 * Example Call: remote_control();
 */
void remote_control(){
 if(data.left==1){
  yaw_setpoint = yaw_setpoint + 0.3;
 }
 else if(data.right==1){
   yaw_setpoint = yaw_setpoint - 0.3;
 }

   if( data.front == 1 ){
  forward();
 }
 else if(data.back==1 ){
   backward();
 }
 else {
  stop(); 
 }
}