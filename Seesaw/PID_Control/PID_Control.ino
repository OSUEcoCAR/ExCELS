#include <Servo.h>
  
/*SERVO POSITION CONSTANTS*/
#define SERVO_OFFSET 1760   //center position
#define SERVO_MIN 1460 //1360
#define SERVO_MAX 2060 //2160
  
/*REFERENCE RANGE*/
#define REFERENCE_MIN 5.0 
#define REFERENCE_MAX 13.0
  
/*DELTA T*/
#define DT 0.001  //seconds
  
/*PID PARAMETERS*/
#define Kp 0.3    //proportional coefficient
#define Ki 0.1    //integral coefficient
#define Kd 0.1    //derivative coefficient
  
/*UPSCALING TO Servo.writeMilliseconds*/
#define OUTPUT_UPSCALE_FACTOR 10
  
/*EMA ALPHAS*/
#define SENSOR_EMA_a 0.05
#define SETPOINT_EMA_a 0.01
  
/*SENSOR SPIKE NOISE HANDLING*/
#define SENSOR_NOISE_SPIKE_THRESHOLD 15
#define SENSOR_NOISE_LP_THRESHOLD 300
  
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
  
Servo myservo;
  
/*ARDUINO PINS*/
int sensor_pin = 0;
int pot_pin = 1;
int servo_pin = 3;
  
/*EMA VARIABLE INITIALIZATIONS*/
float sensor_filtered = 0.0;
int pot_filtered = 0;
  
/*GLOBAL SENSOR SPIKE NOISE HANDLING VARIABLES*/
int last_sensor_value = analogRead(sensor_pin);
int old_sensor_value = analogRead(sensor_pin);
  
/*GLOBAL PID VARIABLES*/
float previous_error = 0;
float integral = 0;
  
void setup() {
  Serial.begin(115200);
  myservo.attach(servo_pin);
}
  
void loop() {
  /*START DELTA T TIMING*/
  unsigned long my_time = millis();
  
  /*READ POT AND RUN POT EMA*/
  int pot_value = analogRead(pot_pin);
  pot_filtered = (SETPOINT_EMA_a*pot_value) + ((1-SETPOINT_EMA_a)*pot_filtered);
    
  /*MAP POT POSITION TO CM SETPOINT RANGE*/
  float setpoint = mapfloat((float)pot_filtered, 0.0, 1024.0, REFERENCE_MIN, REFERENCE_MAX);
  
  /*READ SENSOR DATA*/
  int sensor_value = analogRead(sensor_pin);
  
  /*REMOVE SENSOR NOISE SPIKES*/
  if(abs(sensor_value-old_sensor_value) > SENSOR_NOISE_LP_THRESHOLD || sensor_value-last_sensor_value < SENSOR_NOISE_SPIKE_THRESHOLD){  //everything is in order
    old_sensor_value = last_sensor_value;
    last_sensor_value = sensor_value;
  }
  else{                               //spike detected - set sample equal to last
    sensor_value = last_sensor_value;
  }
    
  /*LINEARIZE SENSOR OUTPUT TO CENTIMETERS*/
  sensor_value = max(1,sensor_value);         //avoid dividing by zero
  float cm = (2598.42/sensor_value) - 0.42;
    
  /*RUN SENSOR EMA*/
  sensor_filtered = (SENSOR_EMA_a*cm) + ((1-SENSOR_EMA_a)*sensor_filtered);
  
  /*PID CONTROLLER*/
  float error = setpoint - sensor_filtered;
  integral = integral + error*DT;
  float derivative = (error - previous_error)/DT;
  float output = (Kp*error + Ki*integral + Kd*derivative)*OUTPUT_UPSCALE_FACTOR;
  previous_error = error;
  
  /*PRINT TO SERIAL THE TERM CONTRIBUTIONS*/
  //Serial.print(Kp*error);
  //Serial.print(' ');
  //Serial.print(Ki*integral);
  //Serial.print(' ');
  //Serial.println(Kd*derivative);
  
  /*PRINT TO SERIAL FILTERED VS UNFILTERED SENSOR DATA*/
  //Serial.print(sensor_filtered);
  //Serial.print(' ');
  //Serial.println(cm);
  
  /*PREPARE AND WRITE SERVO OUTPUT*/
  int servo_output = round(output)+SERVO_OFFSET;
    
  if(servo_output < SERVO_MIN){ 
    //saturate servo output at min/max range 
    servo_output = SERVO_MIN; 
    }
    else if(servo_output > SERVO_MAX){
    servo_output = SERVO_MAX;
  }
    
  myservo.writeMicroseconds(servo_output);  //write to servo
  
  /*PRINT TO SERIAL SETPOINT VS POSITION*/
  Serial.print(sensor_filtered);
  Serial.print(' ');
  Serial.println(setpoint);
  
  /*WAIT FOR DELTA T*/
  //Serial.println(millis() - my_time);
  while(millis() - my_time < DT*1000);
}
  
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
