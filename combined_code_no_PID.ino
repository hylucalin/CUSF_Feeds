// Description (BRIEF)

// Code to be used FIRSTLY ON AN UNCONNECTED MOTOR, and secondly on a DRY RUN ONLY.
// Test endpoint calibration and keyboard interface
// Do NOT start the motor (until PID code has been uploaded)
// Fluid run will be conducted via radio

// Instructions

//********LIBRARIES********//

#include "CytronMotorDriver.h"
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <PID_v1.h>

//********PIN DEFINITIONS********//

#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 2
#define ADS1115_ADDR 0x48

const int ENCODER_A_PIN = 6;
const int ENCODER_B_PIN = 7;

CytronMD motor(PWM_DIR, 46, 8); 

//********ENCODER SPECIFICATION AND INITIALISATION********//

const float ENCODER_PPR = 3895.9; 
ESP32Encoder encoder;

struct EncoderTelemetry {
  float angle;
  float rpm;
};

//******** PID EDITABLES - TUNING AND SETPOINT********//

double pidInput = 0.0;
double pidOutput = 0.0;
// Setpoint initially set as a constant
const float drop_Setpoint = 0.95;

const double Kp = 2.0;
const double Ki = 5.0;
const double Kd = 1.0;

//********PRESSURE CHANNEL SETUP AND CONVERSION + INITIALISATION********//

// ADS1115 analogue input channels in use.
#define ADC_CH_PRESSURE_A1 1   // ADS1115 A1
#define ADC_CH_PRESSURE_A3 3   // ADS1115 A3

// Measured resistor values.
// Circuit: sensor signal -> R_TOP -> ADS input node -> R_BOTTOM -> GND

const float R_TOP_OHMS_A1    = 4310.0f;
const float R_BOTTOM_OHMS_A1 = 8300.0f;

const float R_TOP_OHMS_A3    = 4300.0f;
const float R_BOTTOM_OHMS_A3 = 8160.0f;

const float DIVIDER_RATIO_A1 = R_BOTTOM_OHMS_A1 / (R_TOP_OHMS_A1 + R_BOTTOM_OHMS_A1);
const float DIVIDER_RATIO_A3 = R_BOTTOM_OHMS_A3 / (R_TOP_OHMS_A3 + R_BOTTOM_OHMS_A3);

// Typical 0.5-4.5 V pressure transducer mapping.
// Change these if your sensor is different.
const float SENSOR_MIN_V = 0.5f;
const float SENSOR_MAX_V = 4.5f;
const float SENSOR_FULL_SCALE_PSI = 150.0f;
const float PSI_BAR = 0.0689476f;

Adafruit_ADS1115 ads;

//********SAFETY VARIABLES********//

bool initialised = false;
bool running = false; 
bool ads_ok = false;
int ads_trials = 0;

//********STATE VARIABLES********//

long lastPosition = 0;

//********TIMING VARIABLES********//

unsigned long ADS_last_retry_ms = 0; 
unsigned long Encoder_lastTime = 0;
unsigned long last_sercon_warn_ms = 0;

//********Compiling Guard - Explicit Forward Declarations********//
bool initialiseADS1115();
float pressure_calculator(uint8_t CHANNEL, float DIVIDER_RATIO);
EncoderTelemetry encoder_read();
void serial_input_checker();
void serial_output(float pressure1, float pressure2, float pressure_drop, float setpoint, float motor_angle, float motor_rpm);
float endstop_calibration();

void setup() {

  Serial.begin(115200);
  // Initialisng I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); 
  ads_ok = initialiseADS1115();
  // Initialising Encoder Communication
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder.clearCount();

  motor.setSpeed(0);

}

void loop() {

  if (!ads_ok && (millis() - ADS_last_retry_ms > 500)) {
    ADS_last_retry_ms = millis();

    
    Serial.println("ADSOFF");
    Wire.end(); // Clear the hardware I2C engine blocks completely
    
    // Restart the I2C bus cleanly on your specific pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000); 

    
    ads_ok = initialiseADS1115();
    
    if (ads_ok) {
      Serial.println("ADSON");
    }
    
    
  }

  serial_input_checker();

  EncoderTelemetry encoder_data;
  encoder_data = encoder_read();
  float current_degrees = encoder_data.angle;
  float current_rpm = encoder_data.rpm;

  float bar_a1 = pressure_calculator(ADC_CH_PRESSURE_A1, DIVIDER_RATIO_A1);
  float bar_a3 = pressure_calculator(ADC_CH_PRESSURE_A3, DIVIDER_RATIO_A3);

  float p13_drop = 0;

  if (bar_a1 > 12.0f || bar_a3 > 12.0f) { // High limit example
  running = false;
  motor.setSpeed(0);
  Serial.println("ERR_OVERPRESSURE");
}

if (bar_a1 < 0.01f || bar_a3 < 0.01f) { // Noise-tolerant zero safety check
  p13_drop = 0.0f;
} else {
  p13_drop = (bar_a1 > bar_a3) ? bar_a3 / bar_a1 : bar_a1 / bar_a3;
}


  if (bar_a1 == 0) {
    p13_drop = 0;
    
  }
  else if (bar_a3 == 0) {
    p13_drop = 0;
    
  }

  else {
    p13_drop = (bar_a1 > bar_a3) ? (float)bar_a3/bar_a1 : (float)bar_a1/bar_a3 ;

  }


   // Active Execution Logic
  if (running && initialised) {
    // PID calculations and motor writes will go here later
  } else if (!running) {
    motor.setSpeed(0); // Safety Interlock
  }


  serial_output(bar_a1,bar_a3,p13_drop,drop_Setpoint,current_degrees,current_rpm);



}

//********FUNCTIONS********//

//********ADS-RELATED FUNCTIONS********//

bool initialiseADS1115() {
  // Finding the ADS
  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    return false;
  }
  // GAIN_ONE gives +/-4.096 V full-scale.
  // With ADS1115 powered from 3.3 V, keep the physical input below 3.3 V.
  ads.setGain(GAIN_ONE); 
  return true;
}

float pressure_calculator(uint8_t CHANNEL, float DIVIDER_RATIO) {
  float raw_aX = ads_ok ? (float)ads.readADC_SingleEnded(CHANNEL) : 0.0f;
  float adcV_aX = ads_ok ? ads.computeVolts(raw_aX) : 0.0f;
  float sensorV_aX = adcV_aX / DIVIDER_RATIO;
  float bar_aX = (sensorV_aX - SENSOR_MIN_V)* SENSOR_FULL_SCALE_PSI * PSI_BAR / (SENSOR_MAX_V - SENSOR_MIN_V);
  if (bar_aX < 0.0f) bar_aX = 0.0f;


  return bar_aX;
}

//********ENCODER-RELATED FUNCTIONS********//

EncoderTelemetry encoder_read() {

  long currentPosition = encoder.getCount();

  unsigned long currentTime = millis(); 

  float degrees = (float)currentPosition * 360.0f / ENCODER_PPR;
  
  long deltaTicks = currentPosition - lastPosition;
  unsigned long deltaTime = currentTime - Encoder_lastTime;
  float currentRPM = 0.0f;

  // Prevent division by zero if loop runs too fast
  if (deltaTime > 0) {
    float deltaTimeMins = (float)deltaTime / 60000.0f;
    currentRPM = ((float)deltaTicks / ENCODER_PPR) / deltaTimeMins;
    
  }

  else {
    Serial.println("DIVZEROENC");
  }
  
  lastPosition = currentPosition;
  Encoder_lastTime = currentTime;

  // SORT TIMING SCOPE AND CALLS

  return {degrees, currentRPM};
}

//********MOTOR-RELATED FUNCTIONS********//


// ADD PID
//********SERIAL COMMS/SAFETY-RELATED FUNCTIONS********//

float endstop_calibration() {

  // MAKE SURE MAX ANGLE IS CLOSED POSITION, MIN ANGLE IS OPEN POSITION
  // DETERMINE WHETHER +VE DIRECTION IS CLOCKWISE OR ANTICLOCKWISE

  unsigned long calib_time = millis();

  motor.setSpeed(20);

  float deltaDeg = 20;
  float lastDeg = 0;

  while (abs(deltaDeg) > 0.01) {

    EncoderTelemetry telemetry;
    telemetry = encoder_read();
    float calib_current_degrees = telemetry.angle;
    float calib_current_rpm = telemetry.rpm;
    deltaDeg = calib_current_degrees - lastDeg;

    lastDeg = calib_current_degrees;
    if (millis() - calib_time > 30000) {
      Serial.println("CALIBTIMEOUT")
      return -1.0f;
      // somehow flag calibration failure
    }
    delay(20);
  
  }
  
  motor.setSpeed(0);

  float max_angle = lastDeg;

  calib_time = millis();

  motor.setSpeed(-20);

  deltaDeg = 20;

  while (abs(deltaDeg) > 0.01) {
    EncoderTelemetry telemetry;
    telemetry = encoder_read();
    float calib_current_degrees = telemetry.angle;
    float calib_current_rpm = telemetry.rpm;
    deltaDeg = calib_current_degrees - lastDeg;

    lastDeg = calib_current_degrees;

    if (millis() - calib_time > 30000) {
      Serial.println("CALIBTIMEOUT")
      return -1.0f;
    }

    // somehow flag calibration failure

    delay(20);
  
  }

  motor.setSpeed(0);

  float min_angle = lastDeg;

  float angle_range = abs(max_angle - min_angle);

  encoder.clearCount();

  return angle_range;

}

void serial_input_checker() {
  // NEED TO ADD THIS FUNCTION TO VOID LOOP!

  if (Serial.available() > 0) {
    char incomingByte = Serial.read();

    if (running == false) {
      if (initialised == false) {

        if (incomingByte == 'i') {

          Serial.println("INITING");

          float angle_endpoint = endstop_calibration();

           if (angle_endpoint < 0.0f) {
            initialised = false;
            Serial.println("ERR_CALIB_FAILED"); // Alerts Python host to abort
          } 
          
          else {
            initialised = true;
            Serial.print("INITED,"); // Added separator comma for parsing stability
            Serial.println(angle_endpoint);


        }

        else {
          Serial.println("INITNOW");
          
        }
      }

      else {
        if (incomingByte == 's') {
          running = true;
          Serial.println("MOTSTART");
          // Insert the PID function here
        }

        else {
          Serial.println("NEEDSTART");
          
        }

      }
    }

    else {
      if (incomingByte == ' ') {
        running = false;
        motor.setSpeed(0);
        Serial.println("ESTOP");

        // better stop procedure?
        
        
      }
    }
  }

  else {
    if (millis() - last_sercon_warn_ms > 2000) {
      last_sercon_warn_ms = millis();
      Serial.println("SERCON"); 
    }
  
    
  }

}

void serial_output(float pressure1, float pressure2, float pressure_drop, float setpoint, float motor_angle, float motor_rpm ) {

  Serial.print(pressure1, 4);      
  Serial.print(",");             
  Serial.print(pressure2, 4);   
  Serial.print(",");             
  Serial.print(pressure_drop, 5);   
  Serial.print(",");
  Serial.print(setpoint,5);  
  Serial.print(",");
  Serial.print(motor_angle, 5);
  Serial.print(",");
  Serial.println(motor_rpm, 5);
     
}




// deal with temporary data losses!
// NEED TO ADD PRESSURE SENSOR ERRORS
// deltaDeg 0.01 - gear backlash???






