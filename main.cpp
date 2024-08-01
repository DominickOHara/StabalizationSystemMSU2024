// Made by Dominick O'Hara
// April 30, 2024

// PINS FOR HARDWARE AND DEALING WITH ICM_20948
// GND PIN TO GND ON ARDUINO
// VIN PIN TO 5V PIN ON ARDUINO
// DA PIN TO SDA PIN ON ARDUINO
// CL PIN TO SCL PIN ON ARDUINO

// MAKE SURE TO DO 
//  Serial.println(Serial.paritytype());
//  Serial.println(Serial.stopbits());
// to figure out the parity and stop bits used by
// the arduino. This is to make sure that the sim
// uses the correct end bits and parity.
// the parity value 0 is even, and 1 is odd
// Although, it seems to work with the wrong end bits
// and the serial library used in the sim dose not support 0 end bits
// but it still reads serial data despite it being set to one

#include <ICM_20948.h>
#include <stdlib.h>
#include <math.h>


/***************************************************************
                  CONSTANTS / MACROS
***************************************************************/
// uncomment #define SIM_MODE_ON to send data to serial so the simulation can recieve it;
// if commented, project will compile for the actual rocket and actuate the servo-motors on the canards
#define SIM_MODE_ON 
// uncomment #define DEBUG if debugging -> this will enable console messages for issues or errors
#define DEBUG
#define AD0_VAL 1
ICM_20948_I2C ICM_Obj;
// PI is a macro which is defined somewhere; im not sure.
// this just makes sure PI is defined if it is not present
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;
const float PI2 = 2 * PI;

/** @brief enum which stores the pin used to rotate each canard fin */
enum CANARD {
  CANARD_PIN_1 = 15, // SET TO ACTUAL PIN NUMBER
  CANARD_PIN_2 = 15, // SET TO ACTUAL PIN NUMBER
  CANARD_PIN_3 = 15, // SET TO ACTUAL PIN NUMBER
  CANARD_PIN_4 = 15  // SET TO ACTUAL PIN NUMBER
};


/***************************************************************
                    UTILITY FUNCTIONS
***************************************************************/
/**
 * @brief Converts a given array of floats into a string
 * If there is not enough room, it will not pack the remaining floats  
 *
 * @param str The C-String to store the floats
 * @param str_size The length of the string
 * @param floats The array of floats
 * @param float_c The amount of floats in the floats array
 * @param float_chars The amount of characters for each float
 */
void PackFloatsInStr(char* str, size_t str_size, float* floats, size_t float_c, size_t float_chars) 
{
  const char* FORMAT = "%f";
  for (int i = 0; i < float_c && str_size >= float_chars; i++) {
      snprintf(str, str_size, FORMAT, floats[i]);
      str_size -= float_chars;
      str += float_chars;
  }
}

/**
 * @brief makes sure that the degree measure is between 0 and 360
 * @param degrees A refrence to the degree measure
 */
inline void fixDegree(float& degrees) {
  degrees = (degrees < 0.0f)? degrees + 180.0f : degrees; // add half full rotation if negative
  degrees = fabsf(degrees); // float absolute value func
  degrees = fmodf(degrees, 360.0f); // float modulo func (% for integers) 
}

/**
 * @brief makes sure that the radian measure is between 0 and 2PI
 * @param radians A refrence to the radian measure
 */
inline void fixRadian (float& radians) {
  radians = (radians < 0.0f)? radians + PI : radians; // add half full rotation if negative
  radians = fabsf(radians); // float absolute value func
  radians = fmodf(radians, PI2); // float modulo func (% for integers)
}


/***************************************************************
              ROCKET MANIPULATION / FUNCTIONS
***************************************************************/
/** 
 * @brief namespace that holds all the data of the rocket
 */
namespace Rocket {
  float pitch = 0.0f;
  float roll = 0.0f;
  float yaw = 0.0f;
  float canard_rotations[4] = {0.0f};
}

/** 
 * @brief Sends the rocket data over serial; 
 * floats 0, 1, 2 are orientation, 
 * floats 3, 4, 5, 6 are canard fin rotations (radians) 
 */
void SendDataToSerial()
{
  float float_data_arr[] = {
    Rocket::pitch,
    Rocket::roll,
    Rocket::yaw,
    Rocket::canard_rotations[0],
    Rocket::canard_rotations[1],
    Rocket::canard_rotations[2],
    Rocket::canard_rotations[3]
  };

  const int FLOAT_CHARS = 8; // amount of characters used for each float
  const int FLOAT_ARR_ELEMS = sizeof(float_data_arr) / sizeof(float); // amount of elements in the array 'float_data_arr'
  const int DATA_STR_SIZE = FLOAT_CHARS * FLOAT_ARR_ELEMS + 1; // +1 for null terminator

  char rocketDataStr[DATA_STR_SIZE] = { 0 };
  PackFloatsInStr(rocketDataStr, DATA_STR_SIZE, float_data_arr, FLOAT_ARR_ELEMS, FLOAT_CHARS);
  Serial.println(rocketDataStr);
}

/**
 * @param pitch Pitch angle in radians (x-axis)
 * @param roll Roll angle in radians (y-axis)
 * @param yaw Yaw angle in radians (z-axis)
 */
inline void SetOrientation(float& pitch, float& roll, float& yaw) {
  Rocket::pitch = pitch;
  Rocket::roll = roll;
  Rocket::yaw = yaw;
}

/**
 * @param pitch_ret refrence used to retrieve the pitch angle in radians (x-axis)
 * @param roll_ret refrence used to retrieve the roll angle in radians (y-axis)
 * @param yaw_ret refrence used to retrieve the yaw angle in radians (z-axis)
 */
inline void GetOrientation(float& pitch_ret, float& roll_ret, float& yaw_ret) {
  pitch_ret = Rocket::pitch;
  roll_ret = Rocket::roll;
  yaw_ret = Rocket::yaw;
}

/**
 * @brief Sets the new rotations (in radians) 
 * to the canard fins and makes sure each rot within 0 - 2PI
 * @param rad The 4 new rotations to the canard fins (in radians)
 */
void SetCanardRotations(float rad[4]) {
  for (int i = 0; i < 4; i++) {
    fixRadian(rad[i]);
    Rocket::canard_rotations[i] = rad[i];
  }
}

/**
 * @brief Gets the orientation from the sensors and sets the orientation
 * @param deltaTime the time inbetween each loop itteration
 */
void SetOrientationFromSensors(float deltaTime)
{
  if (ICM_Obj.dataReady())
  {
    ICM_Obj.getAGMT();
    // equations from https://stackoverflow.com/questions/23009549/roll-pitch-yaw-calculation
    float accX = ICM_Obj.accX();
    float accY = ICM_Obj.accY();
    float accZ = ICM_Obj.accZ();

    float magX = ICM_Obj.magX();
    float magY = ICM_Obj.magY();
    float magZ = ICM_Obj.magZ();

    float roll_a = atan2f(accY, accZ);
    float pitch_a = atan2f(-accX, sqrt(accY*accY + accZ*accZ));
    float yaw_m = atan2f(magY, magX);


    // we use + as pitch roll and yaw are negative
    float pitch = roll_a;//Rocket::pitch + pitchAcc * deltaTime;
    float roll = roll_a;//Rocket::roll + rollAcc * deltaTime;
    float yaw = yaw_m;//Rocket::yaw + yawAcc * deltaTime;
    fixRadian(pitch);
    fixRadian(roll);
    fixRadian(yaw);
    SetOrientation(pitch, roll, yaw);
  }
}

/**
 * @brief Sends an electrical signal to rotate a 
 * servo-motor for the specific canard fin
 * @param canardPin The canard pin to actuate
 * @param radians The radian measure to rotate the canard fin
 */
void ActuateCanard(CANARD canardPin, float radians)
{
  // canard fins take degrees, not radians -> convert to degrees
  float degrees = radians * RAD2DEG;
  // TODO write code to actuate the specific servo motor
}


/***************************************************************
                ARDUINO ENTRY POINT / LOOP
***************************************************************/
unsigned long last_tick = 0;

void StabilizationSystem(float deltaTime)
{
  for (int i = 0; i < 4; i++) {
    // rotate each canard 24 degrees per second -> converts to radians
    Rocket::canard_rotations[i] += 24.0f * DEG2RAD * deltaTime;
    fixRadian(Rocket::canard_rotations[i]); // make sure radians are within bounds
  }
}

void setup() {
  // initalize serial and wire
  Serial.begin(9600);
  while (!Serial)
    ;
  Wire.begin();
  Wire.setClock(400000);

  bool init = false;
  while (!init) {
    ICM_Obj.begin(Wire, AD0_VAL);
    if (ICM_Obj.status != ICM_20948_Stat_Ok) {
      #ifdef DEBUG
        Serial.println("Failed to init ICM, trying again...");
      #endif
      delay(100);
    } else {
      init = true;
    }
  }

  // init last_tick so delta_time can be calculated
  last_tick = millis();
}


void loop() {
  // get delta time
  unsigned long current_tick = millis();
  float deltaTime = (current_tick - last_tick) / 1000.0f;
  last_tick = current_tick;
  SetOrientationFromSensors(deltaTime);
  StabilizationSystem(deltaTime);
  // send data to serial (if compiled to work with simulation)
  #ifdef SIM_MODE_ON
    SendDataToSerial();
  // Actuate the Canard fins (rotate servos) (if compiled to work with actual rocket)
  #else
    ActuateCanard(CANARD_PIN_1, Rocket::canard_rotations[0]);
    ActuateCanard(CANARD_PIN_2, Rocket::canard_rotations[1]);
    ActuateCanard(CANARD_PIN_3, Rocket::canard_rotations[2]);
    ActuateCanard(CANARD_PIN_4, Rocket::canard_rotations[3]);
  #endif
  delay(1);
}