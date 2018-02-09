/*
 * OpenFlexure Microscope motor controoler
 * Adapted from James Sharkey's Arduino-powered stepper driver
 * Substantially rewritten Richard Bowman, Spring 2017
 * Written by James Sharkey, Spring 2015
 * Based on code written by Richard Bowman, Autumn 2014
 * Based on code written by Fergus Riche, 2015
 * For use with the 3D printed microscope:
 * https://github.com/rwb27/openflexure_microscope/
 * 
 * (c) Richard Bowman & James Sharkey, Released under GPL v3, 2017
 */
#include <StepperF_alt.h>   //Fergus's hacked stepper library
#include <assert.h>
#include <EEPROM.h>

// LIGHT SENSOR SUPPORT
// Uncomment (exactly) one of the lines below to enable support for that sensor.
//#define ADAFRUIT_TSL2591
//#define ADAFRUIT_ADS1115

#ifdef ADAFRUIT_TSL2591
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#define LIGHT_SENSOR
#endif
#ifdef ADAFRUIT_ADS1115
#include <Wire.h>
#include "Adafruit_ADS1015.h"
#define LIGHT_SENSOR
#endif

#define EACH_MOTOR for(int i=0; i<n_motors; i++)

// The array below has 3 stepper objects, for X,Y,Z respectively
const int n_motors = 3;
long min_step_delay;
const int min_step_delay_eeprom = sizeof(long)*n_motors;
long ramp_time;
const int ramp_time_eeprom = sizeof(long)*(n_motors+1);
Stepper* motors[n_motors];
signed long current_pos[n_motors];
int steps_remaining[n_motors];

// We'll use this input buffer for serial comms
const int INPUT_BUFFER_LENGTH = 64;
char input_buffer[INPUT_BUFFER_LENGTH];

int command_prefix(String command, const char ** prefixes, int n_prefixes){
  // Check if the command starts with any of the prefixes in the list.
  // returns the index if so, otherwise returns -1
  for(int i=0; i<n_prefixes; i++){
    if(command.startsWith(prefixes[i])){
      return i;
    }
  }
  return -1;
}

void setup() {
  // initialise serial port
  Serial.begin(115200);
  
  // get the stepoper objects from the motor shield objects
  motors[0] = new Stepper(8, 13, 12, 11, 10);
  motors[1] = new Stepper(8, 9, 8, 7, 6);
  motors[2] = new Stepper(8, 5, 4, 3, 2);
  
  EACH_MOTOR{
    motors[i]->setSpeed(1*8.0/4096.0); //using Fergus's speed for now, though this is ignored...
    steps_remaining[i]=0;
    EEPROM.get(sizeof(long)*i, current_pos[i]); //read last saved position from EEPROM
    //current_pos[i] = 0; //alternatively, reset on power cycle!
  }

  EEPROM.get(min_step_delay_eeprom, min_step_delay);
  if(min_step_delay < 0){ // -1 seems to be what we get if it's uninitialised.
    min_step_delay = 1000;
    EEPROM.put(min_step_delay_eeprom, min_step_delay); 
  }
  EEPROM.get(ramp_time_eeprom, ramp_time);
  if(ramp_time < 0){ // -1 seems to be what we get if it's uninitialised.
    ramp_time = 0;
    EEPROM.put(ramp_time_eeprom, ramp_time); 
  }
  #ifdef LIGHT_SENSOR
  setup_light_sensor();
  #else
  Serial.println(F("OpenFlexure Motor Board v0.3"));
  #endif /* LIGHT_SENSOR */
  
}

void stepMotor(int motor, int dx){
  //make a single step of a single motor.  
  current_pos[motor] += dx;
  motors[motor]->stepMotor(((current_pos[motor] % 8) + 8) % 8); //forgive the double-modulo; I need 0-7 even for -ve numbers
}

void releaseMotor(int motor){
  //release the stepper motor (de-enegrise the coils)
  motors[motor]->stepMotor(8);
}

void print_position(){
  EACH_MOTOR{
    if(i>0) Serial.print(" ");
    Serial.print(current_pos[i]);
  }
  Serial.println();
}

void move_axes(int displacement[n_motors]){
  // move all the axes in a nice move
  // split displacements into magnitude and direction, and find max. travel
    int max_steps = 0;
    int dir[n_motors];
    EACH_MOTOR{
      dir[i] = displacement[i] > 0 ? 1 : -1;
      displacement [i] *= dir[i];
      if(displacement[i]>max_steps) max_steps=displacement[i];
    }
    // scale the step delays so the move goes in a straight line, with >=1 motor
    // running at max. speed.
    float step_delay[n_motors];
    EACH_MOTOR if(displacement[i]>0){
      step_delay[i] = float(max_steps)/float(displacement[i])*float(min_step_delay);
    }else{
      step_delay[i] = 9999999999;
    }
    // actually make the move
    bool finished = false;
    int distance_moved[n_motors];
    EACH_MOTOR distance_moved[i] = 0;
    float start = (float) micros();
    float final_scaled_t = (float) max_steps * min_step_delay; //NB total time taken will be final_scaled_t + 2*ramp_time
    while(!finished){
      float elapsed_t = (float) micros() - start;
      float scaled_t; //scale time to allow for acceleration
      if(ramp_time > 0){
        // if a ramp time is specified, accelerate at a constant acceleration for the
        // ramp time, then move at constant (maximum) speed, then decelerate.  If the
        // move is shorter than 2*ramp_time, accelerate then decelerate.
        float remaining_t = final_scaled_t + ramp_time - elapsed_t;
        if(elapsed_t < ramp_time && remaining_t > elapsed_t){ //for the first ramp_time, gradually accelerate
          scaled_t = elapsed_t*elapsed_t/(2*ramp_time);
        }else if(remaining_t < ramp_time){ 
          scaled_t = final_scaled_t - remaining_t*remaining_t/(2*ramp_time);
        }else{
          scaled_t = elapsed_t - ramp_time/2;
        }
      }else{
        scaled_t = elapsed_t;
      }
      finished = true;
      EACH_MOTOR{
        if(distance_moved[i] < displacement[i]){
          finished = false; //only if all axes are done are we truly finished.
          
          // check if it's time to take another step and move if needed.
          if(scaled_t > ((float)distance_moved[i] + 0.5) * step_delay[i]){
            stepMotor(i, dir[i]);
            distance_moved[i]++;
          }
        }
      }
      //delayMicroseconds(2000);
    }
    EEPROM.put(0, current_pos);
}

#ifdef ADAFRUIT_TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

void setup_light_sensor(){
  if (tsl.begin()) 
  {
    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    Serial.println(F("OpenFlexure Motor Board (with TSL2591 support)"));
  } 
  else 
  {
    Serial.println(F("No light sensor found.  NB your board will start up faster if you recompile without light sensor support."));
  }
}

void print_light_sensor_gain(){
  // Print the current gain value of the light sensor
  Serial.print  (F("light sensor gain "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
}
void print_light_sensor_gain_values(){
  // Print the allowable gain values of the light sensor
  Serial.println(F("light sensor gains: 1x, 25x, 428x, 9876x"));
}

void set_light_sensor_gain(int newgain){
  // Set the current gain value of the light sensor, and print a confirmation.
  switch(newgain){
    case 1:
      tsl.setGain(TSL2591_GAIN_LOW);
      break;
    case 25:
      tsl.setGain(TSL2591_GAIN_MED);
      break;
    case 428:
      tsl.setGain(TSL2591_GAIN_HIGH);
      break;
    case 9876:
      tsl.setGain(TSL2591_GAIN_MAX);
      break;
    default:
      Serial.println(F("Error: gain may only be 1, 25, 428, or 9876."));
      return;
  }
  print_light_sensor_gain();
}

void print_light_sensor_integration_time(){
  // Print the current integration time in milliseconds.
  Serial.print  (F("light sensor integration time "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
}

void print_light_sensor_intensity(){
  // Print the current light value
  uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  Serial.println(x, DEC);
}
#endif // ADAFRUIT_TSL2591


#ifdef ADAFRUIT_ADS1115
Adafruit_ADS1115 ads; // pass in a number for the sensor identifier (for your use later)

void setup_light_sensor(){
  ads.begin();
  ads.setGain(GAIN_ONE);
  Serial.println(F("OpenFlexure Motor Board (with ADS1115 support)"));
}

void print_light_sensor_gain(){
  // Print the current gain value of the light sensor
  Serial.print  (F("light sensor gain "));
  adsGain_t gain = ads.getGain();
  switch(gain)
  {
    case GAIN_TWOTHIRDS:
      Serial.println(F("0.66x (specify 0)"));
      break;
    case GAIN_ONE:
      Serial.println(F("1x"));
      break;
    case GAIN_TWO:
      Serial.println(F("2x"));
      break;
    case GAIN_FOUR:
      Serial.println(F("4x"));
      break;
    case GAIN_EIGHT:
      Serial.println(F("8x"));
      break;
    case GAIN_SIXTEEN:
      Serial.println(F("16x"));
      break;
  }
}
void print_light_sensor_gain_values(){
  // Print the allowable gain values of the light sensor
  Serial.println(F("light sensor gains: 0.66x, 1x, 2x, 4x, 8x, 16x"));
}

void set_light_sensor_gain(int newgain){
  // Set the current gain value of the light sensor, and print a confirmation.
  switch(newgain){
    case 0:
      ads.setGain(GAIN_TWOTHIRDS);
      break;
    case 1:
      ads.setGain(GAIN_ONE);
      break;
    case 2:
      ads.setGain(GAIN_TWO);
      break;
    case 4:
      ads.setGain(GAIN_FOUR);
      break;
    case 8:
      ads.setGain(GAIN_EIGHT);
      break;
    case 16:
      ads.setGain(GAIN_SIXTEEN);
      break;
    default:
      Serial.println(F("Error: gain may only be 0, 1, 2, 4, 8, 16 (0 means 2/3)."));
      return;
  }
  print_light_sensor_gain();
}

void print_light_sensor_integration_time(){
  // Print the current integration time in milliseconds.
  Serial.println(F("integration time not supported for ADS1115"));
}

void print_light_sensor_intensity(){
  // Print the current light value
  // uint16_t x = ads.readADC_SingleEnded(0); //single ended measurement on pin 0
  uint16_t x = ads.readADC_Differential_0_1(); //differential measurement on pins 0,1
  Serial.println(x, DEC);
}

#endif // ADAFRUIT_ADS1115

void loop() {
  // wait for a serial command and read it
  int received_bytes = Serial.readBytesUntil('\n',input_buffer,INPUT_BUFFER_LENGTH-1);
  if(received_bytes > 0){
    input_buffer[received_bytes] = '\0';
    String command = String(input_buffer);
//    Serial.println("Got command: <"+command+">");

    const char* single_axis_moves[3] = {"mrx ", "mry ", "mrz "};
    int axis = command_prefix(command, single_axis_moves, 3);
    if(axis >= 0){
      int preceding_space = command.indexOf(' ',0);
      if(preceding_space <= 0) Serial.println("Bad command.");
      int n_steps = command.substring(preceding_space+1).toInt();
      int displacement[n_motors];
      EACH_MOTOR displacement[i]=0;
      displacement[axis]=n_steps;
      move_axes(displacement);
      Serial.println("done");
      return;
    }
    if(command.startsWith("move_rel ") or command.startsWith("mr ")){ //relative move
      int preceding_space = -1;
      int displacement[n_motors];
      EACH_MOTOR{ //read three integers and store in steps_remaining
        preceding_space = command.indexOf(' ',preceding_space+1);
        if(preceding_space<0){
          Serial.println(F("Error: command is mr <int> <int> <int>"));
          break;
        }
        displacement[i] = command.substring(preceding_space+1).toInt();
      }
      move_axes(displacement);
      Serial.println("done.");
      return;
    }

    if(command.startsWith("release")){ //release steppers (de-energise coils)
      EACH_MOTOR releaseMotor(i);
      Serial.println("motors released");
      return;
    }
    if(command.startsWith("p?") or command.startsWith("position?")){
      print_position();
      return;
    }
    if(command.startsWith("ramp_time ")){
      int preceding_space = command.indexOf(' ',0);
      if(preceding_space <= 0){
        Serial.println("Bad command.");
        return;
      }
      ramp_time = command.substring(preceding_space+1).toInt();
      EEPROM.put(ramp_time_eeprom, ramp_time);
      Serial.println("done.");
      return;
    }
    if(command.startsWith("ramp_time?")){
      Serial.print("ramp time ");
      Serial.println(ramp_time);
      return;
    }
    if(command.startsWith("min_step_delay ") || command.startsWith("dt ")){
      int preceding_space = command.indexOf(' ',0);
      if(preceding_space <= 0){
        Serial.println("Bad command.");
        return;
      }
      min_step_delay = command.substring(preceding_space+1).toInt();
      EEPROM.put(min_step_delay_eeprom, min_step_delay);
      Serial.println("done.");
      return;
    }
    if(command.startsWith("min_step_delay?") || command.startsWith("dt?")){
      Serial.print("minimum step delay ");
      Serial.println(min_step_delay);
      return;
    }
    if(command.startsWith("zero")){
      EACH_MOTOR current_pos[i]=0;
      Serial.println("position reset to 0 0 0");
      EEPROM.put(0, current_pos);
      return;
    }
    #ifdef LIGHT_SENSOR
    if(command.startsWith("light_sensor_gain?")){
      print_light_sensor_gain();
      return;
    }
    if(command.startsWith("light_sensor_gain_values?")){
      print_light_sensor_gain_values();
      return;
    }
    if(command.startsWith("light_sensor_gain ")){
      int preceding_space = command.indexOf(' ',0);
      if(preceding_space <= 0){
        Serial.println("Bad command.");
        return;
      }
      set_light_sensor_gain(command.substring(preceding_space+1).toInt());
      return;
    }
    if(command.startsWith("light_sensor_integration_time?")){
      print_light_sensor_integration_time();
      return;
    }
    if(command.startsWith("light_sensor_intensity?")){
      print_light_sensor_intensity();
      return;
    }
    #endif //LIGHT_SENSOR
    if(command.startsWith("help")){
      Serial.println(F("OpenFlexure Motor Controller firmware v0.1"));
      Serial.println(F("Commands (terminated by a newline character):"));
      Serial.println(F("mrx ??? - relative move in x"));
      Serial.println(F("mrx ??? - relative move in x"));
      Serial.println(F("mrx ??? - relative move in x"));
      Serial.println(F("mr ??? ??? ??? - relative move in all 3 axes"));
      Serial.println(F("release - de-energise all motors"));
      Serial.println(F("p? - print position (3 space-separated integers"));
      Serial.println(F("ramp_time ??? - set the time taken to accelerate/decelerate in us"));
      Serial.println(F("min_step_delay ??? - set the minimum time between steps in us."));
      Serial.println(F("dt ??? - set the minimum time between steps in us."));
      Serial.println(F("ramp_time? - get the time taken to accelerate/decelerate in us"));
      Serial.println(F("min_step_delay? - get the minimum time between steps in us."));
      Serial.println(F("zero - set the current position to zero."));
      #ifdef LIGHT_SENSOR
      #if defined ADAFRUIT_TSL2591
      Serial.println(F("Compiled with Adafruit TSL2591 support"));
      #elif defined ADAFRUIT_ADS1115
      Serial.println(F("Compiled with Adafruit ADS1115 support"));
      #endif
      Serial.println(F("light_sensor_gain ??? - set the gain of the light sensor"));
      Serial.println(F("light_sensor_gain? - get the gain of the light sensor"));
      Serial.println(F("light_sensor_gain_values? - get the allowable gain values of the light sensor"));
      Serial.println(F("light_sensor_integration_time? - get the integration time in milliseconds"));
      Serial.println(F("light_sensor_intensity? - read the current value from the full spectrum diode"));
      #endif //LIGHT_SENSOR
      Serial.println("");
      Serial.println(F("??? means a decimal integer."));
    }
    Serial.println(F("Type 'help' for a list of commands."));
  }else{
    delay(1);
    return;
  }
}
