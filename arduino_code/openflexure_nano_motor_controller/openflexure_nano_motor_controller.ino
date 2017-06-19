/*
 * OpenFlexure Microscope motor controoler
 * Adapted from James Sharkey's Arduino-powered stepper driver
 * Written by James Sharkey, Spring 2015
 * Based on code written by Richard Bowman, Autumn 2014
 * For use with the 3D printed microscope:
 * http://www.docubricks.com/projects/openflexure-microscope
 * http://dx.doi.org/10.1063/1.4941068
 * 
 * (c) Richard Bowman & James Sharkey, Released under GPL v3, 2017
 */
#include <StepperF_alt.h>   //Fergus's hacked stepper library
#include <assert.h>
#include <EEPROM.h>

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

int command_prefix(String command, char ** prefixes, int n_prefixes){
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
  Serial.println("OpenFlexure Motor Board v0.3");
  
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
          Serial.println("Error: command is mr <int> <int> <int>");
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
      if(preceding_space <= 0) Serial.println("Bad command.");
      ramp_time = command.substring(preceding_space+1).toInt();
      EEPROM.put(ramp_time_eeprom, ramp_time);
      return;
    }
    if(command.startsWith("ramp_time?")){
      Serial.print("ramp time ");
      Serial.println(ramp_time);
      return;
    }
    if(command.startsWith("min_step_delay ") || command.startsWith("dt ")){
      int preceding_space = command.indexOf(' ',0);
      if(preceding_space <= 0) Serial.println("Bad command.");
      min_step_delay = command.substring(preceding_space+1).toInt();
      EEPROM.put(min_step_delay_eeprom, min_step_delay);
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
    if(command.startsWith("help")){
      Serial.println("OpenFlexure Motor Controller firmware v0.1");
      Serial.println("Commands (terminated by a newline character):");
      Serial.println("mrx ??? - relative move in x");
      Serial.println("mrx ??? - relative move in x");
      Serial.println("mrx ??? - relative move in x");
      Serial.println("mr ??? ??? ??? - relative move in all 3 axes");
      Serial.println("release - de-energise all motors");
      Serial.println("p? - print position (3 space-separated integers");
      Serial.println("ramp_time ??? - set the time taken to accelerate/decelerate in us");
      Serial.println("min_step_delay ??? - set the minimum time between steps in us.");
      Serial.println("dt ??? - set the minimum time between steps in us.");
      Serial.println("ramp_time? - get the time taken to accelerate/decelerate in us");
      Serial.println("min_step_delay? - get the minimum time between steps in us.");
      Serial.println("zero - set the current position to zero.");
      Serial.println("??? means a decimal integer.");
    }
    Serial.println("Type help for a list of commands.");
  }else{
    delay(1);
    return;
  }
}
