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

// The array below has 3 stepper objects, for X,Y,Z respectively
const int n_motors = 3;
long min_step_delay = 1000;
Stepper* motors[n_motors];
signed long current_pos[n_motors];
int steps_remaining[n_motors];

// We'll use this input buffer for serial comms
const int INPUT_BUFFER_LENGTH = 64;
char input_buffer[INPUT_BUFFER_LENGTH];

void setup() {
  // initialise serial port
  Serial.begin(115200);
  Serial.println("OpenFlexure Motor Board v0.3");
  
  // get the stepoper objects from the motor shield objects
  motors[0] = new Stepper(8, 13, 12, 11, 10);
  motors[1] = new Stepper(8, 9, 8, 7, 6);
  motors[2] = new Stepper(8, 5, 4, 3, 2);
  
  for(int i=0; i<n_motors; i++){
    motors[i]->setSpeed(1*8.0/4096.0); //using Fergus's speed for now, though this is ignored...
    steps_remaining[i]=0;
    current_pos[i]=0;
  }
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

void loop() {
  // wait for a serial command and read it
  int received_bytes = Serial.readBytesUntil('\n',input_buffer,INPUT_BUFFER_LENGTH-1);
  if(received_bytes > 0){
    input_buffer[received_bytes] = '\0';
    String command = String(input_buffer);
//    Serial.println("Got command: <"+command+">");

    bool single_axis_move = false;
    int motor = -1;
    if(command.startsWith("mrx ")){
      single_axis_move = true;
      motor = 0;
    }
    if(command.startsWith("mry ")){
      single_axis_move = true;
      motor = 1;
    }
    if(command.startsWith("mrz ")){
      single_axis_move = true;
      motor = 2;
    }
    if(single_axis_move){
      int preceding_space = command.indexOf(' ',0);
      if(preceding_space<0){
        Serial.println("Error: command is mr(x|y|z) <int>");
        return;
      }
      int n_steps = command.substring(preceding_space+1).toInt();
      int dx = n_steps > 0 ? 1 : -1;
      n_steps *= dx; //make this number positive
      for(int i=0; i<n_steps; i++){
        stepMotor(motor, dx);
        delayMicroseconds(min_step_delay);
      }
      Serial.println("done");
    }
    if(command.startsWith("move_rel ") or command.startsWith("mr ")){ //relative move
      int preceding_space = -1;
      for(int i=0; i<n_motors; i++){ //read three integers and store in steps_remaining
        preceding_space = command.indexOf(' ',preceding_space+1);
        if(preceding_space<0){
          Serial.println("Error: command is mr <int> <int> <int>");
          break;
        }
        steps_remaining[i] = command.substring(preceding_space+1).toInt();
      }
    }

    if(command.startsWith("release")){ //release steppers (de-energise coils)
      for(int i=0; i<n_motors; i++) releaseMotor(i);
    }
    if(command.startsWith("p?") or command.startsWith("position?")){
      for(int i=0; i<n_motors; i++){
        if(i>0) Serial.print(" ");
        Serial.print(current_pos[i]);
      }
      Serial.println();
    }
///      
  }else{
    delay(5);
    return;
  }
}
