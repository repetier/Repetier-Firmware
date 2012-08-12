/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Configuration.h"
#include "Reptier.h"
#include "Eeprom.h"
#include "pins_arduino.h"
#include "ui.h"

#ifdef SDSUPPORT
#include "SdFat.h"
#endif

const int sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42

void check_periodical() {
  if(!execute_periodical) return;
  execute_periodical=0;
  manage_temperatures();
  if(--counter_250ms==0) {
     if(manage_monitor<=1+NUM_EXTRUDER)
        write_monitor();
     counter_250ms=10;
  }
  UI_SLOW; 
}

/** \brief Waits until movement cache is empty.

  Some commands expect no movement, before they can execute. This function
  waits, until the steppers are stopped. In the meanwhile it buffers incoming
  commands and manages temperatures.
*/
void wait_until_end_of_move() {
  while(lines_count) {
    gcode_read_serial();
    check_periodical(); 
    UI_MEDIUM;
  }
}
void printPosition() {
  out.print_float_P(PSTR("X:"),printer_state.currentPositionSteps[0]*inv_axis_steps_per_unit[0]*(unit_inches?0.03937:1));
  out.print_float_P(PSTR(" Y:"),printer_state.currentPositionSteps[1]*inv_axis_steps_per_unit[1]*(unit_inches?0.03937:1));
  out.print_float_P(PSTR(" Z:"),printer_state.currentPositionSteps[2]*inv_axis_steps_per_unit[2]*(unit_inches?0.03937:1));
  out.println_float_P(PSTR(" E:"),printer_state.currentPositionSteps[3]*inv_axis_steps_per_unit[3]*(unit_inches?0.03937:1));
}
void print_temperatures() {
#if HEATED_BED_SENSOR_TYPE==0 
  out.print_int_P(PSTR("T:"),((1<<(CELSIUS_EXTRA_BITS-1))+extruder_get_temperature())>>CELSIUS_EXTRA_BITS); 
#else
  out.print_int_P(PSTR("T:"),((1<<(CELSIUS_EXTRA_BITS-1))+extruder_get_temperature())>>CELSIUS_EXTRA_BITS); 
  out.print_int_P(PSTR(" B:"),((1<<(CELSIUS_EXTRA_BITS-1))+heated_bed_get_temperature())>>CELSIUS_EXTRA_BITS); 
#endif
#ifdef TEMP_PID
  out.print_int_P(PSTR(" @:"),(int)pwm_pos[current_extruder->id]);
#endif
  out.println();
}
void change_feedrate_multiply(int factor) {
  if(factor<25) factor=25;
  if(factor>500) factor=500;
  printer_state.feedrate *= (float)factor/(float)printer_state.feedrateMultiply;
  printer_state.feedrateMultiply = factor;
  out.println_int_P(PSTR("SpeedMultiply:"),factor);
}
void set_fan_speed(int speed,bool wait) {  
#if FAN_PIN>=0
  speed = constrain(speed,0,255);
  if(wait)
    wait_until_end_of_move(); // use only if neededthis to change the speed exactly at that point, but it may cause blobs if you do!
  pwm_pos[3] = speed;
#endif
}
void home_axis(bool xaxis,bool yaxis,bool zaxis) {
  long steps;
  if(xaxis) {
    if ((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1)){
      UI_STATUS_UPD(UI_TEXT_HOME_X);
      steps = printer_state.xMaxSteps * X_HOME_DIR;         
      printer_state.currentPositionSteps[0] = -steps;
      move_steps(2*steps,0,0,0,homing_feedrate[0],true,true);
      printer_state.currentPositionSteps[0] = 0;
      move_steps(axis_steps_per_unit[0]*-ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homing_feedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
      move_steps(axis_steps_per_unit[0]*2*ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homing_feedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
      if(ENDSTOP_X_BACK_ON_HOME > 0)
        move_steps(axis_steps_per_unit[0]*-ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,0,0,0,homing_feedrate[0],true,false);
#endif
      printer_state.currentPositionSteps[0] = (X_HOME_DIR == -1) ? 0 : printer_state.xMaxSteps;
    }
  }        
  if(yaxis) {
    if ((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1)){
      UI_STATUS_UPD(UI_TEXT_HOME_Y);
      steps = printer_state.yMaxSteps * Y_HOME_DIR;         
      printer_state.currentPositionSteps[1] = -steps;
      move_steps(0,2*steps,0,0,homing_feedrate[1],true,true);
      printer_state.currentPositionSteps[1] = 0;
      move_steps(0,axis_steps_per_unit[1]*-ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homing_feedrate[1]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
      move_steps(0,axis_steps_per_unit[1]*2*ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homing_feedrate[1]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Y_BACK_ON_HOME)
      if(ENDSTOP_Y_BACK_ON_HOME > 0)
        move_steps(0,axis_steps_per_unit[1]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homing_feedrate[1],true,false);
#endif
      printer_state.currentPositionSteps[1] = (Y_HOME_DIR == -1) ? 0 : printer_state.yMaxSteps;
    }
  }        
  if(zaxis) {
    if ((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1)){
      UI_STATUS_UPD(UI_TEXT_HOME_Z);
      steps = printer_state.zMaxSteps * Z_HOME_DIR;         
      printer_state.currentPositionSteps[2] = -steps;
      move_steps(0,0,2*steps,0,homing_feedrate[2],true,true);
      printer_state.currentPositionSteps[2] = 0;
      move_steps(0,0,axis_steps_per_unit[2]*-ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,false);
      move_steps(0,0,axis_steps_per_unit[2]*2*ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Z_BACK_ON_HOME)
      if(ENDSTOP_Z_BACK_ON_HOME > 0)
        move_steps(0,0,axis_steps_per_unit[2]*-ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR,0,homing_feedrate[2],true,false);
#endif
      printer_state.currentPositionSteps[2] = (Z_HOME_DIR == -1) ? 0 : printer_state.zMaxSteps;
    }
  }
  UI_CLEAR_STATUS  
}
/**
  \brief Execute the command stored in com.
*/
void process_command(GCode *com)
{
  unsigned long codenum; //throw away variable
#ifdef INCLUDE_DEBUG_COMMUNICATION
  if(DEBUG_COMMUNICATION) {
    if(GCODE_HAS_G(com) || (GCODE_HAS_M(com) && com->M!=111)) {
      gcode_command_finished(); // free command cache
      previous_millis_cmd = millis();
      return;      
    }
  }
#endif
  if(GCODE_HAS_G(com))
  {
    switch(com->G)
    {
      case 0: // G0 -> G1
      case 1: // G1
        if(get_coordinates(com)) // For X Y Z E F
          queue_move(ALWAYS_CHECK_ENDSTOPS,true);
        break;
      case 4: // G4 dwell
        wait_until_end_of_move();
        codenum = 0;
        if(GCODE_HAS_P(com)) codenum = com->P; // milliseconds to wait
        if(GCODE_HAS_S(com)) codenum = (long)com->S * 1000; // seconds to wait
        codenum += millis();  // keep track of when we started waiting
        while((unsigned long)(codenum-millis())  < 2000000000 ){
          gcode_read_serial();
          check_periodical();
        }
        break;
      case 20: // Units to inches
        unit_inches = 1;
        break;
      case 21: // Units to mm
        unit_inches = 0;
        break;
      case 28: {//G28 Home all Axis one at a time
          byte home_all_axis = (GCODE_HAS_NO_XYZ(com));
          home_axis(home_all_axis || GCODE_HAS_X(com),home_all_axis || GCODE_HAS_Y(com),home_all_axis || GCODE_HAS_Z(com));
        }
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        if(GCODE_HAS_X(com)) printer_state.currentPositionSteps[0] = com->X*axis_steps_per_unit[0]*(unit_inches?25.4:1.0)-printer_state.offsetX;
        if(GCODE_HAS_Y(com)) printer_state.currentPositionSteps[1] = com->Y*axis_steps_per_unit[1]*(unit_inches?25.4:1.0)-printer_state.offsetY;
        if(GCODE_HAS_Z(com)) printer_state.currentPositionSteps[2] = com->Z*axis_steps_per_unit[2]*(unit_inches?25.4:1.0);
        if(GCODE_HAS_E(com)) {
          printer_state.currentPositionSteps[3] = com->E*axis_steps_per_unit[3]*(unit_inches?25.4:1.0);
        }
        break;
        
    }
    previous_millis_cmd = millis();      
  }

  else if(GCODE_HAS_M(com))  { // Process M Code
    
    switch( com->M ) {
#ifdef SDSUPPORT
        
      case 20: // M20 - list SD card
        out.println_P(PSTR("Begin file list"));
#ifdef SD_EXTENDED_DIR
        root.ls(LS_SIZE);
#else
        root.ls();
#endif
        out.println_P(PSTR("End file list"));
        break;
      case 21: // M21 - init SD card
        sdmode = false;
        initsd();
        break;
      case 22: //M22 - release SD card
        sdmode = false;
        sdactive = false;
        break;
      case 23: //M23 - Select file
        if(sdactive){
            sdmode = false;
            file.close();
            if (file.open(&root, com->text, O_READ)) {
                out.print_P(PSTR("File opened:"));
                out.print(com->text);
                out.print_P(PSTR(" Size:"));
                out.println(file.fileSize());
                sdpos = 0;
                filesize = file.fileSize();
                out.println_P(PSTR("File selected"));
            }
            else{
                out.println_P(PSTR("file.open failed"));
            }
        }
        break;
      case 24: //M24 - Start SD print
        if(sdactive){
            sdmode = true;
        }
        break;
      case 25: //M25 - Pause SD print
        if(sdmode){
            sdmode = false;
        }
        break;
      case 26: //M26 - Set SD index
        if(sdactive && GCODE_HAS_S(com)){
            sdpos = com->S;
            file.seekSet(sdpos);
        }
        break;
      case 27: //M27 - Get SD status
        if(sdactive){
            out.print_P(PSTR("SD printing byte "));
            out.print(sdpos);
            out.print("/");
            out.println(filesize);
        }else{
            out.println_P(PSTR("Not SD printing"));
        }
        break;
      case 28: //M28 - Start SD write
        if(sdactive){
            file.close();
            sdmode = false;
            if (!file.open(&root,com->text, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))  {
              out.print_P(PSTR("open failed, File: "));
              out.print(com->text);
              out.print_P(PSTR("."));
            } else {
              UI_STATUS(UI_TEXT_UPLOADING);
              savetosd = true;
              out.print_P(PSTR("Writing to file: "));
              out.println(com->text);
            }
        }
        break;
      case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //savetosd = false;
        break;
      case 30: // M30 filename - Delete file
        if(sdactive){
            sdmode = false;
            file.close();
            if(SdFile::remove(&root, com->text)) {
              out.println_P(PSTR("File deleted"));
            } else {
              out.println_P(PSTR("Deletion failed"));
            }
        }
        break;
#endif
      case 42: //M42 -Change pin status via gcode
        if (GCODE_HAS_S(com) && GCODE_HAS_P(com) && com->S>=0 && com->S<=255) {
          int pin_number = com->P;
          for(byte i = 0; i < (byte)sizeof(sensitive_pins); i++) {
              if (pgm_read_byte(&sensitive_pins[i]) == pin_number)
              {
                pin_number = -1;
                break;
              }
          }
          if (pin_number > -1) {              
            pinMode(pin_number, OUTPUT);
            digitalWrite(pin_number, com->S);
            analogWrite(pin_number, com->S);
            out.print_int_P(PSTR("Set output "),pin_number);
            out.println_int_P(PSTR(" to "),(int)com->S);
          }
        }
        break;
      case 104: // M104
        previous_millis_cmd = millis();
        if(DEBUG_DRYRUN) break;
#ifdef EXACT_TEMPERATURE_TIMING
        wait_until_end_of_move();
#else
        if(GCODE_HAS_P(com))
          wait_until_end_of_move();
#endif
        if (GCODE_HAS_S(com)) extruder_set_temperature(com->S<<CELSIUS_EXTRA_BITS,current_extruder->id);
        break;
      case 140: // M140 set bed temp
        previous_millis_cmd = millis();
        if(DEBUG_DRYRUN) break;
        if (GCODE_HAS_S(com)) heated_bed_set_temperature(com->S<<CELSIUS_EXTRA_BITS);
        break;
      case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
        print_temperatures();
        break;
      case 109: // M109 - Wait for extruder heater to reach target.
        {
          previous_millis_cmd = millis();
          if(DEBUG_DRYRUN) break;
          UI_STATUS_UPD(UI_TEXT_HEATING_EXTRUDER);
          wait_until_end_of_move();
          if (GCODE_HAS_S(com)) extruder_set_temperature(com->S<<CELSIUS_EXTRA_BITS,current_extruder->id);
          if(abs(current_extruder->currentTemperatureC - current_extruder->targetTemperatureC)<(3<<CELSIUS_EXTRA_BITS)) break; // Already in range
          bool dir = current_extruder->currentTemperatureC < current_extruder->targetTemperatureC;
          codenum = millis(); 
          unsigned long waituntil = 0;
          unsigned long cur_time;
          do {
            cur_time = millis();
            if( (cur_time - codenum) > 1000 ) { //Print Temp Reading every 1 second while heating up.
              print_temperatures();
              codenum = cur_time; 
            }
            check_periodical();
            gcode_read_serial();
            if((waituntil==0 && (dir ? current_extruder->currentTemperatureC >= current_extruder->targetTemperatureC:current_extruder->currentTemperatureC <= current_extruder->targetTemperatureC))
#ifdef TEMP_HYSTERESIS
            || (waituntil!=0 && (abs(current_extruder->currentTemperatureC - current_extruder->targetTemperatureC)>>CELSIUS_EXTRA_BITS)>TEMP_HYSTERESIS)            
#endif
            ) {
              waituntil = cur_time+1000UL*(unsigned long)current_extruder->watchPeriod; // now wait for temp. to stabalize
            }
          } while(waituntil==0 || (waituntil!=0 && (unsigned long)(waituntil-cur_time)<2000000000UL));
        }
        UI_CLEAR_STATUS;
        previous_millis_cmd = millis();
        break;
      case 190: // M190 - Wait bed for heater to reach target.
        if(DEBUG_DRYRUN) break;
        UI_STATUS_UPD(UI_TEXT_HEATING_BED);
        wait_until_end_of_move();
#if HEATED_BED_SENSOR_TYPE!=0
        if (GCODE_HAS_S(com)) heated_bed_set_temperature(com->S<<CELSIUS_EXTRA_BITS);
        codenum = millis(); 
        while(current_bed_raw < target_bed_raw) {
          if( (millis()-codenum) > 1000 ) { //Print Temp Reading every 1 second while heating up.
            print_temperatures();
            codenum = millis(); 
          }
          check_periodical();
        }
#endif
        UI_CLEAR_STATUS;
        previous_millis_cmd = millis();
        break;
      case 106: //M106 Fan On
        set_fan_speed(GCODE_HAS_S(com)?com->S:255,GCODE_HAS_P(com));
        break;
      case 107: //M107 Fan Off
        set_fan_speed(0,GCODE_HAS_P(com));
        break;
      case 80: // M80 - ATX Power On
        wait_until_end_of_move();
        previous_millis_cmd = millis();
        if(PS_ON_PIN > -1) {
          pinMode(PS_ON_PIN,OUTPUT); //GND
          digitalWrite(PS_ON_PIN, LOW);
        }
        break;
      case 81: // M81 - ATX Power Off
        wait_until_end_of_move();
        if(PS_ON_PIN > -1) {
          pinMode(PS_ON_PIN,OUTPUT); //GND
          digitalWrite(PS_ON_PIN, HIGH);
        }
        break;
      case 82:
        relative_mode_e = false;
        break;
      case 83:
        relative_mode_e = true;
        break;
      case 84:
        if(GCODE_HAS_S(com)){ stepper_inactive_time = com->S * 1000; }
        else{             
          wait_until_end_of_move();
          kill(true);
        }
        break;
      case 85: // M85
        if(GCODE_HAS_S(com))
          max_inactive_time = (long)com->S * 1000;
        else
          max_inactive_time = 0; 
        break;
      case 92: // M92
        if(GCODE_HAS_X(com)) axis_steps_per_unit[0] = com->X;
        if(GCODE_HAS_Y(com)) axis_steps_per_unit[1] = com->Y;
        if(GCODE_HAS_Z(com)) axis_steps_per_unit[2] = com->Z;
        if(GCODE_HAS_E(com)) current_extruder->stepsPerMM = com->E;
        update_ramps_parameter();        
        break;
      case 111:
        if(GCODE_HAS_S(com)) debug_level = com->S;
        if(DEBUG_DRYRUN) { // simulate movements without printing
          extruder_set_temperature(0,0);
#if NUM_EXTRUDER>1
          extruder_set_temperature(0,1);
#endif
#if HEATED_BED_TYPE!=0
          target_bed_raw = 0;
#endif
        }   
        break;
      case 115: // M115
        out.println_P(PSTR("FIRMWARE_NAME:Repetier_" REPETIER_VERSION " FIRMWARE_URL:https://github.com/repetier/Repetier-Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 REPETIER_PROTOCOL:1"));
        break;
      case 114: // M114
        printPosition();
        break;
      case 117: // M117 message to lcd
        if(GCODE_HAS_STRING(com)) {
          UI_STATUS_UPD_RAM(com->text); 
        }
        break;
      case 119: // M119
        wait_until_end_of_move();
      	#if (X_MIN_PIN > -1)
      	out.print_P(PSTR("x_min:"));
        out.print_P((digitalRead(X_MIN_PIN)^ENDSTOP_X_MIN_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (X_MAX_PIN > -1)
      	out.print_P(PSTR("x_max:"));
        out.print_P((digitalRead(X_MAX_PIN)^ENDSTOP_X_MAX_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Y_MIN_PIN > -1)
      	out.print_P(PSTR("y_min:"));
        out.print_P((digitalRead(Y_MIN_PIN)^ENDSTOP_Y_MIN_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Y_MAX_PIN > -1)
      	out.print_P(PSTR("y_max:"));
        out.print_P((digitalRead(Y_MAX_PIN)^ENDSTOP_Y_MAX_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Z_MIN_PIN > -1)
      	out.print_P(PSTR("z_min:"));
        out.print_P((digitalRead(Z_MIN_PIN)^ENDSTOP_Z_MIN_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Z_MAX_PIN > -1)
      	out.print_P(PSTR("z_max:"));
        out.print_P((digitalRead(Z_MAX_PIN)^ENDSTOP_Z_MAX_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
        out.println();
      	break;
      /*case 120: // Test beeper function
        if(GCODE_HAS_S(com) && GCODE_HAS_P(com))
          beep(com->S,com->P); // Beep test
        break;*/
      #ifdef RAMP_ACCELERATION
      case 201: // M201
        if(GCODE_HAS_X(com)) axis_steps_per_sqr_second[0] = com->X * axis_steps_per_unit[0];
        if(GCODE_HAS_Y(com)) axis_steps_per_sqr_second[1] = com->Y * axis_steps_per_unit[1];
        if(GCODE_HAS_Z(com)) axis_steps_per_sqr_second[2] = com->Z * axis_steps_per_unit[2];
        if(GCODE_HAS_E(com)) axis_steps_per_sqr_second[3] = com->E * axis_steps_per_unit[3];
        break;
      case 202: // M202
        if(GCODE_HAS_X(com)) axis_travel_steps_per_sqr_second[0] = com->X * axis_steps_per_unit[0];
        if(GCODE_HAS_Y(com)) axis_travel_steps_per_sqr_second[1] = com->Y * axis_steps_per_unit[1];
        if(GCODE_HAS_Z(com)) axis_travel_steps_per_sqr_second[2] = com->Z * axis_steps_per_unit[2];
        if(GCODE_HAS_E(com)) axis_travel_steps_per_sqr_second[3] = com->E * axis_steps_per_unit[3];
        break;
      #endif
      case 203: // M203 Temperature monitor
        if(GCODE_HAS_S(com)) manage_monitor = com->S;
        if(manage_monitor==100) manage_monitor=NUM_EXTRUDER; // Set 100 to heated bed
        break;
      case 205: // M205 Show EEPROM settings
        epr_output_settings();
        break;
      case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
#if EEPROM_MODE!=0
          epr_update(com);
#else
          out.println_P(PSTR("Error: No EEPROM support compiled."));
#endif
        break;
      case 207: // M207 X<XY jerk> Z<Z Jerk>
        if(GCODE_HAS_X(com))
          printer_state.maxJerk = com->X;
        if(GCODE_HAS_Z(com))
          printer_state.maxZJerk = com->Z;
        if(GCODE_HAS_E(com)) {
          current_extruder->maxStartFeedrate = com->E;
          extruder_select(current_extruder->id);
        }
        out.print_float_P(PSTR("Jerk:"),printer_state.maxJerk);
        out.println_float_P(PSTR(" ZJerk:"),printer_state.maxZJerk);
        break;
      case 220: // M220 S<Feedrate multiplier in percent>
        if(GCODE_HAS_S(com))
          change_feedrate_multiply(com->S);
        else          
          change_feedrate_multiply(100);
        break;
      case 221: // M221 S<Extrusion flow multiplier in percent>
        if(GCODE_HAS_S(com))
          printer_state.extrudeMultiply = com->S;
        break;
      case 222: //M222 F_CPU / S
       if(GCODE_HAS_S(com))
         out.println_long_P(PSTR("F_CPU/x="),CPUDivU2(com->S));
       break;
 #ifdef USE_ADVANCE
     case 223: // Extruder interrupt test
        if(GCODE_HAS_S(com))
          printer_state.extruderStepsNeeded+=com->S;
          break;
     case 232:
       out.print_int_P(PSTR(" linear steps:"),maxadv2);
 #ifdef ENABLE_QUADRATIC_ADVANCE
       out.print_int_P(PSTR(" quadratic steps:"),maxadv);
 #endif
       out.println_float_P(PSTR(", speed="),maxadvspeed); 
#ifdef ENABLE_QUADRATIC_ADVANCE
       maxadv=0;
#endif
       maxadv2=0;
       maxadvspeed=0;
       break;
#endif
#if USE_OPS==1
      case 231: // M231 S<OPS_MODE> X<Min_Distance> Y<Retract> Z<Backslash> F<ReatrctMove>
        if(GCODE_HAS_S(com) && com->S>=0 && com->S<3)
          printer_state.opsMode = com->S;
        if(GCODE_HAS_X(com) && com->X>=0)
          printer_state.opsMinDistance = com->X;
        if(GCODE_HAS_Y(com) && com->Y>=0)
          printer_state.opsRetractDistance = com->Y;
        if(GCODE_HAS_Z(com) && com->Z>=-printer_state.opsRetractDistance)
          printer_state.opsRetractBackslash = com->Z;
        if(GCODE_HAS_F(com) && com->F>=0 && com->F<=100)
          printer_state.opsMoveAfter = com->F;
        extruder_select(current_extruder->id);
        if(printer_state.opsMode==0) {
          out.println_P(PSTR("OPS disabled"));
        } else {
          if(printer_state.opsMode==1) 
            out.print_P(PSTR("OPS classic mode:"));
          else
            out.print_P(PSTR("OPS fast mode:"));
        
          out.print_float_P(PSTR("min distance = "),printer_state.opsMinDistance);
          out.print_float_P(PSTR(", retract = "),printer_state.opsRetractDistance);
          out.print_float_P(PSTR(", backslash = "),printer_state.opsRetractBackslash);
          if(printer_state.opsMode==2)
            out.print_float_P(PSTR(", move after = "),printer_state.opsMoveAfter);
          out.println();
        }
#ifdef DEBUG_OPS
       out.println_int_P(PSTR("Ret. steps:"),printer_state.opsRetractSteps);
       out.println_int_P(PSTR("PushBack Steps:"),printer_state.opsPushbackSteps);
       out.println_int_P(PSTR("Move after steps:"),printer_state.opsMoveAfterSteps);
#endif
        break;
#endif
#ifdef USE_ADVANCE
      case 233:
        if(GCODE_HAS_Y(com)) 
          current_extruder->advanceL = com->Y;
        out.print_float_P(PSTR("linear L:"),current_extruder->advanceL);
#ifdef ENABLE_QUADRATIC_ADVANCE
        if(GCODE_HAS_X(com)) 
          current_extruder->advanceK = com->X;
        out.print_float_P(PSTR(" quadratic K:"),current_extruder->advanceK);
#endif        
        out.println();
        break;
#endif
    }
  } else if(GCODE_HAS_T(com))  { // Process T code
    wait_until_end_of_move();
    extruder_select(com->T);
  } else{
    if(DEBUG_ERRORS) {
      out.print_P(PSTR("Unknown command:"));
      gcode_print_command(com);
      out.println();
    }
  }
#ifdef ECHO_ON_EXECUTE
  if(DEBUG_ECHO) {
      out.print_P(PSTR("Echo:"));
      gcode_print_command(com);
      out.println();
  }
#endif
  gcode_command_finished(); // free command cache
}

