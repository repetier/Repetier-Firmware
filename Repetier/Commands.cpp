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

#ifdef SDSUPPORT
#include "SdFat.h"
#endif


/** \brief Waits until movement cache is empty.

  Some commands expect no movement, before they can execute. This function
  waits, until the steppers are stopped. In the meanwhile it buffers incoming
  commands and manages temperatures.
*/
void wait_until_end_of_move() {
  while(lines_count) {
    gcode_read_serial();
    check_periodical(); 
  }
}
void print_temperatures() {
#if HEATED_BED_SENSOR_TYPE==0 
  out.print_int_P(PSTR("T:"),extruder_get_temperature()>>CELSIUS_EXTRA_BITS); 
#else
  out.print_int_P(PSTR("T:"),extruder_get_temperature()>>CELSIUS_EXTRA_BITS); 
  out.print_int_P(PSTR(" B:"),heated_bed_get_temperature()>>CELSIUS_EXTRA_BITS); 
#endif
#ifdef TEMP_PID
  out.print_int_P(PSTR(" @:"),(int)current_extruder_out);
#endif
  out.println();
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
          queue_move(ALWAYS_CHECK_ENDSTOPS);
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
        wait_until_end_of_move();
        float saved_feedrate = printer_state.feedrate;
        for(byte i=0; i < 4; i++) {
          printer_state.destinationSteps[i] = printer_state.currentPositionSteps[i];
        }

        byte home_all_axis = !(GCODE_HAS_X(com) || GCODE_HAS_Y(com) || GCODE_HAS_Z(com));

        if(home_all_axis || GCODE_HAS_X(com)) {
          if ((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1)){
            printer_state.destinationSteps[0] = 1.5 * printer_state.xMaxSteps * X_HOME_DIR;
            printer_state.currentPositionSteps[0] = -printer_state.destinationSteps[0];
            printer_state.feedrate = homing_feedrate[0];
            queue_move(true);
            wait_until_end_of_move();
            printer_state.currentPositionSteps[0] = 0;
            printer_state.destinationSteps[0] = axis_steps_per_unit[0]*-5 * X_HOME_DIR;
            queue_move(true);
            wait_until_end_of_move();
            printer_state.destinationSteps[0] = axis_steps_per_unit[0]*10 * X_HOME_DIR;
            queue_move(true);    
            wait_until_end_of_move();
            printer_state.currentPositionSteps[0] = (X_HOME_DIR == -1) ? 0 : printer_state.xMaxSteps;
            printer_state.destinationSteps[0] = printer_state.currentPositionSteps[0];
          }
        }
        
        if(home_all_axis || GCODE_HAS_Y(com)) {
          if ((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1)){
            printer_state.currentPositionSteps[1] = 0;
            printer_state.destinationSteps[1] = 1.5 * printer_state.yMaxSteps * Y_HOME_DIR;
            printer_state.currentPositionSteps[1] = -printer_state.destinationSteps[1];
            printer_state.feedrate = homing_feedrate[1];
            queue_move(true);         
            wait_until_end_of_move();
            printer_state.currentPositionSteps[1] = 0;
            printer_state.destinationSteps[1] = axis_steps_per_unit[1]*-5 * Y_HOME_DIR;
            queue_move(true);          
            wait_until_end_of_move();
            printer_state.destinationSteps[1] = axis_steps_per_unit[1]*10 * Y_HOME_DIR;
            queue_move(true);
            wait_until_end_of_move();
            printer_state.currentPositionSteps[1] = (Y_HOME_DIR == -1) ? 0 : printer_state.yMaxSteps;
            printer_state.destinationSteps[1] = printer_state.currentPositionSteps[1];
          }
        }
        
        if(home_all_axis || GCODE_HAS_Z(com)) {
          if ((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1)){
            printer_state.currentPositionSteps[2] = 0;
            printer_state.destinationSteps[2] = 1.5 * printer_state.zMaxSteps * Z_HOME_DIR;
            printer_state.currentPositionSteps[2] = -printer_state.destinationSteps[2];
            printer_state.feedrate = homing_feedrate[2];
            queue_move(true);
            wait_until_end_of_move();          
            printer_state.currentPositionSteps[2] = 0;
            printer_state.destinationSteps[2] = axis_steps_per_unit[2]*-2 * Z_HOME_DIR;
            queue_move(true);
            wait_until_end_of_move();          
            printer_state.destinationSteps[2] = axis_steps_per_unit[2]*10 * Z_HOME_DIR;
            queue_move(true);
            wait_until_end_of_move();          
            printer_state.currentPositionSteps[2] = (Z_HOME_DIR == -1) ? 0 : printer_state.zMaxSteps;
            printer_state.destinationSteps[2] = printer_state.currentPositionSteps[2];
          }
        }
        printer_state.feedrate = saved_feedrate;
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
      case 104: // M104
        if(DEBUG_DRYRUN) break;
#ifdef EXACT_TEMPERATURE_TIMING
        wait_until_end_of_move();
#endif
        if (GCODE_HAS_S(com)) extruder_set_temperature(com->S<<CELSIUS_EXTRA_BITS);
        break;
      case 140: // M140 set bed temp
        if(DEBUG_DRYRUN) break;
        if (GCODE_HAS_S(com)) heated_bed_set_temperature(com->S<<CELSIUS_EXTRA_BITS);
        break;
      case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
        print_temperatures();
        break;
      case 109: // M109 - Wait for extruder heater to reach target.
        {
          if(DEBUG_DRYRUN) break;
          wait_until_end_of_move();
          if (GCODE_HAS_S(com)) extruder_set_temperature(com->S<<CELSIUS_EXTRA_BITS);
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
        break;
      case 190: // M190 - Wait bed for heater to reach target.
        if(DEBUG_DRYRUN) break;
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
        break;
      case 106: //M106 Fan On
        //wait_until_end_of_move(); // uncomment this to change the speed exactly at that point, but it may cause blobs if you do!
#ifdef SIMULATE_FAN_PWM
        if (GCODE_HAS_S(com))
            fan_speed = constrain(com->S,0,255)<<4;
        else
            fan_speed = 4080;
#else
        if (GCODE_HAS_S(com) && com->S<255){
            digitalWrite(FAN_PIN, HIGH);
            analogWrite(FAN_PIN, constrain(com->S,0,255) );
        }
        else
            digitalWrite(FAN_PIN, HIGH);
#endif
        break;
      case 107: //M107 Fan Off
        //wait_until_end_of_move(); // uncomment this to change the speed exactly at that point, but it may cause blobs if you do!
#ifdef SIMULATE_FAN_PWM
        fan_speed=0;
#else
        analogWrite(FAN_PIN, 0);        
        digitalWrite(FAN_PIN, LOW);
#endif
        break;
      case 80: // M80 - ATX Power On
        wait_until_end_of_move();
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
          extruder_set_temperature(0);
#if HEATED_BED_TYPE!=0
          target_bed_raw = 0;
#endif
        }   
        break;
      case 115: // M115
        out.println_P(PSTR("FIRMWARE_NAME:Repetier FIRMWARE_URL:https://github.com/repetier/Repetier-Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 REPETIER_PROTOCOL:1"));
        break;
      case 114: // M114
	out.print_float_P(PSTR("X:"),printer_state.currentPositionSteps[0]*inv_axis_steps_per_unit[0]*(unit_inches?0.03937:1));
	out.print_float_P(PSTR(" Y:"),printer_state.currentPositionSteps[1]*inv_axis_steps_per_unit[1]*(unit_inches?0.03937:1));
	out.print_float_P(PSTR(" Z:"),printer_state.currentPositionSteps[2]*inv_axis_steps_per_unit[2]*(unit_inches?0.03937:1));
	out.println_float_P(PSTR(" E:"),printer_state.currentPositionSteps[3]*inv_axis_steps_per_unit[3]*(unit_inches?0.03937:1));
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
      case 222: //M222 F_CPU / S
       if(GCODE_HAS_S(com))
         out.println_long_P(PSTR("F_CPU/x="),CPUDivU2(com->S));
       break;
#ifdef USE_ADVANCE
      case 232:
       out.print_int_P(PSTR("Max advance="),maxadv);
       if(maxadv>0) 
         out.println_float_P(PSTR(", speed="),maxadvspeed); 
       else
         out.println();
       maxadv=0;
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
          out.print_float_P(PSTR(", backslash = "),printer_state.opsRetractDistance);
          if(printer_state.opsMode==2)
            out.print_float_P(PSTR(", move after = "),printer_state.opsMoveAfter);
          out.println();
        }
#ifdef DEBUG_OPS
       out.println_int_P(PSTR("Timer diff"),printer_state.timer0Interval);
       out.println_int_P(PSTR("Ret. steps:"),printer_state.opsRetractSteps);
       out.println_int_P(PSTR("PushBack Steps:"),printer_state.opsPushbackSteps);
       out.println_int_P(PSTR("Move after steps:"),printer_state.opsMoveAfterSteps);
#endif
        break;
#endif
#ifdef USE_ADVANCE
      case 233:
        if(GCODE_HAS_X(com)) {
          current_extruder->advanceK = com->X;
        }
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

