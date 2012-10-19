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

#include "Reptier.h"
#include "Eeprom.h"
#include "pins_arduino.h"
#include "ui.h"

#include <SPI.h>

const int sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42

void check_periodical() {
  if(!execute_periodical) return;
  execute_periodical=0;
  manage_temperatures();
  if(--counter_250ms==0) {
     if(manage_monitor<=1+NUM_EXTRUDER)
        write_monitor();
     counter_250ms=5;
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
  OUT_P_F("X:",printer_state.currentPositionSteps[0]*inv_axis_steps_per_unit[0]*(unit_inches?0.03937:1));
  OUT_P_F(" Y:",printer_state.currentPositionSteps[1]*inv_axis_steps_per_unit[1]*(unit_inches?0.03937:1));
  OUT_P_F(" Z:",printer_state.currentPositionSteps[2]*inv_axis_steps_per_unit[2]*(unit_inches?0.03937:1));
  OUT_P_F_LN(" E:",printer_state.currentPositionSteps[3]*inv_axis_steps_per_unit[3]*(unit_inches?0.03937:1));
}
void print_temperatures() {
	float temp = current_extruder->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE==0 
  OUT_P_F("T:",temp)); 
#else
  OUT_P_F("T:",temp); 
  OUT_P_F(" B:",heated_bed_get_temperature()); 
#endif
#ifdef TEMP_PID
  OUT_P_I(" @:",(autotuneIndex==255?pwm_pos[current_extruder->id]:pwm_pos[autotuneIndex])); // Show output of autotune when tuning!
#endif
  OUT_LN;
}
void change_feedrate_multiply(int factor) {
  if(factor<25) factor=25;
  if(factor>500) factor=500;
  printer_state.feedrate *= (float)factor/(float)printer_state.feedrateMultiply;
  printer_state.feedrateMultiply = factor;
  OUT_P_I_LN("SpeedMultiply:",factor);
}
void change_flowate_multiply(int factor) {
  if(factor<25) factor=25;
  if(factor>200) factor=200;
  printer_state.extrudeMultiply = factor;
  OUT_P_I_LN("FlowMultiply:",factor);
}
void set_fan_speed(int speed,bool wait) {  
#if FAN_PIN>=0
  speed = constrain(speed,0,255);
  if(wait)
    wait_until_end_of_move(); // use only if neededthis to change the speed exactly at that point, but it may cause blobs if you do!
  pwm_pos[3] = speed;
#endif
}
#if DRIVE_SYSTEM==3
void delta_move_to_top_endstops(float feedrate) {
  long up_steps = printer_state.zMaxSteps;
  for (byte i=0; i<3; i++)
    printer_state.currentPositionSteps[i] = 0;
  calculate_delta(printer_state.currentPositionSteps, printer_state.currentDeltaPositionSteps);
  move_steps(0,0,printer_state.zMaxSteps*ENDSTOP_Z_BACK_MOVE,0,feedrate, true, true);
}

void home_axis(bool xaxis,bool yaxis,bool zaxis) {
  long steps;
  bool homeallaxis = (xaxis && yaxis && zaxis) || (!xaxis && !yaxis && !zaxis);
  if (X_MAX_PIN > -1 && Y_MAX_PIN > -1 && Z_MAX_PIN > -1) {
    UI_STATUS_UPD(UI_TEXT_HOME_DELTA);
    // Homing Z axis means that you must home X and Y
    if (homeallaxis || zaxis) {
      delta_move_to_top_endstops(homing_feedrate[0]);	
      move_steps(0,0,axis_steps_per_unit[0]*-ENDSTOP_Z_BACK_MOVE,0,homing_feedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
      delta_move_to_top_endstops(homing_feedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR);	
      printer_state.currentPositionSteps[0] = 0;
      printer_state.currentPositionSteps[1] = 0;
      printer_state.currentPositionSteps[2] = printer_state.zMaxSteps;
      calculate_delta(printer_state.currentPositionSteps, printer_state.currentDeltaPositionSteps);
      printer_state.maxDeltaPositionSteps = printer_state.currentDeltaPositionSteps[0];
    } else {
      if (xaxis) printer_state.destinationSteps[0] = 0;
      if (yaxis) printer_state.destinationSteps[1] = 0;
      split_delta_move(true,false,false);
    }
    printer_state.countZSteps = 0;
    UI_CLEAR_STATUS 
  }
}
#else
void home_axis(bool xaxis,bool yaxis,bool zaxis) {
  long steps;
  if(xaxis) {
    if ((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1)){
      UI_STATUS_UPD(UI_TEXT_HOME_X);
      steps = (printer_state.xMaxSteps-printer_state.xMinSteps) * X_HOME_DIR;         
      printer_state.currentPositionSteps[0] = -steps;
      move_steps(2*steps,0,0,0,homing_feedrate[0],true,true);
      printer_state.currentPositionSteps[0] = 0;
      move_steps(axis_steps_per_unit[0]*-ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homing_feedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
      move_steps(axis_steps_per_unit[0]*2*ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homing_feedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
      if(ENDSTOP_X_BACK_ON_HOME > 0)
        move_steps(axis_steps_per_unit[0]*-ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,0,0,0,homing_feedrate[0],true,false);
#endif
      printer_state.currentPositionSteps[0] = (X_HOME_DIR == -1) ? printer_state.xMinSteps : printer_state.xMaxSteps;
    }
  }        
  if(yaxis) {
    if ((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1)){
      UI_STATUS_UPD(UI_TEXT_HOME_Y);
      steps = (printer_state.yMaxSteps-printer_state.yMinSteps) * Y_HOME_DIR;         
      printer_state.currentPositionSteps[1] = -steps;
      move_steps(0,2*steps,0,0,homing_feedrate[1],true,true);
      printer_state.currentPositionSteps[1] = 0;
      move_steps(0,axis_steps_per_unit[1]*-ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homing_feedrate[1]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
      move_steps(0,axis_steps_per_unit[1]*2*ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homing_feedrate[1]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Y_BACK_ON_HOME)
      if(ENDSTOP_Y_BACK_ON_HOME > 0)
        move_steps(0,axis_steps_per_unit[1]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homing_feedrate[1],true,false);
#endif
      printer_state.currentPositionSteps[1] = (Y_HOME_DIR == -1) ? printer_state.yMinSteps : printer_state.yMaxSteps;
    }
  }        
  if(zaxis) {
    if ((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1)){
      UI_STATUS_UPD(UI_TEXT_HOME_Z);
      steps = (printer_state.zMaxSteps-printer_state.zMinSteps) * Z_HOME_DIR;         
      printer_state.currentPositionSteps[2] = -steps;
      move_steps(0,0,2*steps,0,homing_feedrate[2],true,true);
      printer_state.currentPositionSteps[2] = 0;
      move_steps(0,0,axis_steps_per_unit[2]*-ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,false);
      move_steps(0,0,axis_steps_per_unit[2]*2*ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Z_BACK_ON_HOME)
      if(ENDSTOP_Z_BACK_ON_HOME > 0)
        move_steps(0,0,axis_steps_per_unit[2]*-ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR,0,homing_feedrate[2],true,false);
#endif
      printer_state.currentPositionSteps[2] = (Z_HOME_DIR == -1) ? printer_state.zMinSteps : printer_state.zMaxSteps;
    }
  }
  UI_CLEAR_STATUS  
}
#endif

// Digipot methods for controling current and microstepping

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
int digitalPotWrite(int address, int value) // From Arduino DigitalPotControl example
{
    digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
}

void digipot_current(uint8_t driver, int current)
{
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}
#endif

void digipot_init() //Initialize Digipot Motor Current
{
  #if DIGIPOTSS_PIN && DIGIPOTSS_PIN > -1
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
    
    SPI.begin(); 
    SET_OUTPUT(DIGIPOTSS_PIN);    
    for(int i=0;i<=4;i++) 
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i,digipot_motor_current[i]);
  #endif
}

#if defined(X_MS1_PIN) && X_MS1_PIN > -1
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2)
{
  if(ms1 > -1) switch(driver)
  {
    case 0: WRITE( X_MS1_PIN,ms1); break;
    case 1: WRITE( Y_MS1_PIN,ms1); break;
    case 2: WRITE( Z_MS1_PIN,ms1); break;
    case 3: WRITE(E0_MS1_PIN,ms1); break;
    case 4: WRITE(E1_MS1_PIN,ms1); break;
  }
  if(ms2 > -1) switch(driver)
  {
    case 0: WRITE( X_MS2_PIN,ms2); break;
    case 1: WRITE( Y_MS2_PIN,ms2); break;
    case 2: WRITE( Z_MS2_PIN,ms2); break;
    case 3: WRITE(E0_MS2_PIN,ms2); break;
    case 4: WRITE(E1_MS2_PIN,ms2); break;
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{
  switch(stepping_mode)
  {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
  }
}
void microstep_readings()
{
      out.println_P(PSTR("MS1,MS2 Pins"));
      out.print_int_P(PSTR("X: "), READ(X_MS1_PIN));
      out.println_int_P(PSTR(","),READ(X_MS2_PIN));
      out.print_int_P(PSTR("Y: "), READ(Y_MS1_PIN));
      out.println_int_P(PSTR(","),READ(Y_MS2_PIN));
      out.print_int_P(PSTR("Z: "), READ(Z_MS1_PIN));
      out.println_int_P(PSTR(","),READ(Z_MS2_PIN));
      out.print_int_P(PSTR("E0: "), READ(E0_MS1_PIN));
      out.println_int_P(PSTR(","),READ(E0_MS2_PIN));
      out.print_int_P(PSTR("E1: "), READ(E1_MS1_PIN));
      out.println_int_P(PSTR(","),READ(E1_MS2_PIN));
}
#endif

void microstep_init()
{
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
  const uint8_t microstep_modes[] = MICROSTEP_MODES;
  SET_OUTPUT(X_MS2_PIN);
  SET_OUTPUT(Y_MS2_PIN);
  SET_OUTPUT(Z_MS2_PIN);
  SET_OUTPUT(E0_MS2_PIN);
  SET_OUTPUT(E1_MS2_PIN);
  for(int i=0;i<=4;i++) microstep_mode(i,microstep_modes[i]);
#endif
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
#if DRIVE_SYSTEM == 3
          split_delta_move(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
          queue_move(ALWAYS_CHECK_ENDSTOPS,true);
#endif
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
        sd.ls();
        break;
      case 21: // M21 - init SD card
        sd.mount();
        break;
      case 22: //M22 - release SD card
        sd.unmount();
        break;
      case 23: //M23 - Select file
        if(GCODE_HAS_STRING(com))
          sd.selectFile(com->text);
        break;
      case 24: //M24 - Start SD print
        sd.startPrint();
        break;
      case 25: //M25 - Pause SD print
        sd.pausePrint();
        break;
      case 26: //M26 - Set SD index
        if(GCODE_HAS_S(com))
            sd.setIndex(com->S);
        break;
      case 27: //M27 - Get SD status
        sd.printStatus();
        break;
      case 28: //M28 - Start SD write
        if(GCODE_HAS_STRING(com))
          sd.startWrite(com->text);
        break;
      case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //savetosd = false;
        break;
      case 30: // M30 filename - Delete file
        if(GCODE_HAS_STRING(com))
          sd.deleteFile(com->text);
        break;
      case 32: // M32 directoryname
        if(GCODE_HAS_STRING(com))
          sd.makeDirectory(com->text);
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
        if (GCODE_HAS_S(com)) extruder_set_temperature(com->S,current_extruder->id);
        break;
      case 140: // M140 set bed temp
        previous_millis_cmd = millis();
        if(DEBUG_DRYRUN) break;
        if (GCODE_HAS_S(com)) heated_bed_set_temperature(com->S);
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
          Extruder *actExtruder = current_extruder;
          if(GCODE_HAS_T(com) && com->T<NUM_EXTRUDER) actExtruder = &extruder[com->T]; 
          if (GCODE_HAS_S(com)) extruder_set_temperature(com->S,actExtruder->id);
#if defined(SKIP_M109_IF_WITHIN) && SKIP_M109_IF_WITHIN>0
          if(abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC)<(SKIP_M109_IF_WITHIN)) break; // Already in range
#endif
          bool dir = actExtruder->tempControl.targetTemperature > actExtruder->tempControl.currentTemperature;
          codenum = millis(); 
          unsigned long waituntil = 0;
#if RETRACT_DURING_HEATUP
          byte retracted = 0;
#endif
          unsigned long cur_time;
          do {
            cur_time = millis();
            if( (cur_time - codenum) > 1000 ) { //Print Temp Reading every 1 second while heating up.
              print_temperatures();
              codenum = cur_time; 
            }
            check_periodical();
            gcode_read_serial();
#if RETRACT_DURING_HEATUP
            if (current_extruder->waitRetractUnits > 0 && !retracted && dir && current_extruder->tempControl.currentTemperatureC > current_extruder->waitRetractTemperature) {
                move_steps(0,0,0,-current_extruder->waitRetractUnits * axis_steps_per_unit[3],current_extruder->maxFeedrate,false,false);
        	retracted = 1;
            }
#endif
            if((waituntil==0 && (dir ? current_extruder->tempControl.currentTemperatureC >= current_extruder->tempControl.targetTemperatureC-0.5:current_extruder->tempControl.currentTemperatureC <= current_extruder->tempControl.targetTemperatureC+0.5))
#ifdef TEMP_HYSTERESIS
            || (waituntil!=0 && (abs(current_extruder->tempControl.currentTemperatureC - current_extruder->tempControl.targetTemperatureC))>TEMP_HYSTERESIS)            
#endif
            ) {
              waituntil = cur_time+1000UL*(unsigned long)current_extruder->watchPeriod; // now wait for temp. to stabalize
            }
          } while(waituntil==0 || (waituntil!=0 && (unsigned long)(waituntil-cur_time)<2000000000UL));
#if RETRACT_DURING_HEATUP
          if (retracted) {
            move_steps(0,0,0,current_extruder->waitRetractUnits * axis_steps_per_unit[3],current_extruder->maxFeedrate,false,false);
          }
#endif
        }
        UI_CLEAR_STATUS;
        previous_millis_cmd = millis();
        break;
      case 190: // M190 - Wait bed for heater to reach target.
        if(DEBUG_DRYRUN) break;
        UI_STATUS_UPD(UI_TEXT_HEATING_BED);
        wait_until_end_of_move();
#ifdef HEATED_HEATED_BED
        if (GCODE_HAS_S(com)) heated_bed_set_temperature(com->S);
#if defined(SKIP_M190_IF_WITHIN) && SKIP_M190_IF_WITHIN>0
        if(abs(heatedBedController->currentTemperatureC-heatedBed->targetTemperatureC)<SKIP_M190_IF_WITHIN) break;
#endif
        codenum = millis(); 
        while(heatedBedController->currentTemperatureC+0.5<heatedBed->targetTemperatureC) {
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
#ifdef TEMP_PID
      case 303: {
          int temp = 150;
          int cont = 0;
          if(GCODE_HAS_S(com)) temp = com->S;
          if(GCODE_HAS_P(com)) cont = com->P;
          if(cont>=NUM_TEMPERATURE_LOOPS) cont = NUM_TEMPERATURE_LOOPS;
          autotunePID(temp,cont);
        }
        break;
#endif
      case 106: //M106 Fan On
        set_fan_speed(GCODE_HAS_S(com)?com->S:255,GCODE_HAS_P(com));
        break;
      case 107: //M107 Fan Off
        set_fan_speed(0,GCODE_HAS_P(com));
        break;
      case 80: // M80 - ATX Power On
#if PS_ON_PIN>-1
        wait_until_end_of_move();
        previous_millis_cmd = millis();
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, LOW);
#endif
        break;
      case 81: // M81 - ATX Power Off
#if PS_ON_PIN>-1
        wait_until_end_of_move();
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, HIGH);
#endif
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
#if DRIVE_SYSTEM==3
        out.println_P(PSTR("FIRMWARE_NAME:Repetier_" REPETIER_VERSION " FIRMWARE_URL:https://github.com/repetier/Repetier-Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Rostock EXTRUDER_COUNT:1 REPETIER_PROTOCOL:2"));
#else
        out.println_P(PSTR("FIRMWARE_NAME:Repetier_" REPETIER_VERSION " FIRMWARE_URL:https://github.com/repetier/Repetier-Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 REPETIER_PROTOCOL:2"));
#endif
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
        out.print_P((READ(X_MIN_PIN)^ENDSTOP_X_MIN_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (X_MAX_PIN > -1)
      	out.print_P(PSTR("x_max:"));
        out.print_P((READ(X_MAX_PIN)^ENDSTOP_X_MAX_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Y_MIN_PIN > -1)
      	out.print_P(PSTR("y_min:"));
        out.print_P((READ(Y_MIN_PIN)^ENDSTOP_Y_MIN_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Y_MAX_PIN > -1)
      	out.print_P(PSTR("y_max:"));
        out.print_P((READ(Y_MAX_PIN)^ENDSTOP_Y_MAX_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Z_MIN_PIN > -1)
      	out.print_P(PSTR("z_min:"));
        out.print_P((READ(Z_MIN_PIN)^ENDSTOP_Z_MIN_INVERTING)?PSTR("H "):PSTR("L "));
      	#endif
      	#if (Z_MAX_PIN > -1)
      	out.print_P(PSTR("z_max:"));
        out.print_P((READ(Z_MAX_PIN)^ENDSTOP_Z_MAX_INVERTING)?PSTR("H "):PSTR("L "));
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
        if(GCODE_HAS_S(com)) {
          if(com->S<NUM_EXTRUDER) manage_monitor = com->S;
 #if HAVE_HEATED_BED
          else manage_monitor=NUM_EXTRUDER; // Set 100 to heated bed
 #endif
        }
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
          change_flowate_multiply(com->S);
        else
          change_flowate_multiply(100);
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
          update_extruder_flags();
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
        update_extruder_flags();
        break;
#endif
    case 908: // Control digital trimpot directly.
    {
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(GCODE_HAS_P(com) && GCODE_HAS_S(com)) 
          digitalPotWrite((uint8_t)com->P, (uint8_t)com->S);
#endif
    }
    break;
    
    case 350: // Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
      OUT_P_LN("Set Microstepping");
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(GCODE_HAS_S(com)) for(int i=0;i<=4;i++) microstep_mode(i,com->S); 
        if(GCODE_HAS_X(com)) microstep_mode(0,(uint8_t)com->X);
        if(GCODE_HAS_Y(com)) microstep_mode(1,(uint8_t)com->Y);
        if(GCODE_HAS_Z(com)) microstep_mode(2,(uint8_t)com->Z);
        if(GCODE_HAS_E(com)) microstep_mode(3,(uint8_t)com->E);
        if(GCODE_HAS_P(com)) microstep_mode(4,com->P); // Original B but is not supported here
        microstep_readings();
#endif
    }
    break;
#ifdef STEP_COUNTER
#if DRIVE_SYSTEM==3
		case 251:
			if(GCODE_HAS_S(com)) {
				if (com->S == 0) {
					printer_state.countZSteps = 0;
					out.println_P(PSTR("Measurement reset."));
				} else if (com->S == 1) {
					OUT_P_L_LN("Measure/delta (Steps) =",printer_state.countZSteps * inv_axis_steps_per_unit[2]);
					OUT_P_L_LN("Measure/delta =",printer_state.countZSteps * inv_axis_steps_per_unit[2]);
				} else if (com->S = 2) {
					if (printer_state.countZSteps < 0)
						printer_state.countZSteps = -printer_state.countZSteps;
					printer_state.zMin = 0;
					printer_state.zLength = inv_axis_steps_per_unit[2] * printer_state.countZSteps;
					printer_state.zMaxSteps = printer_state.countZSteps;
					for (byte i=0; i<3; i++) {
						printer_state.currentPositionSteps[i] = 0;
					}
					calculate_delta(printer_state.currentPositionSteps, printer_state.currentDeltaPositionSteps);
					OUT_P_LN("Measured origin set. Measurement reset.");
			#if EEPROM_MODE!=0
					epr_data_to_eeprom(false);
					OUT_P_LN("EEPROM updated");
			#endif
				}
			}
			break;
#endif
#endif
    }
  } else if(GCODE_HAS_T(com))  { // Process T code
    wait_until_end_of_move();
    extruder_select(com->T);
  } else{
    if(DEBUG_ERRORS) {
      OUT_P("Unknown command:");
      gcode_print_command(com);
      out.println();
    }
  }
#ifdef ECHO_ON_EXECUTE
  if(DEBUG_ECHO) {
      OUT_P("Echo:");
      gcode_print_command(com);
      out.println();
  }
#endif
  gcode_command_finished(); // free command cache
}

