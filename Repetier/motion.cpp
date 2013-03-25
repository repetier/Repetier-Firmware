/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Reptier.h"
#include "ui.h"

// ##########################################################################
// ###                         Path planner stuff                         ###
// ##########################################################################

inline unsigned long U16SquaredToU32(unsigned int val) {
  long res;
   __asm__ __volatile__ ( // 15 Ticks
   "mul %A1,%A1 \n\t"
   "movw %A0,r0 \n\t"
   "mul %B1,%B1 \n\t"
   "movw %C0,r0 \n\t"
   "mul %A1,%B1 \n\t"
   "clr %A1 \n\t"
   "add %B0,r0 \n\t"
   "adc %C0,r1 \n\t"
   "adc %D0,%A1 \n\t"
   "add %B0,r0 \n\t"
   "adc %C0,r1 \n\t"
   "adc %D0,%A1 \n\t"
   "clr r1 \n\t"
  : "=&r"(res),"=r"(val)
  : "1"(val)
   );
  return res;
}

inline void computeMaxJunctionSpeed(PrintLine *previous,PrintLine *current) {
  if(previous->flags & FLAG_WARMUP) {
    current->joinFlags |= FLAG_JOIN_START_FIXED;
    return;
  }
#ifdef USE_ADVANCE
    if(printer_state.isAdvanceActivated()) {
        if((previous->dir & 128)!=(current->dir & 128) && ((previous->dir & 48) || (current->dir & 48))) {
            previous->joinFlags |= FLAG_JOIN_END_FIXED;
            current->joinFlags |= FLAG_JOIN_START_FIXED;
            previous->maxJunctionSpeed = min(previous->endSpeed,current->startSpeed);
            previous->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
            current->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
            previous->endSpeed = current->startSpeed = previous->maxJunctionSpeed;
            return;
        }
    }
#endif // USE_ADVANCE
#if DRIVE_SYSTEM==3
  if (previous->moveID == current->moveID) { // Avoid computing junction speed for split delta lines
   if(previous->fullSpeed>current->fullSpeed)
      previous->maxJunctionSpeed = current->fullSpeed;
   else
      previous->maxJunctionSpeed = previous->fullSpeed;
   return;
  }
#endif
   // First we compute the normalized jerk for speed 1
   float dx = current->speedX-previous->speedX;
   float dy = current->speedY-previous->speedY;
   float factor=1,tmp;
#if (DRIVE_SYSTEM == 3) // No point computing Z Jerk separately for delta moves
   float dz = current->speedZ-previous->speedZ;
   float jerk = sqrt(dx*dx+dy*dy+dz*dz);
#else
   float jerk = sqrt(dx*dx+dy*dy);
#endif
   //if(DEBUG_ECHO) {OUT_P_F_LN("Jerk:",jerk);OUT_P_F_LN("FS:",p1->fullSpeed);OUT_P_F_LN("MaxJerk:",printer_state.maxJerk);}
   if(jerk>printer_state.maxJerk)
     factor = printer_state.maxJerk/jerk;
#if (DRIVE_SYSTEM!=3)
   if((previous->dir & 64) || (current->dir & 64)) {
   //  float dz = (p2->speedZ*p2->invFullSpeed-p1->speedZ*p1->invFullSpeed)*printer_state.maxJerk/printer_state.maxZJerk;
     float dz = fabs(current->speedZ-previous->speedZ);
     if(dz>printer_state.maxZJerk) {
       tmp = printer_state.maxZJerk/dz;
       if(tmp<factor) factor = tmp;
     }
   }
#endif
   float eJerk = fabs(current->speedE-previous->speedE);
   if(eJerk>current_extruder->maxStartFeedrate) {
     tmp = current_extruder->maxStartFeedrate/eJerk;
     if(tmp<factor) factor = tmp;
   }
   previous->maxJunctionSpeed = previous->fullSpeed*factor;
   if(previous->maxJunctionSpeed>current->fullSpeed) previous->maxJunctionSpeed = current->fullSpeed;
   //if(DEBUG_ECHO) OUT_P_F_LN("Factor:",factor);
   //if(DEBUG_ECHO) OUT_P_F_LN("JSPD:",p1->maxJunctionSpeed);
}

/** Update parameter used by updateTrapezoids

Computes the acceleration/decelleration steps and advanced parameter associated.
*/
void updateStepsParameter(PrintLine *p/*,byte caller*/) {
    if(p->flags & FLAG_WARMUP) return;
    if(p->joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED) return; // Already up to date, spare time
    float startFactor = p->startSpeed * p->invFullSpeed;
    float endFactor   = p->endSpeed   * p->invFullSpeed;
    p->vStart = p->vMax*startFactor; //starting speed
    p->vEnd   = p->vMax*endFactor;
    unsigned long vmax2 = U16SquaredToU32(p->vMax);
    p->accelSteps = ((vmax2-U16SquaredToU32(p->vStart))/(p->accelerationPrim<<1))+1; // Always add 1 for missing precision
    p->decelSteps = ((vmax2-U16SquaredToU32(p->vEnd))  /(p->accelerationPrim<<1))+1;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    p->advanceStart = (float)p->advanceFull*startFactor * startFactor;
    p->advanceEnd   = (float)p->advanceFull*endFactor   * endFactor;
#endif
#endif
    if(p->accelSteps+p->decelSteps>=p->stepsRemaining) { // can't reach limit speed
      unsigned int red = (p->accelSteps+p->decelSteps+2-p->stepsRemaining)>>1;
      if(red<p->accelSteps)
        p->accelSteps-=red;
      else
        p->accelSteps = 0;
      if(red<p->decelSteps) {
        p->decelSteps-=red;
      } else
        p->decelSteps = 0;
    }
    p->joinFlags|=FLAG_JOIN_STEPPARAMS_COMPUTED;
#ifdef DEBUG_QUEUE_MOVE
    if(DEBUG_ECHO) {
      out.print_int_P(PSTR("ID:"),(int)p);
      out.print_int_P(PSTR("vStart/End:"),p->vStart);
      out.println_int_P(PSTR("/"),p->vEnd);
      out.print_int_P(PSTR("accel/decel steps:"),p->accelSteps);
      out.println_int_P(PSTR("/"),p->decelSteps);
      out.print_float_P(PSTR("st./end speed:"),p->startSpeed);
      out.println_float_P(PSTR("/"),p->endSpeed);
#if USE_OPS==1
      if(!(p->dir & 128) && printer_state.opsMode==2)
        out.println_long_P(PSTR("Reverse at:"),p->opsReverseSteps);
#endif
      out.println_int_P(PSTR("flags:"),p->flags);
      out.println_int_P(PSTR("joinFlags:"),p->joinFlags);
    }
#endif
}
void testnum(float x,char c) {
if (isnan(x)) {
    out.print(c);
    OUT_P_LN("NAN");
    return;
  }

  if (isinf(x)) {
    out.print(c);
     OUT_P_LN("INF");
    return;
  }
}

/**
Compute the maximum speed from the last entered move.
The backwards planner traverses the moves from last to first looking at deceleration. The RHS of the accelerate/decelerate ramp.

p = last line inserted
last = last element until we check
*/
inline void backwardPlanner(byte p,byte last) {
  if(p==last) return;
  PrintLine *act = &lines[p],*prev;
  float lastJunctionSpeed = act->endSpeed; // Start always with safe speed
  //PREVIOUS_PLANNER_INDEX(last); // Last element is already fixed in start speed
  while(p!=last) {
    PREVIOUS_PLANNER_INDEX(p);
    prev = &lines[p];
#if USE_OPS==1
    // Retraction points are fixed points where the extruder movement stops anyway. Finish the computation for the move and exit.
    if(printer_state.opsMode && printmoveSeen) {
      if((prev->dir & 136)==136 && (act->dir & 136)!=136) {
        if((act->dir & 64)!=0 || act->distance>printer_state.opsMinDistance) { // Switch printing - travel
          act->joinFlags |= FLAG_JOIN_START_RETRACT | FLAG_JOIN_START_FIXED; // enable retract for this point
          prev->joinFlags |= FLAG_JOIN_END_FIXED;
          return;
        } else {
          act->joinFlags |= FLAG_JOIN_NO_RETRACT;
        }
      } else
      if((prev->dir & 136)!=136 && (act->dir & 136)==136) { // Switch travel - print
        prev->joinFlags |= FLAG_JOIN_END_RETRACT | FLAG_JOIN_END_FIXED; // reverse retract for this point
        if(printer_state.opsMode==2) {
          prev->opsReverseSteps = ((long)printer_state.opsPushbackSteps*(long)printer_state.maxExtruderSpeed*TIMER0_PRESCALE)/prev->fullInterval;
          long ponr = prev->stepsRemaining/(1.0+0.01*printer_state.opsMoveAfter);
          if(prev->opsReverseSteps>ponr)
            prev->opsReverseSteps = ponr;
        }
          act->joinFlags |= FLAG_JOIN_START_FIXED; // Wait only with safe speeds!
          return;
      }
    }
#endif
    // Avoid speed calc once crusing in split delta move
#if DRIVE_SYSTEM==3
    if (prev->moveID==act->moveID && lastJunctionSpeed==prev->maxJunctionSpeed) {
      act->startSpeed = prev->endSpeed = lastJunctionSpeed;
      prev->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
      act->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
    }
#endif

    // Switch move-retraction or vice versa start always with save speeds! Keeps extruder from blocking
    if(((prev->dir & 240)!=128) && ((act->dir & 240)==128)) { // switch move - extruder only move
      prev->joinFlags |= FLAG_JOIN_END_FIXED;
      act->joinFlags |= FLAG_JOIN_START_FIXED;
      return;          
    } 
    if(prev->joinFlags & FLAG_JOIN_END_FIXED) { // Nothing to update from here on, happens when path optimize disabled
      act->joinFlags |= FLAG_JOIN_START_FIXED; // Wait only with safe speeds!
      return;
    }
	
    // Avoid speed calcs if we know we can accelerate within the line
    if (act->flags & FLAG_NOMINAL)
      lastJunctionSpeed = act->fullSpeed;
    else
      // If you accelerate from end of move to start what speed to you reach?
      lastJunctionSpeed = sqrt(lastJunctionSpeed*lastJunctionSpeed+act->acceleration); // acceleration is acceleration*distance*2! What can be reached if we try?
      // If that speed is more that the maximum junction speed allowed then ...
      if(lastJunctionSpeed>=prev->maxJunctionSpeed) { // Limit is reached
      // If the previous line's end speed has not been updated to maximum speed then do it now
      if(prev->endSpeed!=prev->maxJunctionSpeed) {
        prev->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
        prev->endSpeed = prev->maxJunctionSpeed; // possibly unneeded???
      }        
      // If actual line start speed has not been updated to maximum speed then do it now
      if(act->startSpeed!=prev->maxJunctionSpeed) {
        act->startSpeed = prev->maxJunctionSpeed; // possibly unneeded???
        act->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
      }
      lastJunctionSpeed = prev->maxJunctionSpeed;     
    } else {
      // Block prev end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
      act->startSpeed = prev->endSpeed = lastJunctionSpeed;
      prev->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
      act->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
    }
    act = prev;
  } // while loop
}

inline void forwardPlanner(byte p) {
  PrintLine *act,*next;
  if(p==lines_write_pos) return;
  byte last = lines_write_pos;
  //NEXT_PLANNER_INDEX(last);
  next = &lines[p];
  float leftspeed = next->startSpeed;

  while(p!=last) { // All except last segment, which has fixed end speed
    act = next;
    NEXT_PLANNER_INDEX(p);
    next = &lines[p];
    if(act->joinFlags & FLAG_JOIN_END_FIXED) {
      leftspeed = act->endSpeed;
      continue; // Nothing to do here
    }
	// Avoid speed calc once crusing in split delta move	
	#if DRIVE_SYSTEM==3
	if (act->moveID == next->moveID && act->endSpeed == act->maxJunctionSpeed) {
		act->startSpeed = leftspeed;
		leftspeed       = act->endSpeed;
	    act->joinFlags  |= FLAG_JOIN_END_FIXED;
        next->joinFlags |= FLAG_JOIN_START_FIXED;
		continue;
	}
	#endif


    float vmax_right;
	// Avoid speed calcs if we know we can accelerate within the line.
	if (act->flags & FLAG_NOMINAL)
	  vmax_right = act->fullSpeed;
	else
      vmax_right = sqrt(leftspeed*leftspeed+act->acceleration); // acceleration is 2*acceleration*distance!, 1000 Ticks
    if(vmax_right>act->endSpeed) { // Could be higher next run?
      act->startSpeed = leftspeed;
      leftspeed       = act->endSpeed;
      if(act->endSpeed==act->maxJunctionSpeed) {// Full speed reached, don't compute again!
        act->joinFlags  |= FLAG_JOIN_END_FIXED;
        next->joinFlags |= FLAG_JOIN_START_FIXED;
      }
      act->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
    } else { // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
      act->joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
      act->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // Needs recomputation
      act->startSpeed = leftspeed;
      act->endSpeed   = next->startSpeed = leftspeed = vmax_right;
      next->joinFlags |= FLAG_JOIN_START_FIXED;
    }
  }
  next->startSpeed = leftspeed; // This is the new segment, wgich is updated anyway, no extra flag needed.
}

/**
This is the path planner.

It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
is already optimal from previous updates.
The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely become active.

The method is called before lines_count is increased!
*/
void updateTrapezoids(byte p) {
  byte first = p;
  PrintLine *firstLine;
  PrintLine *act = &lines[p];
  BEGIN_INTERRUPT_PROTECTED; 
  byte maxfirst = lines_pos; // first non fixed segment
  if(maxfirst!=p)
    NEXT_PLANNER_INDEX(maxfirst); // don't touch the line printing
  // Now ignore enough segments to gain enough time for path planning
  long timeleft = 0;
  while(timeleft<4500*MOVE_CACHE_SIZE && maxfirst!=p) {
    timeleft+=lines[maxfirst].timeInTicks;
    NEXT_PLANNER_INDEX(maxfirst);
  }
  while(first!=maxfirst && !(lines[first].joinFlags & FLAG_JOIN_END_FIXED)) {
    PREVIOUS_PLANNER_INDEX(first);
  }
  if(first!=p && (lines[first].joinFlags & FLAG_JOIN_END_FIXED)) {
    NEXT_PLANNER_INDEX(first);
  }
  // First is now the new element or the first element with non fixed end speed.
  // anyhow, the start speed of first is fixed
  firstLine = &lines[first];
  firstLine->flags |= FLAG_BLOCKED; // don't let printer touch this or following segments during update
  END_INTERRUPT_PROTECTED; 
  byte previdx = p-1;
  if(previdx>=MOVE_CACHE_SIZE) previdx = MOVE_CACHE_SIZE-1;
  if(lines_count && (lines[previdx].flags & FLAG_WARMUP)==0)
    computeMaxJunctionSpeed(&lines[previdx],act); // Set maximum junction speed if we have a real move before
  else
    act->joinFlags |= FLAG_JOIN_START_FIXED;

  backwardPlanner(p,first);  
  // Reduce speed to reachable speeds
  forwardPlanner(first);
  
  // Update precomputed data
  do {
    updateStepsParameter(&lines[first]);
    lines[first].flags &= ~FLAG_BLOCKED;  // Flying block to release next used segment as early as possible
    NEXT_PLANNER_INDEX(first);
    lines[first].flags |= FLAG_BLOCKED;
  } while(first!=lines_write_pos);
  updateStepsParameter(act);
  act->flags &= ~FLAG_BLOCKED;
}


// ##########################################################################
// ###                         Motion computations                        ###
// ##########################################################################

inline float safeSpeed(PrintLine *p) {
    float safe;
    #ifdef USE_ADVANCE
    if((p->dir & 128) && printer_state.isAdvanceActivated()) {
        safe = min(p->fullSpeed,printer_state.minimumSpeed);
    } else
    #endif
    safe = min(p->fullSpeed,max(printer_state.minimumSpeed,printer_state.maxJerk*0.5));
#if DRIVE_SYSTEM != 3
  if(p->dir & 64) {
    if(fabs(p->speedZ)>printer_state.maxZJerk*0.5) {
      float safe2 = printer_state.maxZJerk*0.5*p->fullSpeed/fabs(p->speedZ);
      if(safe2<safe) safe = safe2;
    }
  }
#endif
  if(p->dir & 128) {
    if(p->dir & 112) {
      float safe2 = 0.5*current_extruder->maxStartFeedrate*p->fullSpeed/fabs(p->speedE);
      if(safe2<safe) safe = safe2;
    } else {
      safe = 0.5*current_extruder->maxStartFeedrate; // This is a retraction move
    }
  }
  return (safe<p->fullSpeed?safe:p->fullSpeed);
}

/**
Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands.
*/
void move_steps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop) {
  float saved_feedrate = printer_state.feedrate;
  for(byte i=0; i < 4; i++) {
      printer_state.destinationSteps[i] = printer_state.currentPositionSteps[i];
  }
  printer_state.destinationSteps[0]+=x;
  printer_state.destinationSteps[1]+=y;
  printer_state.destinationSteps[2]+=z;
  printer_state.destinationSteps[3]+=e;
  printer_state.feedrate = feedrate;
#if DRIVE_SYSTEM==3
  split_delta_move(check_endstop,false,false);
#else
  queue_move(check_endstop,false);
#endif
  printer_state.feedrate = saved_feedrate;
  if(waitEnd)
    wait_until_end_of_move();
}

/** Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
processing.
*/
byte check_new_move(byte pathOptimize, byte waitExtraLines) {
  if(lines_count==0 && waitRelax==0 && pathOptimize) { // First line after some time - warmup needed
#ifdef DEBUG_OPS
    out.println_P(PSTR("New path"));
#endif
    byte w = 3;
    PrintLine *p = &lines[lines_write_pos];
    while(w) {
      p->flags = FLAG_WARMUP;
      p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
      p->dir = 0;
      p->primaryAxis = w + waitExtraLines;
      p->timeInTicks = p->accelerationPrim = p->facceleration = 10000*(unsigned int)w;
      lines_write_pos++;
      if(lines_write_pos>=MOVE_CACHE_SIZE) lines_write_pos = 0;
BEGIN_INTERRUPT_PROTECTED
      lines_count++;
END_INTERRUPT_PROTECTED
      p = &lines[lines_write_pos];
      w--;
    }
    return 1;
  }
  return 0;
}
void log_long_array(PGM_P ptr,long *arr) {
  out.print_P(ptr);
  for(byte i=0;i<4;i++) {
    out.print(' ');
    out.print(arr[i]);
  }
  out.println();
}
void log_float_array(PGM_P ptr,float *arr) {
  out.print_P(ptr);
  for(byte i=0;i<3;i++)
    out.print_float_P(PSTR(" "),arr[i]);
  out.println_float_P(PSTR(" "),arr[3]);
}
void log_printLine(PrintLine *p) {
  out.println_int_P(PSTR("ID:"),(int)p);
  log_long_array(PSTR("Delta"),p->delta);
  //log_long_array(PSTR("Error"),p->error);
  //out.println_int_P(PSTR("Prim:"),p->primaryAxis);
  out.println_int_P(PSTR("Dir:"),p->dir);
  out.println_int_P(PSTR("Flags:"),p->flags);
  out.println_float_P(PSTR("fullSpeed:"),p->fullSpeed);
  out.println_long_P(PSTR("vMax:"),p->vMax);
  out.println_float_P(PSTR("Acceleration:"),p->acceleration);
  out.println_long_P(PSTR("Acceleration Prim:"),p->accelerationPrim);
  //out.println_long_P(PSTR("Acceleration Timer:"),p->facceleration);
  out.println_long_P(PSTR("Remaining steps:"),p->stepsRemaining);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  out.println_long_P(PSTR("advanceFull:"),p->advanceFull>>16);
  out.println_long_P(PSTR("advanceRate:"),p->advanceRate);
#endif
#endif
}


void calculate_move(PrintLine *p,float axis_diff[],byte check_endstops,byte pathOptimize)
{
#if DRIVE_SYSTEM==3
  long axis_interval[5];
#else
  long axis_interval[4];
#endif
  float time_for_move = (float)(F_CPU)*p->distance / printer_state.feedrate; // time is in ticks
  bool critical=false;
  if(lines_count<MOVE_CACHE_LOW && time_for_move<LOW_TICKS_PER_MOVE) { // Limit speed to keep cache full.
    //OUT_P_I("L:",lines_count);
    time_for_move += (3*(LOW_TICKS_PER_MOVE-time_for_move))/(lines_count+1); // Increase time if queue gets empty. Add more time if queue gets smaller.
    //OUT_P_F_LN("Slow ",time_for_move);
    critical=true;
  }
  p->timeInTicks = time_for_move;
  UI_MEDIUM; // do check encoder
  // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
  long limitInterval = time_for_move/p->stepsRemaining; // until not violated by other constraints it is your target speed
  axis_interval[0] = fabs(axis_diff[0])*F_CPU/(max_feedrate[0]*p->stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
  if(axis_interval[0]>limitInterval) limitInterval = axis_interval[0];
  axis_interval[1] = fabs(axis_diff[1])*F_CPU/(max_feedrate[1]*p->stepsRemaining);
  if(axis_interval[1]>limitInterval) limitInterval = axis_interval[1];
  if(p->dir & 64) { // normally no move in z direction
    axis_interval[2] = fabs((float)axis_diff[2])*(float)F_CPU/(float)(max_feedrate[2]*p->stepsRemaining); // must prevent overflow!
    if(axis_interval[2]>limitInterval) limitInterval = axis_interval[2];
  } else axis_interval[2] = 0;
  axis_interval[3] = fabs(axis_diff[3])*F_CPU/(max_feedrate[3]*p->stepsRemaining);
  if(axis_interval[3]>limitInterval) limitInterval = axis_interval[3];
#if DRIVE_SYSTEM==3
  axis_interval[4] = fabs(axis_diff[4])*F_CPU/(max_feedrate[0]*p->stepsRemaining);
#endif

  p->fullInterval = limitInterval>200 ? limitInterval : 200; // This is our target speed
  // new time at full speed = limitInterval*p->stepsRemaining [ticks]
  time_for_move = (float)limitInterval*(float)p->stepsRemaining; // for large z-distance this overflows with long computation
  float inv_time_s = (float)F_CPU/time_for_move;
  if(p->dir & 16) {
    axis_interval[0] = time_for_move/p->delta[0];
    p->speedX = axis_diff[0]*inv_time_s;
    if(!(p->dir & 1)) p->speedX = -p->speedX;
  } else p->speedX = 0;
  if(p->dir & 32) {
    axis_interval[1] = time_for_move/p->delta[1];
    p->speedY = axis_diff[1]*inv_time_s;
    if(!(p->dir & 2)) p->speedY = -p->speedY;
  } else p->speedY = 0;
  if(p->dir & 64) {
    axis_interval[2] = time_for_move/p->delta[2];
    p->speedZ = axis_diff[2]*inv_time_s;
    if(!(p->dir & 4)) p->speedZ = -p->speedZ;
  } else p->speedZ = 0;
  if(p->dir & 128) {
    axis_interval[3] = time_for_move/p->delta[3];
    p->speedE = axis_diff[3]*inv_time_s;
    if(!(p->dir & 8)) p->speedE = -p->speedE;
  }
#if DRIVE_SYSTEM==3
  axis_interval[4] = time_for_move/p->stepsRemaining;
#endif
  p->fullSpeed = p->distance*inv_time_s;


  //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
  byte is_print_move = (p->dir & 136)==136; // are we printing
#if USE_OPS==1
  if(is_print_move) printmoveSeen = 1;
#endif
  //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
  #ifdef RAMP_ACCELERATION

    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowest_axis_plateau_time_repro = 1e20; // repro to reduce division Unit: 1/s
    for(byte i=0; i < 4 ; i++) {
	// Errors for delta move are initialized in timer
	#if DRIVE_SYSTEM!=3
      p->error[i] = p->delta[p->primaryAxis] >> 1;
	#endif
      if(p->dir & (16<<i)) {
        // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
        slowest_axis_plateau_time_repro = min(slowest_axis_plateau_time_repro,
               (float)axis_interval[i] * (float)(is_print_move ?  axis_steps_per_sqr_second[i] : axis_travel_steps_per_sqr_second[i])); //  steps/s^2 * step/tick  Ticks/s^2
      }
    }
	// Errors for delta move are initialized in timer (except extruder)
#if DRIVE_SYSTEM==3
    p->error[3] = p->stepsRemaining >> 1;
#endif
    p->invFullSpeed = 1.0/p->fullSpeed;
    p->accelerationPrim = slowest_axis_plateau_time_repro / axis_interval[p->primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    p->facceleration = 262144.0*(float)p->accelerationPrim/F_CPU; // will overflow without float!
    p->acceleration = 2.0*p->distance*slowest_axis_plateau_time_repro*p->fullSpeed/((float)F_CPU); // mm^2/s^2
    p->startSpeed = p->endSpeed = safeSpeed(p);
	// Can accelerate to full speed within the line
	if (sqrt(p->startSpeed*p->startSpeed+p->acceleration) >= p->fullSpeed)
	  p->flags |= FLAG_NOMINAL;

    p->vMax = F_CPU / p->fullInterval; // maximum steps per second, we can reach
   // if(p->vMax>46000)  // gets overflow in N computation
   //   p->vMax = 46000;
    //p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;
#ifdef USE_ADVANCE
  if((p->dir & 112)==0 || (p->dir & 128)==0 || (p->dir & 8)==0) {
#ifdef ENABLE_QUADRATIC_ADVANCE
    p->advanceRate = 0; // No head move or E move only or sucking filament back
    p->advanceFull = 0;
#endif
    p->advanceL = 0;
  } else {
    float advlin = fabs(p->speedE)*current_extruder->advanceL*0.001*axis_steps_per_unit[3];
    p->advanceL = (65536*advlin)/p->vMax; //advanceLscaled = (65536*vE*k2)/vMax
 #ifdef ENABLE_QUADRATIC_ADVANCE;
    p->advanceFull = 65536*current_extruder->advanceK*p->speedE*p->speedE; // Steps*65536 at full speed
    long steps = (U16SquaredToU32(p->vMax))/(p->accelerationPrim<<1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
    p->advanceRate = p->advanceFull/steps;
    if((p->advanceFull>>16)>maxadv) {
        maxadv = (p->advanceFull>>16);
        maxadvspeed = fabs(p->speedE);
    }
 #endif
    if(advlin>maxadv2) {
      maxadv2 = advlin;
      maxadvspeed = fabs(p->speedE);
    }
  }
#endif
    UI_MEDIUM; // do check encoder
    updateTrapezoids(lines_write_pos);
    // how much steps on primary axis do we need to reach target feedrate
    //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
  #else
  #ifdef USE_ADVANCE
  #ifdef ENABLE_QUADRATIC_ADVANCE
    p->advanceRate = 0; // No advance for constant speeds
    p->advanceFull = 0;
  #endif
  #endif
  #endif

  // Correct integers for fixed point math used in bresenham_step
  if(p->fullInterval<MAX_HALFSTEP_INTERVAL || critical)
    p->halfstep = 0;
  else {
    p->halfstep = 1;
#if DRIVE_SYSTEM==3
    // Error 0-2 are used for the towers and set up in the timer
    p->error[3] = p->stepsRemaining;
#else
    p->error[0] = p->error[1] = p->error[2] = p->error[3] = p->delta[p->primaryAxis];
#endif
  }
#ifdef DEBUG_STEPCOUNT
// Set in delta move calculation
#if DRIVE_SYSTEM!=3
  p->totalStepsRemaining = p->delta[0]+p->delta[1]+p->delta[2];
#endif
#endif
#ifdef DEBUG_QUEUE_MOVE
  if(DEBUG_ECHO) {
    log_printLine(p);
      OUT_P_L_LN("limitInterval:", limitInterval);
      OUT_P_F_LN("Move distance on the XYZ space:", p->distance);
      OUT_P_F_LN("Commanded feedrate:", printer_state.feedrate);
      OUT_P_F_LN("Constant full speed move time:", time_for_move);
      //log_long_array(PSTR("axis_int"),(long*)axis_interval);
      //out.println_float_P(PSTR("Plateau repro:"),slowest_axis_plateau_time_repro);
  }
#endif
  // Make result permanent
  NEXT_PLANNER_INDEX(lines_write_pos);
  if (pathOptimize) waitRelax = 70;
BEGIN_INTERRUPT_PROTECTED
  lines_count++;
END_INTERRUPT_PROTECTED
  DEBUG_MEMORY;
}

#if DRIVE_SYSTEM != 3
/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
  @param check_endstops Read endstop during move.
*/
void queue_move(byte check_endstops,byte pathOptimize) {
  printer_state.flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED; // Motor is enabled now
  while(lines_count>=MOVE_CACHE_SIZE) { // wait for a free entry in movement cache
    gcode_read_serial();
    check_periodical();
  }
  byte newPath=check_new_move(pathOptimize, 0);
  PrintLine *p = &lines[lines_write_pos];
  float axis_diff[4]; // Axis movement in mm
  if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS;
  else p->flags = 0;
  p->joinFlags = 0;
  if(!pathOptimize) p->joinFlags = FLAG_JOIN_END_FIXED;
  p->dir = 0;
#if min_software_endstop_x == true
    if (printer_state.destinationSteps[0] < 0) printer_state.destinationSteps[0] = 0.0;
#endif
#if min_software_endstop_y == true
    if (printer_state.destinationSteps[1] < 0) printer_state.destinationSteps[1] = 0.0;
#endif
#if min_software_endstop_z == true
    if (printer_state.destinationSteps[2] < 0) printer_state.destinationSteps[2] = 0.0;
#endif

#if max_software_endstop_x == true
    if (printer_state.destinationSteps[0] > printer_state.xMaxSteps) printer_state.destinationSteps[0] = printer_state.xMaxSteps;
#endif
#if max_software_endstop_y == true
    if (printer_state.destinationSteps[1] > printer_state.yMaxSteps) printer_state.destinationSteps[1] = printer_state.yMaxSteps;
#endif
#if max_software_endstop_z == true
    if (printer_state.destinationSteps[2] > printer_state.zMaxSteps) printer_state.destinationSteps[2] = printer_state.zMaxSteps;
#endif
  //Find direction
#if DRIVE_SYSTEM==0 || defined(NEW_XY_GANTRY)
  for(byte i=0; i < 4; i++) {
    if((p->delta[i]=printer_state.destinationSteps[i]-printer_state.currentPositionSteps[i])>=0) {
      p->dir |= 1<<i;
    } else {
      p->delta[i] = -p->delta[i];
    }
    if(i==3 && printer_state.extrudeMultiply!=100) {
      p->delta[3]=(long)((p->delta[3]*(float)printer_state.extrudeMultiply)*0.01f); 
      //p->delta[3]=(p->delta[3]*printer_state.extrudeMultiply)/100;
    }
    axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i];
    if(p->delta[i]) p->dir |= 16<<i;
    printer_state.currentPositionSteps[i] = printer_state.destinationSteps[i];
  }
  printer_state.filamentPrinted+=axis_diff[3];
#else
  long deltax = printer_state.destinationSteps[0]-printer_state.currentPositionSteps[0];
  long deltay = printer_state.destinationSteps[1]-printer_state.currentPositionSteps[1];
#if DRIVE_SYSTEM==1
  p->delta[2] = printer_state.destinationSteps[2]-printer_state.currentPositionSteps[2];
  p->delta[3] = printer_state.destinationSteps[3]-printer_state.currentPositionSteps[3];
  p->delta[0] = deltax+deltay;
  p->delta[1] = deltax-deltay;
#endif
#if DRIVE_SYSTEM==2
  p->delta[2] = printer_state.destinationSteps[2]-printer_state.currentPositionSteps[2];
  p->delta[3] = printer_state.destinationSteps[3]-printer_state.currentPositionSteps[3];
  p->delta[0] = deltay+deltax;
  p->delta[1] = deltay-deltax;
#endif
  //Find direction
  for(byte i=0; i < 4; i++) {
    if(p->delta[i]>=0) {
      p->dir |= 1<<i;
      axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i];
    } else {
      axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i];
      p->delta[i] = -p->delta[i];
    }
    if(p->delta[i]) p->dir |= 16<<i;
    printer_state.currentPositionSteps[i] = printer_state.destinationSteps[i];
  }
#endif
  if((p->dir & 240)==0) {
    if(newPath) { // need to delete dummy elements, otherwise commands can get locked.
      lines_count = 0;
      lines_pos = lines_write_pos;
    }
    return; // No steps included
  }
  byte primary_axis;
  float xydist2;
#if USE_OPS==1
  p->opsReverseSteps=0;
#endif
#if ENABLE_BACKLASH_COMPENSATION
  if((p->dir & 112) && ((p->dir & 7)^(printer_state.backlashDir & 7)) & (printer_state.backlashDir >> 3)) { // We need to compensate backlash, add a move
    while(lines_count>=MOVE_CACHE_SIZE-1) { // wait for a second free entry in movement cache
      gcode_read_serial();
      check_periodical();
    }
    byte wpos2 = lines_write_pos+1;
    if(wpos2>=MOVE_CACHE_SIZE) wpos2 = 0;
    PrintLine *p2 = &lines[wpos2];
    memcpy(p2,p,sizeof(PrintLine)); // Move current data to p2
    byte changed = (p->dir & 7)^(printer_state.backlashDir & 7);
    float back_diff[4]; // Axis movement in mm
    back_diff[3] = 0;
    back_diff[0] = (changed & 1 ? (p->dir & 1 ? printer_state.backlashX : -printer_state.backlashX) : 0);
    back_diff[1] = (changed & 2 ? (p->dir & 2 ? printer_state.backlashY : -printer_state.backlashY) : 0);
    back_diff[2] = (changed & 4 ? (p->dir & 4 ? printer_state.backlashZ : -printer_state.backlashZ) : 0);
    p->dir &=7; // x,y and z are already correct
    for(byte i=0; i < 4; i++) {
      float f = back_diff[i]*axis_steps_per_unit[i];
      p->delta[i] = abs((long)f);
      if(p->delta[i]) p->dir |= 16<<i;
    }
    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
    if(p->delta[1] > p->delta[0] && p->delta[1] > p->delta[2]) p->primaryAxis = 1;
    else if (p->delta[0] > p->delta[2] ) p->primaryAxis = 0;
    else p->primaryAxis = 2;
    p->stepsRemaining = p->delta[p->primaryAxis];
    //Feedrate calc based on XYZ travel distance
    xydist2 = back_diff[0] * back_diff[0] + back_diff[1] * back_diff[1];
    if(p->dir & 64) {
      p->distance = sqrt(xydist2 + back_diff[2] * back_diff[2]);
    } else {
      p->distance = sqrt(xydist2);
    }
    printer_state.backlashDir = (printer_state.backlashDir & 56) | (p2->dir & 7);
    calculate_move(p,back_diff,false,pathOptimize);    
    p = p2; // use saved instance for the real move
  } 
#endif

  //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  if(p->delta[1] > p->delta[0] && p->delta[1] > p->delta[2] && p->delta[1] > p->delta[3]) primary_axis = 1;
  else if (p->delta[0] > p->delta[2] && p->delta[0] > p->delta[3]) primary_axis = 0;
  else if (p->delta[2] > p->delta[3]) primary_axis = 2;
  else primary_axis = 3;
  p->primaryAxis = primary_axis;
  p->stepsRemaining = p->delta[primary_axis];
  //Feedrate calc based on XYZ travel distance
  // TODO - Simplify since Z will always move
  if(p->dir & 112) {
#if DRIVE_SYSTEM==0 || defined(NEW_XY_GANTRY)
    xydist2 = axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1];
#else
    float dx,dy;
#if DRIVE_SYSTEM==1
    dx = 0.5*(axis_diff[0]+axis_diff[1]);
    dy = axis_diff[0]-dx;
#endif
#if DRIVE_SYSTEM==2
    dy = 0.5*(axis_diff[0]+axis_diff[1]);
    dx = axis_diff[0]-dy;
#endif
    xydist2 = dx*dx+dy*dy;
#endif
    if(p->dir & 64) {
      p->distance = sqrt(xydist2 + axis_diff[2] * axis_diff[2]);
    } else {
      p->distance = sqrt(xydist2);
    }
  }  else if(p->dir & 128)
    p->distance = fabs(axis_diff[3]);
  else {
    return; // no steps to take, we are finished
  }
  calculate_move(p,axis_diff,check_endstops,pathOptimize);
}
#endif

#if DRIVE_SYSTEM==3
#define DEBUG_DELTA_OVERFLOW
/**
  Calculate and cache the delta robot positions of the cartesian move in a line.
  @return The largest delta axis move in a single segment
  @param p The line to examine.
*/
inline long calculate_delta_segments(PrintLine *p, byte softEndstop) {

	long destination_steps[3], destination_delta_steps[3];

	for(byte i=0; i < NUM_AXIS - 1; i++) {
		// Save current position
		destination_steps[i] = printer_state.currentPositionSteps[i];
	}

//	out.println_byte_P(PSTR("Calculate delta segments:"), p->numDeltaSegments);
	p->deltaSegmentReadPos = delta_segment_write_pos;
#ifdef DEBUG_STEPCOUNT
	p->totalStepsRemaining=0;
#endif

	long max_axis_move = 0;
	unsigned int produced_segments = 0;
	for (int s = p->numDeltaSegments; s > 0; s--) {
		for(byte i=0; i < NUM_AXIS - 1; i++)
			destination_steps[i] += (printer_state.destinationSteps[i] - destination_steps[i]) / s;

		// Wait for buffer here
		while(delta_segment_count + produced_segments>=DELTA_CACHE_SIZE) { // wait for a free entry in movement cache
			gcode_read_serial();
			check_periodical();
		}

		DeltaSegment *d = &segments[delta_segment_write_pos];

		// Verify that delta calc has a solution
		if (calculate_delta(destination_steps, destination_delta_steps)) {
			d->dir = 0;
			for(byte i=0; i < NUM_AXIS - 1; i++) {
				if (softEndstop && destination_delta_steps[i] > printer_state.maxDeltaPositionSteps)
					destination_delta_steps[i] = printer_state.maxDeltaPositionSteps;
				long delta = destination_delta_steps[i] - printer_state.currentDeltaPositionSteps[i];
//#ifdef DEBUG_DELTA_CALC
//				out.println_long_P(PSTR("dest:"), destination_delta_steps[i]);
//				out.println_long_P(PSTR("cur:"), printer_state.currentDeltaPositionSteps[i]);
//#endif
				if (delta == 0) {
					d->deltaSteps[i] = 0;
				} else if (delta > 0) {
					d->dir |= 17<<i;
	#ifdef DEBUG_DELTA_OVERFLOW
					if (delta > 65535)
						out.println_long_P(PSTR("Delta overflow:"), delta);
	#endif
					d->deltaSteps[i] = delta;
				} else {
					d->dir |= 16<<i;
	#ifdef DEBUG_DELTA_OVERFLOW
					if (-delta > 65535)
						out.println_long_P(PSTR("Delta overflow:"), delta);
	#endif
					d->deltaSteps[i] = -delta;
				}
	#ifdef DEBUG_STEPCOUNT
				p->totalStepsRemaining += d->deltaSteps[i];
	#endif

				if (max_axis_move < d->deltaSteps[i]) max_axis_move = d->deltaSteps[i];
				printer_state.currentDeltaPositionSteps[i] = destination_delta_steps[i];
			}
		} else {
			// Illegal position - idnore move
			out.println_P(PSTR("Invalid delta coordinate - move ignored"));
			d->dir = 0;
			for(byte i=0; i < NUM_AXIS - 1; i++) {
				d->deltaSteps[i]=0;
			}
		}
		// Move to the next segment
		delta_segment_write_pos++; if (delta_segment_write_pos >= DELTA_CACHE_SIZE) delta_segment_write_pos=0;
		produced_segments++;
	}
	BEGIN_INTERRUPT_PROTECTED
	delta_segment_count+=produced_segments;
	END_INTERRUPT_PROTECTED

	#ifdef DEBUG_STEPCOUNT
//		out.println_long_P(PSTR("totalStepsRemaining:"), p->totalStepsRemaining);
	#endif
	return max_axis_move;
}

/**
  Set delta tower positions
  @param xaxis X tower position.
  @param yaxis Y tower position.
  @param zaxis Z tower position.
*/
inline void set_delta_position(long xaxis, long yaxis, long zaxis) {
	printer_state.currentDeltaPositionSteps[0] = xaxis;
	printer_state.currentDeltaPositionSteps[1] = yaxis;
	printer_state.currentDeltaPositionSteps[2] = zaxis;
}

/**
  Calculate the delta tower position from a cartesian position
  @param cartesianPosSteps Array containing cartesian coordinates.
  @param deltaPosSteps Result array with tower coordinates.
  @returns 1 if cartesian coordinates have a valid delta tower position 0 if not.
*/
byte calculate_delta(long cartesianPosSteps[], long deltaPosSteps[]) {
	long temp;
	long opt = DELTA_DIAGONAL_ROD_STEPS_SQUARED - sq(DELTA_TOWER1_Y_STEPS - cartesianPosSteps[Y_AXIS]);

	if ((temp = opt - sq(DELTA_TOWER1_X_STEPS - cartesianPosSteps[X_AXIS])) >= 0)
		deltaPosSteps[X_AXIS] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
	else
		return 0;

	if ((temp = opt - sq(DELTA_TOWER2_X_STEPS - cartesianPosSteps[X_AXIS])) >= 0)
		deltaPosSteps[Y_AXIS] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
	else
		return 0;

	if ((temp = DELTA_DIAGONAL_ROD_STEPS_SQUARED
		 - sq(DELTA_TOWER3_X_STEPS - cartesianPosSteps[X_AXIS])
		 - sq(DELTA_TOWER3_Y_STEPS - cartesianPosSteps[Y_AXIS])) >= 0)
		deltaPosSteps[Z_AXIS] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
	else
		return 0;

	return 1;
}

inline void calculate_dir_delta(long difference[], byte *dir, long delta[]) {
  *dir = 0;
	//Find direction
	for(byte i=0; i < 4; i++) {
		if(difference[i]>=0) {
			delta[i] = difference[i];
			*dir |= 1<<i;
		} else {
			delta[i] = -difference[i];
	}
		if(delta[i]) *dir |= 16<<i;
	}
	if(printer_state.extrudeMultiply!=100) {
		delta[3]=(long)((delta[3]*(float)printer_state.extrudeMultiply)*0.01f);
	}
}

inline byte calculate_distance(float axis_diff[], byte dir, float *distance) {
  // Calculate distance depending on direction
	if(dir & 112) {
		if(dir & 64) {
			*distance = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1] + axis_diff[2] * axis_diff[2]);
		} else {
			*distance = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1]);
		}
	} else if(dir & 128)
		*distance = fabs(axis_diff[3]);
	else {
		return 0; // no steps to take, we are finished
	}
  return 1;
}

#ifdef SOFTWARE_LEVELING
void calculate_plane(long factors[], long p1[], long p2[], long p3[]) {
	factors[0] = p1[1] * (p2[2] - p3[2]) + p2[1] * (p3[2] - p1[2]) + p3[1] * (p1[2] - p2[2]);
	factors[1] = p1[2] * (p2[0] - p3[0]) + p2[2] * (p3[0] - p1[0]) + p3[2] * (p1[0] - p2[0]);
	factors[2] = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]);
	factors[3] = p1[0] * ((p2[1] * p3[2]) - (p3[1] * p2[2])) + p2[0] * ((p3[1] * p1[2]) - (p1[1] * p3[2])) + p3[0] * ((p1[1] * p2[2]) - (p2[1] * p1[2]));
}

float calc_zoffset(long factors[], long pointX, long pointY) {
	return (factors[3] - factors[0] * pointX - factors[1] * pointY) / (float) factors[2];
}
#endif

inline void queue_E_move(long e_diff,byte check_endstops,byte pathOptimize) {
  printer_state.flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED; // Motor is enabled now
  while(lines_count>=MOVE_CACHE_SIZE) { // wait for a free entry in movement cache
    gcode_read_serial();
    check_periodical();
  }
  byte newPath=check_new_move(pathOptimize, 0);
  PrintLine *p = &lines[lines_write_pos];
  float axis_diff[4]; // Axis movement in mm
  if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS;
  else p->flags = 0;
  p->joinFlags = 0;
  if(!pathOptimize) p->joinFlags = FLAG_JOIN_END_FIXED;
  p->dir = 0;
  //Find direction
  for(byte i=0; i< 3; i++) {
	p->delta[i] = 0;
	axis_diff[i] = 0;
  }
  axis_diff[3] = e_diff*inv_axis_steps_per_unit[3];
  if (e_diff >= 0) {
	p->delta[3] = e_diff;
	p->dir = 0x88;
  } else {
	p->delta[3] = -e_diff;
	p->dir = 0x80;
  }
  if(printer_state.extrudeMultiply!=100) {
    //p->delta[3]=(p->delta[3]*printer_state.extrudeMultiply)/100;
    p->delta[3]=(long)((p->delta[3]*(float)printer_state.extrudeMultiply)*0.01f);
  }
  printer_state.currentPositionSteps[3] = printer_state.destinationSteps[3];

#if USE_OPS==1
  p->opsReverseSteps=0;
#endif
  p->numDeltaSegments = 0;
  //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  p->primaryAxis = 3;
  p->stepsRemaining = p->delta[3];
  p->distance = fabs(axis_diff[3]);
  p->moveID = lastMoveID++;
  calculate_move(p,axis_diff,check_endstops,pathOptimize);
}

/**
  Split a line up into a series of lines with at most MAX_DELTA_SEGMENTS_PER_LINE delta segments.
  @param check_endstops Check endstops during the move.
  @param pathOptimize Run the path optimizer.
  @param delta_step_rate delta step rate in segments per second for the move.
*/
void split_delta_move(byte check_endstops,byte pathOptimize, byte softEndstop) {
    if (softEndstop && printer_state.destinationSteps[2] < 0) printer_state.destinationSteps[2] = 0;
	long difference[NUM_AXIS];
	float axis_diff[5]; // Axis movement in mm. Virtual axis in 4;
	for(byte i=0; i < NUM_AXIS; i++) {
		difference[i] = printer_state.destinationSteps[i] - printer_state.currentPositionSteps[i];
		axis_diff[i] = difference[i] * inv_axis_steps_per_unit[i];
	}
    printer_state.filamentPrinted+=axis_diff[3];

#if max_software_endstop_r == true
// TODO - Implement radius checking
// I'm guessing I need the floats to prevent overflow. This is pretty horrible.
// The NaN checking in the delta calculation routine should be enough
//float a = difference[0] * difference[0] + difference[1] * difference[1];
//float b = 2 * (difference[0] * printer_state.currentPositionSteps[0] + difference[1] * printer_state.currentPositionSteps[1]);
//float c = printer_state.currentPositionSteps[0] * printer_state.currentPositionSteps[0] + printer_state.currentPositionSteps[1] * printer_state.currentPositionSteps[1] - r * r;
//float disc = b * b - 4 * a * c;
//if (disc >= 0) {
//    float t = (-b + (float)sqrt(disc)) / (2 * a);
//    printer_state.destinationSteps[0] = (long) printer_state.currentPositionSteps[0] + difference[0] * t;
//    printer_state.destinationSteps[1] = (long) printer_state.currentPositionSteps[1] + difference[1] * t;
//}
#endif

	float save_distance;
	byte save_dir;
	long save_delta[4];
	calculate_dir_delta(difference, &save_dir, save_delta);
	if (!calculate_distance(axis_diff, save_dir, &save_distance))
		return;

	if (!(save_dir & 112)) {
		queue_E_move(difference[3],check_endstops,pathOptimize);
		return;
	}
	
	int segment_count;
	int num_lines;
	int segments_per_line;

	if (save_dir & 48) {
		// Compute number of seconds for move and hence number of segments needed
		float seconds = 100 * save_distance / (printer_state.feedrate * printer_state.feedrateMultiply);
#ifdef DEBUG_SPLIT
		out.println_float_P(PSTR("Seconds: "), seconds);
#endif
		segment_count = max(1, int(((save_dir & 136)==136 ? DELTA_SEGMENTS_PER_SECOND_PRINT : DELTA_SEGMENTS_PER_SECOND_MOVE) * seconds));
		// Now compute the number of lines needed
		num_lines = (segment_count + MAX_DELTA_SEGMENTS_PER_LINE - 1)/MAX_DELTA_SEGMENTS_PER_LINE;
		// There could be some error here but it doesn't matter since the number of segments will just be reduced slightly
		segments_per_line = segment_count / num_lines;
	} else {
		// Optimize pure Z axis move. Since a pure Z axis move is linear all we have to watch out for is unsigned integer overuns in
		// the queued moves;
#ifdef DEBUG_SPLIT
		out.println_long_P(PSTR("Z delta: "), save_delta[2]);
#endif
		segment_count = (save_delta[2] + (unsigned long)65534) / (unsigned long)65535;
		num_lines = (segment_count + MAX_DELTA_SEGMENTS_PER_LINE - 1)/MAX_DELTA_SEGMENTS_PER_LINE;
		segments_per_line = segment_count / num_lines;
	}

	long start_position[4], fractional_steps[4];
	for (byte i = 0; i < 4; i++) {
		start_position[i] = printer_state.currentPositionSteps[i];
	}

#ifdef DEBUG_SPLIT
	out.println_int_P(PSTR("Segments:"), segment_count);
	out.println_int_P(PSTR("Num lines:"), num_lines);
	out.println_int_P(PSTR("segments_per_line:"), segments_per_line);
#endif

	printer_state.flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED; // Motor is enabled now
	while(lines_count>=MOVE_CACHE_SIZE) { // wait for a free entry in movement cache
		gcode_read_serial();
		check_periodical();
	}

	// Insert dummy moves if necessary
	// Nead to leave at least one slot open for the first split move
	byte newPath=check_new_move(pathOptimize, min(MOVE_CACHE_SIZE-4,num_lines-1));

	for (int line_number=1; line_number < num_lines + 1; line_number++) {
		while(lines_count>=MOVE_CACHE_SIZE) { // wait for a free entry in movement cache
			gcode_read_serial();
			check_periodical();
		}
		PrintLine *p = &lines[lines_write_pos];
		// Downside a comparison per loop. Upside one less distance calculation and simpler code.
		if (num_lines == 1) {
			p->numDeltaSegments = segment_count;
			p->dir = save_dir;
			for (byte i=0; i < 4; i++) {
				p->delta[i] = save_delta[i];
				fractional_steps[i] = difference[i];
			}
			p->distance = save_distance;
		} else {
			for (byte i=0; i < 4; i++) {
				printer_state.destinationSteps[i] = start_position[i] + (difference[i] * line_number / num_lines);
				fractional_steps[i] = printer_state.destinationSteps[i] - printer_state.currentPositionSteps[i];
				axis_diff[i] = fractional_steps[i]*inv_axis_steps_per_unit[i];
			}
			calculate_dir_delta(fractional_steps, &p->dir, p->delta);
			calculate_distance(axis_diff, p->dir, &p->distance);
		}

		p->joinFlags = 0;
		p->moveID = lastMoveID;

		// Only set fixed on last segment
		if (line_number == num_lines && !pathOptimize)
			p->joinFlags = FLAG_JOIN_END_FIXED;

		if(check_endstops)
			p->flags = FLAG_CHECK_ENDSTOPS;
		else
			p->flags = 0;

		p->numDeltaSegments = segments_per_line;

	#if USE_OPS==1
		p->opsReverseSteps=0;
	#endif

		long max_delta_step = calculate_delta_segments(p, softEndstop);

	#ifdef DEBUG_SPLIT
		out.println_long_P(PSTR("Max DS:"), max_delta_step);
	#endif
		long virtual_axis_move = max_delta_step * segments_per_line;
		if (virtual_axis_move == 0 && p->delta[3] == 0) {
			if (num_lines!=1)
				OUT_P_LN("ERROR: No move in delta segment with > 1 segment. This should never happen and may cause a problem!");
			return;  // Line too short in low precision area
		}
		p->primaryAxis = 4; // Virtual axis will lead bresenham step either way
		if (virtual_axis_move > p->delta[3]) { // Is delta move or E axis leading
			p->stepsRemaining = virtual_axis_move;
			axis_diff[4] = virtual_axis_move * inv_axis_steps_per_unit[0]; // Steps/unit same as all the towers
			// Virtual axis steps per segment
			p->numPrimaryStepPerSegment = max_delta_step;
		} else {
			// Round up the E move to get something divisible by segment count which is greater than E move
			p->numPrimaryStepPerSegment = (p->delta[3] + segments_per_line - 1) / segments_per_line;
			p->stepsRemaining = p->numPrimaryStepPerSegment * segments_per_line;
			axis_diff[4] = p->stepsRemaining * inv_axis_steps_per_unit[0];
		}
#ifdef DEBUG_SPLIT
		out.println_long_P(PSTR("Steps Per Segment:"), p->numPrimaryStepPerSegment);
		out.println_long_P(PSTR("Virtual axis step:"), p->stepsRemaining);
#endif

		calculate_move(p,axis_diff,check_endstops,pathOptimize);
		for (byte i=0; i < 4; i++) {
			printer_state.currentPositionSteps[i] += fractional_steps[i];
		}
	}
	lastMoveID++; // Will wrap at 255
}

#endif

#if ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise)
{      
  //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
  float center_axis0 = position[0] + offset[0];
  float center_axis1 = position[1] + offset[1];
  float linear_travel = 0; //target[axis_linear] - position[axis_linear];
  float extruder_travel = printer_state.destinationSteps[3]-printer_state.currentPositionSteps[3];
  float r_axis0 = -offset[0];  // Radius vector from center to current location
  float r_axis1 = -offset[1];
  float rt_axis0 = target[0] - center_axis0;
  float rt_axis1 = target[1] - center_axis1;
  long xtarget = printer_state.destinationSteps[0];
  long ytarget = printer_state.destinationSteps[1];
  long etarget = printer_state.destinationSteps[3];
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (isclockwise) { angular_travel -= 2*M_PI; }
  
  float millimeters_of_travel = fabs(angular_travel)*radius; //hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel < 0.001) { return; }
  //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
  // Increase segment size if printing faster then computation speed allows
  uint16_t segments = (printer_state.feedrate>60 ? floor(millimeters_of_travel/min(MM_PER_ARC_SEGMENT_BIG,printer_state.feedrate*0.01666*MM_PER_ARC_SEGMENT)) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
  if(segments == 0) segments = 1;
  /*  
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
  */
  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;
  float extruder_per_segment = extruder_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;
     
     For arc generation, the center of the circle is the axis of rotation and the radius vector is 
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented. 

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.
     
     This approximation also allows mc_arc to immediately insert a line segment into the planner 
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
     This is important when there are successive arc motions. 
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  float arc_target[4];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  //arc_target[axis_linear] = position[axis_linear];
  
  // Initialize the extruder axis
  arc_target[3] = printer_state.currentPositionSteps[3];

  for (i = 1; i<segments; i++) 
  { // Increment (segments-1)
    
    if((count & 4) == 0)
    {
       gcode_read_serial();
       check_periodical();
       UI_MEDIUM; // do check encoder
    }

    if (count < N_ARC_CORRECTION)  //25 pieces
    {
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    }
    else
    {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti  = cos(i*theta_per_segment);
      sin_Ti  = sin(i*theta_per_segment);
      r_axis0 = -offset[0]*cos_Ti + offset[1]*sin_Ti;
      r_axis1 = -offset[0]*sin_Ti - offset[1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[0] = center_axis0 + r_axis0;
    arc_target[1] = center_axis1 + r_axis1;
    //arc_target[axis_linear] += linear_per_segment;
    arc_target[3] += extruder_per_segment;
    
    printer_state.destinationSteps[0] = arc_target[0]*axis_steps_per_unit[0];
    printer_state.destinationSteps[1] = arc_target[1]*axis_steps_per_unit[1];
    printer_state.destinationSteps[3] = arc_target[3];
#if DRIVE_SYSTEM == 3
    split_delta_move(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
    queue_move(ALWAYS_CHECK_ENDSTOPS,true);
#endif
  }
  // Ensure last segment arrives at target location.
  printer_state.destinationSteps[0] = xtarget;
  printer_state.destinationSteps[1] = ytarget;
  printer_state.destinationSteps[3] = etarget;
#if DRIVE_SYSTEM == 3
  split_delta_move(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
  queue_move(ALWAYS_CHECK_ENDSTOPS,true);
#endif
}
#endif
