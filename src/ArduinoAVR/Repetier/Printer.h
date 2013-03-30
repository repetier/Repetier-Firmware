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
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#ifndef PRINTER_H_INCLUDED
#define PRINTER_H_INCLUDED


#define PRINTER_FLAG0_STEPPER_DISABLED      1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT     4
#define PRINTER_FLAG0_FORCE_CHECKSUM        8

class Printer
{
public:
    static byte flag0; // 1 = stepper disabled, 2 = use external extruder interrupt, 4 = temp Sensor defect
#if USE_OPS==1 || defined(USE_ADVANCE)
    volatile int extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//  float extruderSpeed;              ///< Extruder speed in mm/s.
    byte minExtruderSpeed;            ///< Timer delay for start extruder speed
    byte maxExtruderSpeed;            ///< Timer delay for end extruder speed
    byte extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
    static long interval;                    ///< Last step duration in ticks.
#if USE_OPS==1
    bool filamentRetracted;           ///< Is the extruder filament retracted
#endif
    unsigned long timer;              ///< used for acceleration/deceleration timing
    unsigned long stepNumber;         ///< Step number in current move.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    long advance_executed;             ///< Executed advance steps
#endif
    int advance_steps_set;
    unsigned int advance_lin_set;
#endif
    static long currentPositionSteps[4];     ///< Position in steps from origin.
    static long destinationSteps[4];         ///< Target position in steps.
#if DRIVE_SYSTEM==3
#ifdef STEP_COUNTER
    long countZSteps;					///< Count of steps from last position reset
#endif
    long currentDeltaPositionSteps[4];
    long maxDeltaPositionSteps;
#endif
#ifdef SOFTWARE_LEVELING
    long levelingP1[3];
    long levelingP2[3];
    long levelingP3[3];
#endif
#if USE_OPS==1
    int opsRetractSteps;              ///< Retract filament this much steps
    int opsPushbackSteps;             ///< Retract+extra distance for backsash
    float opsMinDistance;
    float opsRetractDistance;
    float opsRetractBacklash;
    byte opsMode;                     ///< OPS operation mode. 0 = Off, 1 = Classic, 2 = Fast
    float opsMoveAfter;               ///< Start move after opsModeAfter percent off full retract.
    int opsMoveAfterSteps;            ///< opsMoveAfter converted in steps (negative value!).
#endif
    float minimumSpeed;               ///< lowest allowed speed to keep integration error small
    long xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    long yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    long zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    long xMinSteps;                   ///< For software endstops, limit of move in negative direction.
    long yMinSteps;                   ///< For software endstops, limit of move in negative direction.
    long zMinSteps;                   ///< For software endstops, limit of move in negative direction.
    float xLength;
    float xMin;
    float yLength;
    float yMin;
    float zLength;
    float zMin;
    float feedrate;                   ///< Last requested feedrate.
    int feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
    unsigned int extrudeMultiply;     ///< Flow multiplier in percdent (factor 1 = 100)
    float maxJerk;                    ///< Maximum allowed jerk in mm/s
    float maxZJerk;                   ///< Maximum allowed jerk in z direction in mm/s
    long offsetX;                     ///< X-offset for different extruder positions.
    long offsetY;                     ///< Y-offset for different extruder positions.
    unsigned int vMaxReached;         ///< MAximumu reached speed
    byte stepper_loops;
    unsigned long msecondsPrinting;            ///< Milliseconds of printing time (means time with heated extruder)
    float filamentPrinted;            ///< mm of filament printed since counting started
    byte waslasthalfstepping;         ///< Indicates if last move had halfstepping enabled
#if ENABLE_BACKLASH_COMPENSATION
    float backlashX;
    float backlashY;
    float backlashZ;
    byte backlashDir;
#endif
#ifdef DEBUG_STEPCOUNT
    long totalStepsRemaining;
#endif
#if FEATURE_MEMORY_POSITION
    long memoryX;
    long memoryY;
    long memoryZ;
#endif
#ifdef XY_GANTRY
    char motorX;
    char motorY;
#endif

    /** \brief Disable stepper motor for x direction. */
    static inline void disableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN,!X_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for y direction. */
    static inline void disableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for z direction. */
    static inline void disableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for x direction. */
    static inline void  enableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for y direction. */
    static inline void  enableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for z direction. */
    static inline void  enableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
    }


    inline byte isAdvanceActivated()
    {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    }
    inline void setAdvanceActivated(byte b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    }
    static inline float convertToMM(float x)
    {
        return (unit_inches ? x*25.4 : x);
    }
    static inline bool isXMinEndstopHit()
    {
#if X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
        return READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isYMinEndstopHit()
    {
#if Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
        return READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isZMinEndstopHit()
    {
#if Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
        return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isXMaxEndstopHit()
    {
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
        return READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isYMaxEndstopHit()
    {
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
        return READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING;
#else
        return false;
#endif
    }
    static inline bool isZMaxEndstopHit()
    {
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
        return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
#else
        return false;
#endif
    }
    static inline bool areAllSteppersDisabled()
    {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static inline void setAllSteppersDiabled()
    {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static inline void unsetAllSteppersDisabled()
    {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;
    }
    inline void executeXYGantrySteps()
    {
#if defined(XY_GANTRY)
        if(motorX <= -2)
        {
            ANALYZER_ON(ANALYZER_CH2);
            WRITE(X_STEP_PIN,HIGH);
            motorX += 2;
        }
        else if(motorX >= 2)
        {
            ANALYZER_ON(ANALYZER_CH2);
            WRITE(X_STEP_PIN,HIGH);
            motorX -= 2;
        }
        if(motorY <= -2)
        {
            ANALYZER_ON(ANALYZER_CH3);
            WRITE(Y_STEP_PIN,HIGH);
            motorY += 2;
        }
        else if(motorY >= 2)
        {
            ANALYZER_ON(ANALYZER_CH3);
            WRITE(Y_STEP_PIN,HIGH);
            motorY -= 2;
        }
#endif
    }
    static inline void endXYZSteps() {
            WRITE(X_STEP_PIN,LOW);
            WRITE(Y_STEP_PIN,LOW);
            WRITE(Z_STEP_PIN,LOW);
            ANALYZER_OFF(ANALYZER_CH1);
            ANALYZER_OFF(ANALYZER_CH2);
            ANALYZER_OFF(ANALYZER_CH3);
            ANALYZER_OFF(ANALYZER_CH6);
            ANALYZER_OFF(ANALYZER_CH7);
    }
};
extern Printer printer;


#endif // PRINTER_H_INCLUDED
