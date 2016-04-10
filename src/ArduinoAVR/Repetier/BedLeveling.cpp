
/*
More and more printers now have automatic bed leveling using an ever increasing variety of methods.
This makes the leveling routine one of the most complex parts of the firmware and there is not one
way to level but hundreds of combinations.

First you should decide on the correction method. Once we know how our bed is tilted we want to
remove that. This correction is defined by BED_CORRECTION_METHOD and allows the following values:
BED_CORRECTION_METHOD 0
Use a rotation matrix. This will make z axis go up/down while moving in x/y direction to compensate
the tilt. For multiple extruders make sure the height match the tilt of the bed or one will scratch.

BED_CORRECTION_METHOD 1
Motorized correction. This method needs a bed that is fixed on 3 points from which 2 have a motor
to change the height. The positions are defined by
BED_MOTOR_1_X, BED_MOTOR_1_Y, BED_MOTOR_2_X, BED_MOTOR_2_Y, BED_MOTOR_3_X, BED_MOTOR_3_Y
Motor 2 and 3 are the one driven by motor driver 0 and 1. These can be extra motors like Felix Pro 1
uses them or a system with 3 z axis where motors can be controlled individually like the Sparkcube
does.

Next we have to distinguish several methods of z probing sensors. Each have their own advantages and
disadvantages. First the z probe has a position when activated and that position is defined by
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
This is needed since we need to know where we measure the height when the z probe triggers. When
probing is activated you will see a move to make probe go over current extruder position. The
position can be changed in eeprom later on.

Some probes need to be activated/deactivated so we can use them. This is defined in the scripts
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""

Now when we probe we want to know the distance of the extruder to the bed. This is defined by
#define Z_PROBE_HEIGHT 4
The 4 means that when we trigger the distance of the nozzle tip is 4mm. If your switch tends
to return different points you might repeat a measured point and use the average height:
#define Z_PROBE_SWITCHING_DISTANCE 1
#define Z_PROBE_REPETITIONS 5
Switching distance is the z raise needed to turn back a signal reliably to off. Inductive sensors
need only a bit while mechanical switches may require a bit more.

Next thing to consider is the force for switching. Some beds use a cantilever design and pushing on
the outside easily bends the bed. If your sensor needs some force to trigger you add the error of
bending. For this reason you might add a bending correction. Currently you define
#define BENDING_CORRECTION_A 0
#define BENDING_CORRECTION_B 0
#define BENDING_CORRECTION_C 0
which are the deflections at the 3 z probe points. For all other possible measurements these values
get interpolated. You can modify the values later on in eeprom. For force less sensors set them to 0.

Next thing is endstop handling. Without bed leveling you normally home to minimum position for x,y and z.
With bed leveling this is not that easy any more. Since we do bed leveling we already assume the bed is
not leveled for x/y moves. So without correction we would hit the bed for different x/y positions at
different heights. As a result we have no real minimum position. That makes a z min endstop quite useless.
There is an exception to this. If your nozzle triggers z min or if a inductive sensor would trigger at a given
position we could use that signal. With nozzle triggers you need to be careful as a drop of filament
would change the height. The other problem is that while homing the auto leveling is not used. So
the only position would be if the z min sensor is directly over the 0,0 coordinate which is the rotation point
if we have matrix based correction. For motor based correction this will work everywhere correctly.

So the only useful position for a z endstop is z max position. Apart from not having the bed tilt problem it
also allows homing with a full bed so you can continue an aborted print with some gcode tweaking. With z max
homing we adjust the error by simply changing the max. z height. One thing you need to remember is setting
#define ENDSTOP_Z_BACK_ON_HOME 4
so we release the z max endstop. This is very important if we move xy at z max. Auto leveling might want to
increase z and the endstop might prevent it causing wrong position and a head crash if we later go down.
The value should be larger then the maximum expected tilt.

Now it is time to define how we measure the bed rotation. Here again we have several methods to choose.
All methods need at least 3 points to define the bed rotation correctly. The quality we get comes
from the selection of the right points and method.

BED_LEVELING_METHOD 0
This method measures at the 3 probe points and creates a plane through these points. If you have
a really planar bed this gives the optimum result. The 3 points must not be in one line and have
a long distance to increase numerical stability.

BED_LEVELING_METHOD 1
This measures a grid. Probe point 1 is the origin and points 2 and 3 span a grid. We measure
BED_LEVELING_GRID_SIZE points in each direction and compute a regression plane through all
points. This gives a good overall plane if you have small bumps measuring inaccuracies.

BED_LEVELING_METHOD 2
Bending correcting 4 point measurement. This is for cantilevered beds that have the rotation axis
not at the side but inside the bed. Here we can assume no bending on the axis and a symmetric
bending to both sides of the axis. So probe points 2 and 3 build the symmetric axis and
point 1 is mirrored to 1m across the axis. Using the symmetry we then remove the bending
from 1 and use that as plane.

By now the leveling process is finished. All errors that remain are measuring errors and bumps on
the bed it self. For deltas you can enable distortion correction to follow the bumps.

There are 2 ways to consider a changing bed coating, which are defined by Z_PROBE_Z_OFFSET_MODE.
Z_PROBE_Z_OFFSET_MODE = 0 means we measure the surface of the bed below any coating. This is e.g. 
the case with inductive sensors where we put BuildTak on top. In that case we can set Z_PROBE_Z_OFFSET
to the thickness of BuildTak to compensate. If we later change the coating, we only change Z_PROBE_Z_OFFSET
to new coating thickness.

Z_PROBE_Z_OFFSET_MODE = 1 means we measure the surface of the coating, e.g. because we have a mechanical switch.
In that case we add Z_PROBE_Z_OFFSET for the measured height to compensate for correct distance to bed surface.

In homing to max we reduce z length by Z_PROBE_Z_OFFSET to get a correct height.
In homing to z min we assume z endstop is bed level so we move up Z_PROBE_Z_OFFSET after endstop is hit. This 
requires the extruder to bend the coating thickness without harm!
*/

#include "Repetier.h"

#ifndef BED_LEVELING_METHOD
#define BED_LEVELING_METHOD 0
#endif

#ifndef BED_CORRECTION_METHOD
#define BED_CORRECTION_METHOD 0
#endif

#ifndef BED_LEVELING_GRID_SIZE
#define BED_LEVELING_GRID_SIZE 5
#endif

#ifndef BED_LEVELING_REPETITIONS
#define BED_LEVELING_REPETITIONS 1
#endif


class PlaneBuilder {
        float sum_xx,sum_xy,sum_yy,sum_x,sum_y,sum_xz,sum_yz,sum_z,n;
    public:
		PlaneBuilder() {
			reset();
		}
        void reset() {
            sum_xx = sum_xy = sum_yy = sum_x = sum_y = sum_xz = sum_yz = sum_z = n = 0;
        }
        void addPoint(float x,float y,float z) {
            n++;
            sum_xx += x * x;
            sum_xy += x * y;
            sum_yy += y * y;
            sum_x  += x;
            sum_y  += y;
            sum_xz += x * z;
            sum_yz += y * z;
            sum_z  += z;
        }
        void createPlane(Plane &plane,bool silent=false) {
            float det = (sum_x * (sum_xy * sum_y - sum_x * sum_yy) + sum_xx * (n * sum_yy - sum_y * sum_y) + sum_xy * (sum_x * sum_y - n * sum_xy));
            plane.a = ((sum_xy * sum_y  - sum_x * sum_yy)  * sum_z + (sum_x * sum_y  - n      * sum_xy) * sum_yz + sum_xz * (n      * sum_yy - sum_y * sum_y))  / det;
            plane.b = ((sum_x  * sum_xy - sum_xx * sum_y)  * sum_z + (n     * sum_xx - sum_x  * sum_x)  * sum_yz + sum_xz * (sum_x  * sum_y  - n     * sum_xy)) / det;
            plane.c = ((sum_xx * sum_yy - sum_xy * sum_xy) * sum_z + (sum_x * sum_xy - sum_xx * sum_y)  * sum_yz + sum_xz * (sum_xy * sum_y  - sum_x * sum_yy)) / det;
			if(!silent) {
				Com::printF(PSTR("plane: a = "),plane.a,4);
				Com::printF(PSTR(" b = "),plane.b,4);
				Com::printFLN(PSTR(" c = "),plane.c,4);
			}
        }
};

#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE

bool measureAutolevelPlane(Plane &plane) {
    PlaneBuilder builder;
    builder.reset();
#if BED_LEVELING_METHOD == 0 // 3 point
    float h;
    Printer::moveTo(EEPROM::zProbeX1(),EEPROM::zProbeY1(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h = Printer::runZProbe(false,false);
    if(h == ILLEGAL_Z_PROBE)
        return false;
    builder.addPoint(EEPROM::zProbeX1(),EEPROM::zProbeY1(),h);
    Printer::moveTo(EEPROM::zProbeX2(),EEPROM::zProbeY2(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h = Printer::runZProbe(false,false);
    if(h == ILLEGAL_Z_PROBE)
        return false;
    builder.addPoint(EEPROM::zProbeX2(),EEPROM::zProbeY2(),h);
    Printer::moveTo(EEPROM::zProbeX3(),EEPROM::zProbeY3(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h = Printer::runZProbe(false,false);
    if(h == ILLEGAL_Z_PROBE)
        return false;
    builder.addPoint(EEPROM::zProbeX3(),EEPROM::zProbeY3(),h);
#elif BED_LEVELING_METHOD == 1 // linear regression
    float delta = 1.0 / (BED_LEVELING_GRID_SIZE - 1);
    float ox = EEPROM::zProbeX1();
    float oy = EEPROM::zProbeY1();
    float ax = delta * (EEPROM::zProbeX2() - EEPROM::zProbeX1());
    float ay = delta * (EEPROM::zProbeY2() - EEPROM::zProbeY1());
    float bx = delta * (EEPROM::zProbeX3() - EEPROM::zProbeX1());
    float by = delta * (EEPROM::zProbeY3() - EEPROM::zProbeY1());
    for(int ix = 0; ix < BED_LEVELING_GRID_SIZE; ix++) {
        for(int iy = 0; iy < BED_LEVELING_GRID_SIZE; iy++) {
            float px = ox + static_cast<float>(ix) * ax + static_cast<float>(iy) * bx;
            float py = oy + static_cast<float>(ix) * ay + static_cast<float>(iy) * by;
            Printer::moveTo(px,py,IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            float h = Printer::runZProbe(false,false);
            if(h == ILLEGAL_Z_PROBE)
                return false;
            builder.addPoint(px,py,h);
        }
    }

#elif BED_LEVELING_METHOD == 2 // 4 point symmetric
    float h1,h2,h3,h4;
    float apx = EEPROM::zProbeX1() - EEPROM::zProbeX2();
    float apy = EEPROM::zProbeY1() - EEPROM::zProbeY2();
    float abx = EEPROM::zProbeX3() - EEPROM::zProbeX2();
    float aby = EEPROM::zProbeY3() - EEPROM::zProbeY2();
    float ab2 = abx * abx + aby * aby;
    float abap = apx * abx + apy * aby;
    float t = abap / ab2;
    float xx = EEPROM::zProbeX2() + t * abx;
    float xy = EEPROM::zProbeY2() + t * aby;
    float x1Mirror = EEPROM::zProbeX1() + 2.0 * (xx - EEPROM::zProbeX1());
    float y1Mirror = EEPROM::zProbeY1() + 2.0 * (xy - EEPROM::zProbeY1());
    Printer::moveTo(EEPROM::zProbeX1(),EEPROM::zProbeY1(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h1 = Printer::runZProbe(false,false);
    if(h1 == ILLEGAL_Z_PROBE)
        return false;
    Printer::moveTo(EEPROM::zProbeX2(),EEPROM::zProbeY2(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h2 = Printer::runZProbe(false,false);
    if(h2 == ILLEGAL_Z_PROBE)
        return false;
    Printer::moveTo(EEPROM::zProbeX3(),EEPROM::zProbeY3(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h3 = Printer::runZProbe(false,false);
    if(h3 == ILLEGAL_Z_PROBE)
        return false;
    Printer::moveTo(x1Mirror,y1Mirror,IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    h4 = Printer::runZProbe(false,false);
    if(h4 == ILLEGAL_Z_PROBE)
        return false;
    t = h2 + (h3 - h2) * t; // theoretical height for crossing point for symmetric axis
    h1 = t - (h4 - h1) * 0.5; // remove bending part
    builder.addPoint(EEPROM::zProbeX1(), EEPROM::zProbeY1(), h1);
    builder.addPoint(EEPROM::zProbeX2(), EEPROM::zProbeY2(), h2);
    builder.addPoint(EEPROM::zProbeX3(), EEPROM::zProbeY3(), h3);
#else
#error Unknown bed leveling method
#endif
    builder.createPlane(plane,false);
    return true;
}

void correctAutolevel(GCode *code,Plane &plane) {
#if BED_CORRECTION_METHOD == 0 // rotation matrix
    //Printer::buildTransformationMatrix(plane.z(EEPROM::zProbeX1(),EEPROM::zProbeY1()),plane.z(EEPROM::zProbeX2(),EEPROM::zProbeY2()),plane.z(EEPROM::zProbeX3(),EEPROM::zProbeY3()));
	Printer::buildTransformationMatrix(plane);
#elif BED_CORRECTION_METHOD == 1 // motorized correction
#if !defined(NUM_MOTOR_DRIVERS) || NUM_MOTOR_DRIVERS < 2
#error You need to define 2 motors for motorized bed correction
#endif
    Commands::waitUntilEndOfAllMoves(); // move steppers might be leveling steppers as well !
    float h1 = plane.z(BED_MOTOR_1_X,BED_MOTOR_1_Y);
    float h2 = plane.z(BED_MOTOR_2_X,BED_MOTOR_2_Y);
    float h3 = plane.z(BED_MOTOR_3_X,BED_MOTOR_3_Y);
    // h1 is reference heights, h2 => motor 0, h3 => motor 1
    h2 -= h1;
    h3 -= h1;
#if defined(LIMIT_MOTORIZED_CORRECTION)
	if(h2 < -LIMIT_MOTORIZED_CORRECTION) h2 = -LIMIT_MOTORIZED_CORRECTION;
	if(h2 > LIMIT_MOTORIZED_CORRECTION) h2 = LIMIT_MOTORIZED_CORRECTION;
	if(h3 < -LIMIT_MOTORIZED_CORRECTION) h3 = -LIMIT_MOTORIZED_CORRECTION;
	if(h3 > LIMIT_MOTORIZED_CORRECTION) h3 = LIMIT_MOTORIZED_CORRECTION;
#endif	
    MotorDriverInterface *motor2 = getMotorDriver(0);
    MotorDriverInterface *motor3 = getMotorDriver(1);
    motor2->setCurrentAs(0);
    motor3->setCurrentAs(0);
    motor2->gotoPosition(h2);
    motor3->gotoPosition(h3);
    motor2->disable();
    motor3->disable(); // now bed is even
    Printer::currentPositionSteps[Z_AXIS] = h1 * Printer::axisStepsPerMM[Z_AXIS];
#else
#error Unknown bed correction method set
#endif
}

/*
Implementation of the G32 command
G32 S<0..2> - Autolevel print bed. S = 1 measure zLength, S = 2 Measure and store new zLength
S = 0 : Do not update length - use this if you have not homed before or you mess up zlength!
S = 1 : Measure zLength so homing works
S = 2 : Like s = 1 plus store results in EEPROM for next connection.
*/
bool runBedLeveling(GCode *com) {
    float h1,h2,h3,hc,oldFeedrate = Printer::feedrate;
    int s = com->hasS() ? com->S : -1;
#if DISTORTION_CORRECTION
    bool distEnabled = Printer::distortion.isEnabled();
    Printer::distortion.disable(false); // if level has changed, distortion is also invalid
#endif
    Printer::setAutolevelActive(false); // iterate
    Printer::resetTransformationMatrix(true); // in case we switch from matrix to motorized!
#if DRIVE_SYSTEM == DELTA
    // It is not possible to go to the edges at the top, also users try
    // it often and wonder why the coordinate system is then wrong.
    // For that reason we ensure a correct behavior by code.
    Printer::homeAxis(true, true, true);
    Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance() + EEPROM::zProbeHeight(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
#endif
    Printer::startProbing(true);
    //GCode::executeFString(Com::tZProbeStartScript);
    Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;
    Plane plane;
#if BED_CORRECTION_METHOD == 1
    for(int r = 0; r < BED_LEVELING_REPETITIONS; r++) {
#if DRIVE_SYSTEM == DELTA
        if(r > 0) {
            Printer::finishProbing();
            Printer::homeAxis(true, true, true);
            Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance() + EEPROM::zProbeHeight(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
            Printer::startProbing(true);
        }
#endif
#endif
        if(!measureAutolevelPlane(plane)) {
            Com::printErrorFLN(PSTR("Probing had returned errors - autoleveling canceled."));
            return false;
        }
        correctAutolevel(com,plane);

        // Leveling is finished now update own positions and store leveling data if needed
        float currentZ = plane.z((float)Printer::currentPositionSteps[X_AXIS] * Printer::invAxisStepsPerMM[X_AXIS],(float)Printer::currentPositionSteps[Y_AXIS] * Printer::invAxisStepsPerMM[Y_AXIS]);
        Com::printF(PSTR("CurrentZ:"),currentZ);
        Com::printFLN(PSTR(" atZ:"),Printer::currentPosition[Z_AXIS]);
        // With max z endstop we adjust zlength so after next homing we have also a calibrated printer
        Printer::zMin = 0;
#if MAX_HARDWARE_ENDSTOP_Z
        float xRot,yRot,zRot;
        Printer::transformFromPrinter(Printer::currentPosition[X_AXIS],Printer::currentPosition[Y_AXIS],Printer::currentPosition[Z_AXIS],xRot,yRot,zRot);
        Com::printFLN(PSTR("Z after rotation:"),zRot);
        // With max z endstop we adjust zlength so after next homing we have also a calibrated printer
        if(s != 0) {
            Printer::zLength += currentZ - zRot;
            Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
        }
#endif
        Printer::currentPositionSteps[Z_AXIS] = currentZ * Printer::axisStepsPerMM[Z_AXIS];
        Printer::updateCurrentPosition(true);
#if BED_CORRECTION_METHOD == 1
        if(fabs(plane.a) < 0.00025 && fabsf(plane.b) < 0.00025 )
            break;  // we reached achievable precision so we can stop
    }
#endif
    Printer::updateDerivedParameter();
    Printer::finishProbing();
#if BED_CORRECTION_METHOD != 1
    Printer::setAutolevelActive(true); // only for software correction or we can spare the comp. time
#endif
    if(s >= 2) {
        EEPROM::storeDataIntoEEPROM();
    }
    Printer::updateCurrentPosition(true);
    Commands::printCurrentPosition(PSTR("G32 "));
#if DISTORTION_CORRECTION
    if(distEnabled)
        Printer::distortion.enable(false); // if level has changed, distortion is also invalid
#endif
#if DRIVE_SYSTEM == DELTA
    Printer::homeAxis(true, true, true); // shifting z makes positioning invalid, need to recalibrate
#endif
    Printer::feedrate = oldFeedrate;
	return true;
}

#endif

void Printer::setAutolevelActive(bool on) {
#if FEATURE_AUTOLEVEL
    if(on == isAutolevelActive()) return;
    flag0 = (on ? flag0 | PRINTER_FLAG0_AUTOLEVEL_ACTIVE : flag0 & ~PRINTER_FLAG0_AUTOLEVEL_ACTIVE);
    if(on)
        Com::printInfoFLN(Com::tAutolevelEnabled);
    else
        Com::printInfoFLN(Com::tAutolevelDisabled);
    updateCurrentPosition(false);
#endif // FEATURE_AUTOLEVEL
}
#if MAX_HARDWARE_ENDSTOP_Z
float Printer::runZMaxProbe() {
#if NONLINEAR_SYSTEM
    long startZ = realDeltaPositionSteps[Z_AXIS] = currentNonlinearPositionSteps[Z_AXIS]; // update real
#endif
    Commands::waitUntilEndOfAllMoves();
    long probeDepth = 2*(Printer::zMaxSteps-Printer::zMinSteps);
    stepsRemainingAtZHit = -1;
    setZProbingActive(true);
    PrintLine::moveRelativeDistanceInSteps(0,0,probeDepth,0,EEPROM::zProbeSpeed(),true,true);
    if(stepsRemainingAtZHit < 0) {
        Com::printErrorFLN(PSTR("z-max homing failed"));
        return ILLEGAL_Z_PROBE;
    }
    setZProbingActive(false);
    currentPositionSteps[Z_AXIS] -= stepsRemainingAtZHit;
#if NONLINEAR_SYSTEM
    probeDepth -= (realDeltaPositionSteps[Z_AXIS] - startZ);
#else
    probeDepth -= stepsRemainingAtZHit;
#endif
    float distance = (float)probeDepth * invAxisStepsPerMM[Z_AXIS];
    Com::printF(Com::tZProbeMax,distance);
    Com::printF(Com::tSpaceXColon,realXPosition());
    Com::printFLN(Com::tSpaceYColon,realYPosition());
    PrintLine::moveRelativeDistanceInSteps(0,0,-probeDepth,0,EEPROM::zProbeSpeed(),true,true);
    return distance;
}
#endif

#if FEATURE_Z_PROBE
void Printer::startProbing(bool runScript) {
    float oldOffX = Printer::offsetX;
    float oldOffY = Printer::offsetY;
    float oldOffZ = Printer::offsetZ;
    if(runScript)
        GCode::executeFString(Com::tZProbeStartScript);
    float maxStartHeight = EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0) + 0.1;
    if(currentPosition[Z_AXIS] > maxStartHeight) {
        moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, maxStartHeight, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]);
    }
    Printer::offsetX = -EEPROM::zProbeXOffset();
    Printer::offsetY = -EEPROM::zProbeYOffset();
    Printer::offsetZ = 0; // we correct this with probe height
    PrintLine::moveRelativeDistanceInSteps((Printer::offsetX - oldOffX) * Printer::axisStepsPerMM[X_AXIS],
                                           (Printer::offsetY - oldOffY) * Printer::axisStepsPerMM[Y_AXIS],
                                           0, 0, EEPROM::zProbeXYSpeed(), true, ALWAYS_CHECK_ENDSTOPS);
}

void Printer::finishProbing() {
    float oldOffX = Printer::offsetX;
    float oldOffY = Printer::offsetY;
    float oldOffZ = Printer::offsetZ;
    GCode::executeFString(Com::tZProbeEndScript);
    if(Extruder::current) {
        Printer::offsetX = -Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
        Printer::offsetY = -Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
        Printer::offsetZ = -Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
    }
    PrintLine::moveRelativeDistanceInSteps((Printer::offsetX - oldOffX) * Printer::axisStepsPerMM[X_AXIS],
                                           (Printer::offsetY - oldOffY) * Printer::axisStepsPerMM[Y_AXIS],
                                           (Printer::offsetZ - oldOffZ) * Printer::axisStepsPerMM[Z_AXIS], 0, EEPROM::zProbeXYSpeed(), true, ALWAYS_CHECK_ENDSTOPS);
}

/*
This is the most important function for bed leveling. It does
1. Run probe start script if first = true and runStartScript = true
2. Position zProbe at current position if first = true. If we are more then maxStartHeight away from bed we also go down to that distance.
3. Measure the the steps until probe hits the bed.
4. Undo positioning to z probe and run finish script if last = true.

Now we compute the nozzle height as follows:
a) Compute average height from repeated measurements
b) Add zProbeHeight to correct difference between triggering point and nozzle height above bed
c) If Z_PROBE_Z_OFFSET_MODE == 1 we add zProbeZOffset() that is coating thickness if we measure below coating with indictive sensor.
d) Add distortion correction.
e) Add bending correction

Then we return the measured and corrected z distance.
*/
float Printer::runZProbe(bool first,bool last,uint8_t repeat,bool runStartScript) {
    float oldOffX = Printer::offsetX;
    float oldOffY = Printer::offsetY;
    float oldOffZ = Printer::offsetZ;
    if(first)
        startProbing(runStartScript);
    Commands::waitUntilEndOfAllMoves();
    int32_t sum = 0, probeDepth;
    int32_t shortMove = static_cast<int32_t>((float)Z_PROBE_SWITCHING_DISTANCE * axisStepsPerMM[Z_AXIS]); // distance to go up for repeated moves
    int32_t lastCorrection = currentPositionSteps[Z_AXIS]; // starting position
#if NONLINEAR_SYSTEM
    realDeltaPositionSteps[Z_AXIS] = currentNonlinearPositionSteps[Z_AXIS]; // update real
#endif
    //int32_t updateZ = 0;
    waitForZProbeStart();
    Endstops::update();
    Endstops::update();
    if(Endstops::zProbe()) {
        Com::printErrorFLN(PSTR("z-probe triggered before starting probing."));
        return ILLEGAL_Z_PROBE;
    }
    for(int8_t r = 0; r < repeat; r++) {
        probeDepth = 2 * (Printer::zMaxSteps - Printer::zMinSteps); // probe should always hit within this distance
        stepsRemainingAtZHit = -1; // Marker that we did not hit z probe
        //int32_t offx = axisStepsPerMM[X_AXIS] * EEPROM::zProbeXOffset();
        //int32_t offy = axisStepsPerMM[Y_AXIS] * EEPROM::zProbeYOffset();
        //PrintLine::moveRelativeDistanceInSteps(-offx,-offy,0,0,EEPROM::zProbeXYSpeed(),true,true);
        setZProbingActive(true);
        PrintLine::moveRelativeDistanceInSteps(0, 0, -probeDepth, 0, EEPROM::zProbeSpeed(), true, true);
        if(stepsRemainingAtZHit < 0) {
            Com::printErrorFLN(Com::tZProbeFailed);
            return ILLEGAL_Z_PROBE;
        }
        setZProbingActive(false);
#if NONLINEAR_SYSTEM
        stepsRemainingAtZHit = realDeltaPositionSteps[C_TOWER] - currentNonlinearPositionSteps[C_TOWER]; // nonlinear moves may split z so stepsRemainingAtZHit is only what is left from last segment not total move. This corrects the problem.
#endif
#if DRIVE_SYSTEM == DELTA
        currentNonlinearPositionSteps[A_TOWER] += stepsRemainingAtZHit; // Update difference
        currentNonlinearPositionSteps[B_TOWER] += stepsRemainingAtZHit;
        currentNonlinearPositionSteps[C_TOWER] += stepsRemainingAtZHit;
#endif
        currentPositionSteps[Z_AXIS] += stepsRemainingAtZHit; // now current position is correct
        sum += lastCorrection - currentPositionSteps[Z_AXIS];
        if(r + 1 < repeat) {
            // go only shortest possible move up for repetitions
            PrintLine::moveRelativeDistanceInSteps(0, 0, shortMove, 0, EEPROM::zProbeSpeed(), true, true);
            if(Endstops::zProbe()) {
                Com::printErrorFLN(PSTR("z-probe did not untrigger on repetitive measurement - maybe you need to increase distance!"));
                return ILLEGAL_Z_PROBE;
            }
        }
    }
    float distance = static_cast<float>(sum) * invAxisStepsPerMM[Z_AXIS] / static_cast<float>(repeat) + EEPROM::zProbeHeight();
#if Z_PROBE_Z_OFFSET_MODE == 1
    distance += EEPROM::zProbeZOffset(); // We measured including coating, so we need to add coating thickness!
#endif
#if DISTORTION_CORRECTION
    float zCorr = 0;
    if(Printer::distortion.isEnabled()) {
        zCorr = distortion.correct(currentPositionSteps[X_AXIS] + EEPROM::zProbeXOffset() * axisStepsPerMM[X_AXIS],currentPositionSteps[Y_AXIS]
                                   + EEPROM::zProbeYOffset() * axisStepsPerMM[Y_AXIS],0) * invAxisStepsPerMM[Z_AXIS];
        distance += zCorr;
    }
#endif
    distance += bendingCorrectionAt(currentPosition[X_AXIS], currentPosition[Y_AXIS]);
    Com::printF(Com::tZProbe, distance);
    Com::printF(Com::tSpaceXColon, realXPosition());
#if DISTORTION_CORRECTION
    if(Printer::distortion.isEnabled()) {
        Com::printF(Com::tSpaceYColon, realYPosition());
        Com::printFLN(PSTR(" zCorr:"), zCorr);
    } else {
        Com::printFLN(Com::tSpaceYColon, realYPosition());
    }
#else
    Com::printFLN(Com::tSpaceYColon, realYPosition());
#endif
    // Go back to start position
    PrintLine::moveRelativeDistanceInSteps(0, 0, lastCorrection - currentPositionSteps[Z_AXIS], 0, EEPROM::zProbeSpeed(), true, true);
    if(Endstops::zProbe()) {
        Com::printErrorFLN(PSTR("z-probe did not untrigger after going back to start position."));
        return ILLEGAL_Z_PROBE;
    }
    //PrintLine::moveRelativeDistanceInSteps(offx,offy,0,0,EEPROM::zProbeXYSpeed(),true,true);
    if(last)
        finishProbing();
    return distance;
}

float Printer::bendingCorrectionAt(float x, float y) {
	PlaneBuilder builder;
    builder.addPoint(EEPROM::zProbeX1(),EEPROM::zProbeY1(),EEPROM::bendingCorrectionA());
    builder.addPoint(EEPROM::zProbeX2(),EEPROM::zProbeY2(),EEPROM::bendingCorrectionB());
    builder.addPoint(EEPROM::zProbeX3(),EEPROM::zProbeY3(),EEPROM::bendingCorrectionC());
	Plane plane;
	builder.createPlane(plane,true);
	return plane.z(x,y);
}

void Printer::waitForZProbeStart() {
#if Z_PROBE_WAIT_BEFORE_TEST
    Endstops::update();
    Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    if(Endstops::zProbe()) return;
#if UI_DISPLAY_TYPE != NO_DISPLAY
    uid.setStatusP(Com::tHitZProbe);
    uid.refreshPage();
#endif
#ifdef DEBUG_PRINT
    debugWaitLoop = 3;
#endif
    while(!Endstops::zProbe()) {
        defaultLoopActions();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    }
#ifdef DEBUG_PRINT
    debugWaitLoop = 4;
#endif
    HAL::delayMilliseconds(30);
    while(Endstops::zProbe()) {
        defaultLoopActions();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    }
    HAL::delayMilliseconds(30);
    UI_CLEAR_STATUS;
#endif
}
#endif

void Printer::transformToPrinter(float x,float y,float z,float &transX,float &transY,float &transZ) {
#if FEATURE_AXISCOMP
    // Axis compensation:
    x = x + y * EEPROM::axisCompTanXY() + z * EEPROM::axisCompTanXZ();
    y = y + z * EEPROM::axisCompTanYZ();
#endif
#if BED_CORRECTION_METHOD != 1 && FEATURE_AUTOLEVEL
	if(isAutolevelActive()) {
		transX = x * autolevelTransformation[0] + y * autolevelTransformation[3] + z * autolevelTransformation[6];
		transY = x * autolevelTransformation[1] + y * autolevelTransformation[4] + z * autolevelTransformation[7];
		transZ = x * autolevelTransformation[2] + y * autolevelTransformation[5] + z * autolevelTransformation[8];
	} else {
		transX = x;
		transY = y;
		transZ = z;		
	}
#else
	transX = x;
	transY = y;
	transZ = z;
#endif	
}

void Printer::transformFromPrinter(float x,float y,float z,float &transX,float &transY,float &transZ) {
#if BED_CORRECTION_METHOD != 1 && FEATURE_AUTOLEVEL
	if(isAutolevelActive()) {
		transX = x * autolevelTransformation[0] + y * autolevelTransformation[1] + z * autolevelTransformation[2];
		transY = x * autolevelTransformation[3] + y * autolevelTransformation[4] + z * autolevelTransformation[5];
		transZ = x * autolevelTransformation[6] + y * autolevelTransformation[7] + z * autolevelTransformation[8];
	} else {
		transX = x;
		transY = y;
		transZ = z;		
	}
#else
	transX = x;
	transY = y;
	transZ = z;
#endif
#if FEATURE_AXISCOMP
    // Axis compensation:
    transY = transY - transZ * EEPROM::axisCompTanYZ();
    transX = transX - transY * EEPROM::axisCompTanXY() - transZ * EEPROM::axisCompTanXZ();
#endif
}
#if FEATURE_AUTOLEVEL
void Printer::resetTransformationMatrix(bool silent) {
    autolevelTransformation[0] = autolevelTransformation[4] = autolevelTransformation[8] = 1;
    autolevelTransformation[1] = autolevelTransformation[2] = autolevelTransformation[3] =
                                     autolevelTransformation[5] = autolevelTransformation[6] = autolevelTransformation[7] = 0;
    if(!silent)
        Com::printInfoFLN(Com::tAutolevelReset);
}

void Printer::buildTransformationMatrix(Plane &plane) {
	float z0 = plane.z(0,0);
	float az = z0-plane.z(1,0); // ax = 1, ay = 0
	float bz = z0-plane.z(0,1); // bx = 0, by = 1
    // First z direction
    autolevelTransformation[6] = -az;
    autolevelTransformation[7] = -bz;
    autolevelTransformation[8] = 1;
    float len = sqrt(az * az + bz * bz + 1);
    autolevelTransformation[6] /= len;
    autolevelTransformation[7] /= len;
    autolevelTransformation[8] /= len;
    autolevelTransformation[0] = 1;
    autolevelTransformation[1] = 0;
    autolevelTransformation[2] = -autolevelTransformation[6]/autolevelTransformation[8];
    len = sqrt(autolevelTransformation[0] * autolevelTransformation[0] + autolevelTransformation[1] * autolevelTransformation[1] + autolevelTransformation[2] * autolevelTransformation[2]);
    autolevelTransformation[0] /= len;
    autolevelTransformation[1] /= len;
    autolevelTransformation[2] /= len;
    // cross(z,x) y,z)
    autolevelTransformation[3] = autolevelTransformation[7] * autolevelTransformation[2] - autolevelTransformation[8] * autolevelTransformation[1];
    autolevelTransformation[4] = autolevelTransformation[8] * autolevelTransformation[0] - autolevelTransformation[6] * autolevelTransformation[2];
    autolevelTransformation[5] = autolevelTransformation[6] * autolevelTransformation[1] - autolevelTransformation[7] * autolevelTransformation[0];
    len = sqrt(autolevelTransformation[3] * autolevelTransformation[3] + autolevelTransformation[4] * autolevelTransformation[4] + autolevelTransformation[5] * autolevelTransformation[5]);
    autolevelTransformation[3] /= len;
    autolevelTransformation[4] /= len;
    autolevelTransformation[5] /= len;
	
    Com::printArrayFLN(Com::tTransformationMatrix,autolevelTransformation, 9, 6);
}
/* 
void Printer::buildTransformationMatrix(float h1,float h2,float h3) {
    float ax = EEPROM::zProbeX2() - EEPROM::zProbeX1();
    float ay = EEPROM::zProbeY2() - EEPROM::zProbeY1();
    float az = h1 - h2;
    float bx = EEPROM::zProbeX3() - EEPROM::zProbeX1();
    float by = EEPROM::zProbeY3() - EEPROM::zProbeY1();
    float bz = h1 - h3;
    // First z direction
    autolevelTransformation[6] = ay * bz - az * by;
    autolevelTransformation[7] = az * bx - ax * bz;
    autolevelTransformation[8] = ax * by - ay * bx;
    float len = sqrt(autolevelTransformation[6] * autolevelTransformation[6] + autolevelTransformation[7] * autolevelTransformation[7] + autolevelTransformation[8] * autolevelTransformation[8]);
    if(autolevelTransformation[8] < 0) len = -len;
    autolevelTransformation[6] /= len;
    autolevelTransformation[7] /= len;
    autolevelTransformation[8] /= len;
    autolevelTransformation[3] = 0;
    autolevelTransformation[4] = autolevelTransformation[8];
    autolevelTransformation[5] = -autolevelTransformation[7];
    // cross(y,z)
    autolevelTransformation[0] = autolevelTransformation[4] * autolevelTransformation[8] - autolevelTransformation[5] * autolevelTransformation[7];
    autolevelTransformation[1] = autolevelTransformation[5] * autolevelTransformation[6];// - autolevelTransformation[3] * autolevelTransformation[8];
    autolevelTransformation[2] = autolevelTransformation[3] * autolevelTransformation[7] - autolevelTransformation[4] * autolevelTransformation[6];
    len = sqrt(autolevelTransformation[0] * autolevelTransformation[0] + autolevelTransformation[1] * autolevelTransformation[1] + autolevelTransformation[2] * autolevelTransformation[2]);
    autolevelTransformation[0] /= len;
    autolevelTransformation[1] /= len;
    autolevelTransformation[2] /= len;
    len = sqrt(autolevelTransformation[4] * autolevelTransformation[4] + autolevelTransformation[5] * autolevelTransformation[5]);
    autolevelTransformation[4] /= len;
    autolevelTransformation[5] /= len;
    Com::printArrayFLN(Com::tTransformationMatrix,autolevelTransformation, 9, 6);
}
*/
#endif
