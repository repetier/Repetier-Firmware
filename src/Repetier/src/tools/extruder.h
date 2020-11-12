
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
*/

/** Defines a simple extruder with one motor.
 * It is ok if several extruders share same HeatManager.
 */
class ToolExtruder : public Tool {
    PGM_P startScript;
    PGM_P endScript;
    HeatManager* heater;
    StepperDriverBase* stepper;
    float stepsPerMM;
    float yank;
    float maxSpeed;
    float acceleration;
    float advance;
    float diameter;

public:
    ToolExtruder(float offX, float offY, float offZ,
                 HeatManager* heat,
                 StepperDriverBase* _stepper,
                 float dia,
                 float resolution,
                 float _yank,
                 float _maxSpeed,
                 float _acceleration,
                 float _advance,
                 PGM_P _startScript,
                 PGM_P _endScript,
                 PWMHandler* fan)
        : Tool(offX, offY, offZ, fan)
        , startScript(_startScript)
        , endScript(_endScript)
        , heater(heat)
        , stepper(_stepper)
        , stepsPerMM(resolution)
        , yank(_yank)
        , maxSpeed(_maxSpeed)
        , acceleration(_acceleration)
        , advance(_advance)
        , diameter(dia) {}
    void reset(float offx, float offy, float offz, float diameter, float resolution, float yank, float maxSpeed, float acceleration, float advance);
    virtual bool supportsTemperatures() final override { return true; }
    /// Called when the tool gets activated.
    virtual void activate() final override;
    /// Gets called when the tool gets disabled.
    virtual void deactivate() final override;
    virtual void copySettingsToMotion1() override final;
    /// Called on kill/emergency to disable the tool
    virtual void shutdown() final override;
    /// Set temperature in case tool supports temperatures.
    virtual HeatManager* getHeater() final override { return heater; }
    virtual float getMaxSpeed() override { return maxSpeed; }
    void setMaxSpeed(float speed) { maxSpeed = speed; }
    float getAcceleration() { return acceleration; }
    void setAcceleration(float accel) { acceleration = accel; }
    float getMaxStartSpeed() { return 0.5 * yank; }
    float getMaxYank() { return yank; }
    void setMaxYank(float y) { yank = y; }
    virtual float getDiameter() override final { return diameter; }
    void setDiameter(float dia) { diameter = dia; }
    virtual float getMaxTemp() final override { return heater->getMaxTemperature(); }
    virtual void eepromHandle() final override;
    virtual void init() final override;
    virtual void setAdvance(float adv) final override;
    virtual void updateDerived() final override;
    virtual void disableMotor() final override;
    virtual void enableMotor() final override;
    virtual void stepMotor() final override;
    virtual void unstepMotor() final override;
    virtual bool updateMotor() final override;
    void directionMotor(bool dir) final;
    void setResolution(float stepspermm) { stepsPerMM = stepspermm; }
    float getResolution() { return stepsPerMM; }
    void retract(bool backwards, bool longRetract);
    /// Computes intensity based on speed
    virtual int computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM) { return intensity; }
    virtual bool secondaryIsFan() final override { return true; }
    virtual bool isSecondaryMove(bool isG0, bool isEMove) final override { return (!isG0 || isEMove); }
    virtual ToolTypes getToolType() override { return ToolTypes::EXTRUDER; }
};

template <class inputPin, class ObserverType>
class JamDetectorHW {
    int eepromStart;
    uint32_t lastSignal;
    int32_t distanceSteps;
    int32_t errorSteps;
    int32_t jitterSteps;
    int32_t jamPercentage;
    Tool* tool;
    ObserverType* observer;

public:
    JamDetectorHW(ObserverType* _observer, Tool* _tool, int32_t distanceSteps, int32_t jitterSteps, int32_t jamPercentage);
    void reset(int32_t distanceSteps, int32_t jitterSteps, int32_t jamPercentage);
    void eepromHandle();
    void interruptSignaled();
    void testForJam();
    inline int32_t getDistanceSteps() { return distanceSteps; }
    inline int32_t getJitterSteps() { return jitterSteps; }
    inline int32_t getJamPercentage() { return jamPercentage; }
    static void menuDistanceSteps(GUIAction action, void* data);
    static void menuJitterSteps(GUIAction action, void* data);
    static void menuJamPercentage(GUIAction action, void* data);
};

template <class inputPin>
class FilamentDetector {
    Tool* tool;
    millis_t lastFound;

public:
    FilamentDetector(Tool* tool);
    void testFilament();
    void setup();
};
