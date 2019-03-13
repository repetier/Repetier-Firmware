enum class ToolTypes {
    EXTRUDER = 0,
    SYRINGE = 1,
    LASER = 2,
    MILL = 3
};

extern HeatManager* heatedBeds[];
class Motion3;
class Motion1;
class GCode;

#define TOOL_ERROR_HEATER_DEFECT 1
#define TOOL_ERROR_HEATER_DECOUPLED 2
#define TOOL_ERROR_JAMMED_OR_NO_FILAMENT 4

extern void menuExtruderStepsPerMM(GUIAction action, void* data);
extern void menuExtruderMaxSpeed(GUIAction action, void* data);
extern void menuExtruderMaxAcceleration(GUIAction action, void* data);
extern void menuExtruderMaxYank(GUIAction action, void* data);
extern void menuExtruderFilamentDiameter(GUIAction action, void* data);

class Tool;

class ToolChangeHandler {
public:
    virtual void M6(GCode* com, Tool* tool) = 0;
};

class ToolChangeCustomEvent : ToolChangeHandler {
public:
    ToolChangeCustomEvent(Tool* tool);
    void M6(GCode* com, Tool* tool) final;
    void setup(Tool* tool);
};

class CoolantHandler {
public:
    virtual void M7(GCode* com, Tool* tool) = 0;
    virtual void M8(GCode* com, Tool* tool) = 0;
    virtual void M9(GCode* com, Tool* tool) = 0;
};

class Tool {
    friend class Motion3;
    friend class Motion1;

protected:
    float offsetX;
    float offsetY;
    float offsetZ;
    int eepromStart;
    int toolId;
    PWMHandler* secondary;        ///< Second controlled pwm device, e.g. fan
    int activeSecondaryValue;     ///< Fixed speed
    float activeSecondaryPerMMPS; ///< Speed per mm/s move speed
    fast8_t errorFlags;
    static fast8_t activeToolId;
    static Tool* activeTool;
    static Tool* const tools[NUM_TOOLS];

public:
    Tool(float offX, float offY, float offZ, PWMHandler* _secondary)
        : offsetX(offX)
        , offsetY(offY)
        , offsetZ(offZ)
        , secondary(_secondary) {
        activeSecondaryValue = 0;
        activeSecondaryPerMMPS = 0;
        toolId = -1;
        errorFlags = 0;
    }
    inline void setToolId(int t) { toolId = t; }
    inline int getToolId() { return toolId; }
    inline bool hasError() { return errorFlags != 0; }
    inline void setError(fast8_t flag) { errorFlags |= flag; }
    inline bool hasError(fast8_t flag) { return (errorFlags & flag) == flag; }
    inline void resetErrors() { errorFlags = 0; }
    inline void resetError(fast8_t flag) { errorFlags &= ~flag; }
    inline bool usesSecondary(void* sec) { return secondary == sec; }
    bool hasSecondary() { return secondary != nullptr; }
    inline void setSecondaryFixed(int sec) {
        activeSecondaryValue = sec;
        activeSecondaryPerMMPS = 0;
    }
    inline void setSecondarySpeedDependent(float sec) {
        activeSecondaryValue = 0;
        activeSecondaryPerMMPS = sec;
    }
    ///< Sends speed directly to connected pwm device
    inline void sendSecondary(int speed) {
        if (secondary != nullptr) {
            secondary->set(speed);
        }
    }
    void resetBase(float offX, float offY, float offZ);
    ///< If returning true motion planner will insert a warmup sequence
    virtual bool requiresWarmup() { return warmupTime() != 0; }
    virtual int32_t warmupTime() { return 0; }
    virtual void M3(GCode* com) {}
    virtual void M4(GCode* com) {}
    virtual void M5(GCode* com) {}
    virtual void M6(GCode* com) {}
    virtual void M7(GCode* com) {}
    virtual void M8(GCode* com) {}
    virtual void M9(GCode* com) {}
    virtual void setToolChangeHandler(ToolChangeHandler* th) {}
    virtual void setCoolantHandler(CoolantHandler* ch) {}

    /// Called when the tool gets activated.
    virtual void activate() = 0;
    /// Gets called when the tool gets disabled.
    virtual void deactivate() = 0;
    /// Called on kill/emergency to disable the tool
    virtual void shutdown() = 0;
    /// Set temperature in case tool supports temperatures.
    virtual HeatManager* getHeater() { return nullptr; }
    /// Sets intensity or similar value e.g. laser intensity or mill speed
    virtual void setIntensity(int32_t intensity) {}
    virtual bool supportsTemperatures() { return false; }
    virtual bool supportsIntensity() { return false; }
    float getOffsetX() { return offsetX; }
    float getOffsetY() { return offsetY; }
    float getOffsetZ() { return offsetZ; }
    float getOffsetForAxis(fast8_t axis);
    virtual float getMaxSpeed() { return 0; }
    virtual float getAcceleration() { return 0; }
    virtual float getMaxStartSpeed() { return 0; }
    virtual float getMaxYank() { return 0; }
    virtual float getDiameter() { return 0; }
    virtual float getMaxTemp() { return 0; }
    virtual void setDiameter(float d) {}
    virtual void retract(bool backwards, bool longRetract) {}
    void setEepromStart(int pos) {
        eepromStart = pos;
    }
    int getEepromStart() {
        return eepromStart;
    }
    virtual void eepromHandle();
    virtual int eepromSize() {
        return 12;
    }
    virtual void init() {}
    virtual void setAdvance(float adv) {}
    virtual void updateDerived();
    virtual void setResolution(float stepspermm) {}
    virtual void autocalibrate(GCode* g) {
        Com::printWarningFLN(PSTR("Autocalibration for this tool not supported!"));
    }
    virtual void disableMotor() {}
    virtual void enableMotor() {}
    virtual void stepMotor() {}
    virtual void unstepMotor() {}
    virtual void directionMotor(bool dir) {}
    virtual bool updateMotor() { return false; }
    /// Computes intensity based on speed
    virtual int computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM) { return 0; }
    /// Gets called after each move is completed
    virtual void moveFinished() {}
    /// Switch between different seconcdary states will occur. Can add a pause or warmup
    virtual void secondarySwitched(bool nowSecondary) {}
    /// Called if e.g. door is opened
    virtual void afterPause() {}
    virtual void beforeContinue() {}
    virtual bool secondaryIsFan() { return false; }
    int secondaryPercent() { return secondary ? (static_cast<int>(secondary->get()) * 100) / 255 : 0; }
    virtual void extractG1(GCode* com) {}
    virtual bool isSecondaryMove(bool isG0, bool isEMove) { return true; }
    virtual ToolTypes getToolType() { return ToolTypes::EXTRUDER; }
    inline static Tool* getActiveTool() { return activeTool; }
    inline static fast8_t getActiveToolId() { return activeToolId; }
    static void selectTool(fast8_t id, bool force = false);
    static void unselectTool();
    static inline Tool* getTool(fast8_t id) {
        if (id < 0 || id >= NUM_TOOLS) {
            return nullptr;
        }
        return tools[id];
    }
    static void initTools();
    static void eepromHandleTools();
    static void updateDerivedTools();
    static void disableMotors();
    static void enableMotors();
    static void pauseHeaters();
    static void unpauseHeaters();
    static void waitForTemperatures();
    static void minMaxOffsetForAxis(fast8_t axis, float& min, float& max);
};

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
    bool supportsTemperatures() final { return true; }
    /// Called when the tool gets activated.
    void activate() final;
    /// Gets called when the tool gets disabled.
    void deactivate() final;
    /// Called on kill/emergency to disable the tool
    void shutdown() final;
    /// Set temperature in case tool supports temperatures.
    HeatManager* getHeater() final { return heater; }
    float getMaxSpeed() { return maxSpeed; }
    void setMaxSpeed(float speed) { maxSpeed = speed; }
    float getAcceleration() { return acceleration; }
    void setAcceleration(float accel) { acceleration = accel; }
    float getMaxStartSpeed() { return 0.5 * yank; }
    float getMaxYank() { return yank; }
    void setMaxYank(float y) { yank = y; }
    float getDiameter() { return diameter; }
    void setDiameter(float dia) { diameter = dia; }
    float getMaxTemp() { return heater->getMaxTemperature(); }
    void eepromHandle();
    void init();
    void setAdvance(float adv);
    void updateDerived();
    void disableMotor() final;
    void enableMotor() final;
    void stepMotor() final;
    void unstepMotor() final;
    bool updateMotor() final;
    void directionMotor(bool dir) final;
    void setResolution(float stepspermm) { stepsPerMM = stepspermm; }
    float getResolution() { return stepsPerMM; }
    void retract(bool backwards, bool longRetract) {
        // TODO: Add retract handling
    }
    /// Computes intensity based on speed
    virtual int computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM) { return activeSecondaryValue; }
    virtual bool secondaryIsFan() { return true; }
    virtual ToolTypes getToolType() { return ToolTypes::EXTRUDER; }
};

/** Controls a laser by adjusting the PWM frequency to an output depending
 * on laser power P [W = J/s] at 100% and planned intensity I [J/mm].
 * Power required is then Preq = I * v
 * Scaled to available power it is Ppercent = 100 * Preq/P
 */
template <class enabledPin, class activePin>
class ToolLaser : public Tool {
    PGM_P startScript;
    PGM_P endScript;
    float milliWatt;
    float scalePower;
    int32_t warmup;
    int16_t warmupPower;
    float bias;
    float gamma;
    uint8_t gammaMap[256];
    bool active;
    bool powderMode;

public:
    ToolLaser(float offX, float offY, float offZ,
              PWMHandler* pwm,
              float milliWatt,
              int32_t warmupTime,
              int16_t _warmupPower,
              float _bias,
              float _gamma,
              PGM_P _startScript,
              PGM_P _endScript)
        : Tool(offX, offY, offZ, pwm)
        , startScript(_startScript)
        , endScript(_endScript)
        , milliWatt(milliWatt)
        , warmup(warmupTime)
        , warmupPower(_warmupPower)
        , bias(_bias)
        , gamma(_gamma) {
        updateGammaMap(false);
        scalePower = 255.0 / milliWatt;
        active = false;
        powderMode = false;
    }
    void reset(float offx, float offy, float offz, float _milliwatt, int32_t _warmup, int16_t _warmupPower);
    void updateGammaMap(bool report);
    void extractNewGammaCorrection(GCode* com);
    bool supportsTemperatures() final { return false; }
    /// Called when the tool gets activated.
    void activate() final;
    /// Gets called when the tool gets disabled.
    void deactivate() final;
    /// Called on kill/emergency to disable the tool
    void shutdown() final;
    float getMaxSpeed() { return 200; }
    float getAcceleration() { return 10000.0f; }
    float getMaxStartSpeed() { return 200.0f; }
    float getMaxYank() { return 200.0f; }
    float getDiameter() { return 1.0f; }
    float getMaxTemp() { return 0; }
    void eepromHandle();
    void init();
    void setAdvance(float adv) {}
    void updateDerived();
    void retract(bool backwards, bool longRetract) {}
    /// Computes intensity based on speed
    virtual int computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM);
    /// Gets called after each move is completed
    virtual void moveFinished();
    virtual bool isSecondaryMove(bool isG0, bool isEMove) {
        if (isG0 || !active) {
            return false;
        }
        if (powderMode) {
            return isEMove;
        }
        return true;
    }
    /// Switch between different seconcdary states will occur. Can add a pause or warmup
    virtual void secondarySwitched(bool nowSecondary);
    virtual void extractG1(GCode* com);
    virtual void M3(GCode* com);
    virtual void M4(GCode* com);
    virtual void M5(GCode* com);
    virtual ToolTypes getToolType() { return ToolTypes::LASER; }
};

template <class dirPin, class enabledPin, class activePin>
class ToolCNC : public Tool {
    PGM_P startScript;
    PGM_P endScript;
    float rpm;
    int32_t startStopDelay;
    bool active;
    ToolChangeHandler* toolChangeHandler;
    CoolantHandler* coolantHandler;

public:
    ToolCNC(float offX, float offY, float offZ,
            PWMHandler* pwm,
            float _rpm,
            int32_t _startStopDelay,
            PGM_P _startScript,
            PGM_P _endScript)
        : Tool(offX, offY, offZ, pwm)
        , startScript(_startScript)
        , endScript(_endScript)
        , rpm(_rpm)
        , startStopDelay(_startStopDelay)
        , toolChangeHandler(nullptr)
        , coolantHandler(nullptr) {
    }
    void setToolChangeHandler(ToolChangeHandler* th) final {
        toolChangeHandler = th;
    }
    void setCoolantHandler(CoolantHandler* ch) final {
        coolantHandler = ch;
    }
    void reset(float offx, float offy, float offz, float _rpm, int32_t _startStopDelay);
    bool supportsTemperatures() final { return false; }
    /// Called when the tool gets activated.
    void activate() final;
    /// Gets called when the tool gets disabled.
    void deactivate() final;
    /// Called on kill/emergency to disable the tool
    void shutdown() final;
    float getMaxSpeed() { return 200; }
    float getAcceleration() { return 10000.0f; }
    float getMaxStartSpeed() { return 200.0f; }
    float getMaxYank() { return 200.0f; }
    float getDiameter() { return 1.0f; }
    float getMaxTemp() { return 0; }
    void eepromHandle();
    void init();
    void setAdvance(float adv) {}
    void updateDerived();
    void retract(bool backwards, bool longRetract) {}
    int computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM) { return intensity; }
    /// Gets called after each move is completed
    /// Switch between different seconcdary states will occur. Can add a pause or warmup
    virtual void M3(GCode* com);
    virtual void M4(GCode* com);
    virtual void M5(GCode* com);
    virtual void M6(GCode* com);
    virtual void M7(GCode* com);
    virtual void M8(GCode* com);
    virtual void M9(GCode* com);
    virtual ToolTypes getToolType() { return ToolTypes::MILL; }
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