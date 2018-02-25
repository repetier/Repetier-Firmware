enum ToolTypes {
    EXTRUDER = 0,
    SYRINGE = 1,
    LASER = 2,
    MILL = 3
};

extern HeatManager* heatedBeds[];

class Tool {
    float offsetX;
    float offsetY;
    float offsetZ;
    int eepromStart;

    static fast8_t activeToolId;
    static Tool* activeTool;
    static Tool* tools[];

public:
    Tool(float offX, float offY, float offZ)
        : offsetX(offX)
        , offsetY(offY)
        , offsetZ(offZ) {}
    /// Called when the tool gets activated.
    virtual void activate() = 0;
    /// Gets called when the tool gets disabled.
    virtual void deactivate() = 0;
    /// Called on kill/emergency to disable the tool
    virtual void shutdown() = 0;
    /// Set temperature in case tool supports temperatures.
    virtual HeatManager* getHeater() { return nullptr; }
    /// Sets intensity or similar value e.g. laser intensity or mill speed
    virtual void setIntensity(int32_t intensite) {}
    virtual bool supportsTemperatures() { return false; }
    virtual bool supportsIntensity() { return false; }
    virtual float getOffsetX() { return offsetX; }
    virtual float getOffsetY() { return offsetY; }
    virtual float getOffsetZ() { return offsetZ; }
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
    inline static Tool* getActiveTool() { return activeTool; }
    inline static fast8_t getActiveToolId() { return activeToolId; }
    static void selectTool(fast8_t id);
    static Tool* getTool(fast8_t id);
    static void initTools();
    static void eepromHandleTools();
    static void updateDerivedTools();
    static void disableMotors();    
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
                 PGM_P _endScript)
        : Tool(offX, offY, offZ)
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
    bool supportsTemperatures() final { return true; }
    /// Called when the tool gets activated.
    void activate();
    /// Gets called when the tool gets disabled.
    void deactivate();
    /// Called on kill/emergency to disable the tool
    void shutdown();
    /// Set temperature in case tool supports temperatures.
    HeatManager* getHeater() final { return heater; }
    float getMaxSpeed() { return maxSpeed; }
    float getAcceleration() { return acceleration; }
    float getMaxStartSpeed() { return 0.5 * yank; }
    float getMaxYank() { return yank; }
    float getDiameter() { return diameter; }
    float getMaxTemp() { return heater->getMaxTemperature(); }
    void eepromHandle();
    void init();
    void setAdvance(float adv);
    void updateDerived();
    void disableMotor();
    void setResolution(float stepspermm) { stepsPerMM = stepspermm; }
    void retract(bool backwards, bool longRetract) {
        // TODO: Add retract handling
    }
    void setDiameter(float d) { diameter = d; }
    /// Sets intensity or similar value e.g. laser intensity or mill speed
};
