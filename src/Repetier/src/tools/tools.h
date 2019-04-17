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
extern void menuToolOffsetX(GUIAction action, void* data);
extern void menuToolOffsetY(GUIAction action, void* data);
extern void menuToolOffsetZ(GUIAction action, void* data);

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
    void setOffsetForAxis(fast8_t axis, float offset);
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
    virtual bool showMachineCoordinates() { return true; }

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

#include "extruder.h"
#include "laser.h"
#include "cnc.h"
