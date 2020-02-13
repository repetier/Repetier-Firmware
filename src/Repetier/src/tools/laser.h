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
    inline float getMilliWatt() { return milliWatt; }
    inline int32_t getWarmupPower() { return static_cast<int32_t>(warmupPower); }
    inline int32_t getWarmupTime() { return static_cast<int32_t>(warmup); }
    void setMilliWatt(float mw);
    void setWarmupPower(int16_t pwm);
    void setWarmupTime(int32_t time);
    void reset(float offx, float offy, float offz, float _milliwatt, int32_t _warmup, int16_t _warmupPower);
    void updateGammaMap(bool report);
    void extractNewGammaCorrection(GCode* com);
    bool supportsTemperatures() final override { return false; }
    /// Called when the tool gets activated.
    void activate() final;
    /// Gets called when the tool gets disabled.
    void deactivate() final;
    void copySettingsToMotion1() final override;
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
    virtual bool isSecondaryMove(bool isG0, bool isEMove) override final {
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
    virtual bool showMachineCoordinates() { return false; }
    static void menuToolLaserMilliWatt(GUIAction action, void* data);
    static void menuToolLaserWarmupPower(GUIAction action, void* data);
    static void menuToolLaserWarmupTime(GUIAction action, void* data);
};
