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

template <class dirPin, class enabledPin, class activePin>
class ToolCNC : public Tool {
    PGM_P startScript;
    PGM_P endScript;
    float rpm;
    int32_t startStopDelay;
    bool active;

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
        , startStopDelay(_startStopDelay) {
    }
    virtual bool isSecondaryMove(bool isG0, bool isEMove) override final {
        if (isG0 || !active) {
            return false;
        }
        return true;
    }

    void reset(float offx, float offy, float offz, float _rpm, int32_t _startStopDelay);
    bool supportsTemperatures() final { return false; }
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
    float getRPM() { return rpm; }
    void setRPM(float newRPM);
    void eepromHandle();
    void init();
    void setAdvance(float adv) { }
    void updateDerived();
    void retract(bool backwards, bool longRetract) { }
    int computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM) { return intensity; }
    /// Gets called after each move is completed
    /// Switch between different seconcdary states will occur. Can add a pause or warmup
    virtual void M3(GCode* com);
    virtual void M4(GCode* com);
    virtual void M5(GCode* com);
    virtual ToolTypes getToolType() { return ToolTypes::MILL; }
    virtual bool showMachineCoordinates() { return false; }
    static void menuRPM(GUIAction action, void* data);
};
