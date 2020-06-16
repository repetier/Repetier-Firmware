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

*/

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef TONE_THEME
#undef TONE_THEME_COND
#undef BEEPER_SOURCE_IO
#undef BEEPER_SOURCE_PWM
#undef TONES // helper macro for init lists

#if IO_TARGET == IO_TARGET_CLASS_DEFINITION

constexpr fast8_t beepBufSize = 50;

class ToneTheme;
struct TonePacket {
    //If the frequency is 0, it'll behave as putting a G4 P(duration) command in between M300's.
    uint16_t frequency;
    uint16_t duration;
};

class ToneTheme {
private:
    const TonePacket* savedTheme;
    const fast8_t themeSize;

public:
    template <fast8_t size>
    ToneTheme(const TonePacket (&theme)[size])
        : savedTheme(theme)
        , themeSize(size) {
    }
    const TonePacket getTone(fast8_t index) {
        TonePacket beep;
        memcpy_P(&beep, &savedTheme[index], sizeof(TonePacket));
        return beep;
    }
    const inline fast8_t getSize() { return themeSize; }
};

class BeeperSourceBase {
protected:
    virtual void refreshBeepFreq() = 0;
    volatile bool playing;
    volatile bool halted; // Special state between beeps/duration only beeps.
    bool muted;           // eeprom etc
    fast8_t blockNewFor;
    fast8_t toneHead;
    fast8_t toneTail;
    millis_t prevToneTime;
    uint16_t playingFreq;
    TonePacket beepBuf[beepBufSize] {};

public:
    fast8_t condLastValidIndex;
    fast8_t condValidIndex;
    fast8_t condNumPlays;

    BeeperSourceBase()
        : playing(false)
        , halted(false)
        , muted(false)
        , blockNewFor(0)
        , toneHead(-1)
        , toneTail(-1)
        , prevToneTime(0)
        , playingFreq(0)
        , condLastValidIndex(0)
        , condValidIndex(0)
        , condNumPlays(0) {}
    virtual inline fast8_t getHeadDist() {
        return !isPlaying() ? 0 : (toneHead >= toneTail ? (toneHead - toneTail) : (beepBufSize - toneTail + toneHead));
    }
    virtual inline ufast8_t getOutputType() { return 0; };
    virtual inline uint16_t getCurFreq() { return playingFreq; }
    virtual inline volatile bool isPlaying() { return playing; }
    virtual inline volatile bool isHalted() { return halted; }
    virtual inline bool isMuted() { return muted; }
    virtual inline bool isBlocking() { return (blockNewFor != 0); }
    virtual inline bool mute(bool set) {
        if (set && isPlaying()) {
            finishPlaying();
        }
        return (muted = set);
    }

    virtual void setFreqDiv(ufast8_t div) = 0;
    virtual ufast8_t getFreqDiv() = 0;

    virtual bool pushTone(const TonePacket packet) {
        if (isMuted() || isBlocking() || !packet.duration) {
            return false;
        }

        if (++toneHead >= beepBufSize) {
            toneHead = 0;
        }
        beepBuf[toneHead] = packet;
        if (!isPlaying()) {
            InterruptProtectedBlock noInts;
            prevToneTime = HAL::timeInMilliseconds();
            toneTail = 0;
            playingFreq = packet.frequency;
            playing = true;
            refreshBeepFreq();
        }
        return true;
    }

    virtual inline fast8_t process() {
        if (!isMuted()) {
            if (isPlaying()) {
                millis_t curTime = HAL::timeInMilliseconds();
                if ((curTime - prevToneTime) >= beepBuf[toneTail].duration) {
                    if (getHeadDist()) {
                        if (++toneTail >= beepBufSize) {
                            toneTail = 0;
                        }
                        prevToneTime = curTime;
                        playingFreq = beepBuf[toneTail].frequency;
                        --blockNewFor;
                        refreshBeepFreq();
                    } else {
                        finishPlaying();
                    }
                }
            }
            // handling for later macros
            if (condLastValidIndex != condValidIndex) {
                condNumPlays = 0;
                condLastValidIndex = condValidIndex;
            }
            condValidIndex = 0;
        }
        return toneTail;
    }
    virtual inline void finishPlaying() {
        playing = halted = false;
        toneHead = toneTail = -1;
        playingFreq = blockNewFor = 0;
    }
    virtual bool playTheme(ToneTheme& theme, bool block) {
        if (!isMuted() && !isBlocking()) {
            fast8_t headTailDif = getHeadDist();
            for (fast8_t i = 0; i < theme.getSize(); i++) {
                pushTone(theme.getTone(i));
            }
            if (block) {
                blockNewFor = theme.getSize() + headTailDif;
            }
            return true;
        }
        return false;
    }
};

template <class IOPin>
class BeeperSourceIO : public BeeperSourceBase {
private:
    volatile bool pinState;
    volatile ufast8_t freqDiv;

public:
    volatile uint8_t freqCnt;
    BeeperSourceIO()
        : BeeperSourceBase()
        , pinState(false)
        , freqDiv(0)
        , freqCnt(0) {
        IOPin::off();
    };
    virtual inline ufast8_t getOutputType() final {
        return 1;
    }
    virtual inline ufast8_t getFreqDiv() final {
        return freqDiv;
    }
    virtual inline void setFreqDiv(ufast8_t div) final {
        freqDiv = div;
    }
    virtual void refreshBeepFreq() final {
        InterruptProtectedBlock noInts;
        if (playingFreq > 0) {
            halted = false;
            freqDiv = 0;
            HAL::tone(playingFreq);
        } else { // Turn off and just wait if we have no frequency.
            HAL::noTone();
            IOPin::set(pinState = (freqCnt = 0));
            halted = true;
        }
    }
    virtual void finishPlaying() final {
        InterruptProtectedBlock noInts;
        BeeperSourceBase::finishPlaying();
        HAL::noTone();
        IOPin::off();
        freqDiv = freqCnt = 0;
    }
    inline void toggle() {
        freqCnt = 0;
        IOPin::set(pinState = !pinState);
    }
};

class BeeperSourcePWM : public BeeperSourceBase {
private:
    PWMHandler* pwmPin;

public:
    BeeperSourcePWM(PWMHandler* pwm)
        : BeeperSourceBase()
        , pwmPin(pwm) {
        pwmPin->set(0);
    };
    virtual inline ufast8_t getOutputType() final { return 2; }
    virtual inline ufast8_t getFreqDiv() final { return 0; }
    virtual inline void setFreqDiv(ufast8_t div) final {}
    virtual inline void refreshBeepFreq() final {
        if (playingFreq) {
            pwmPin->set(255 / 2);
            pwmPin->setFreq(playingFreq);
            halted = false;
        } else {
            halted = true;
            pwmPin->set(0);
        }
    }
    virtual inline void finishPlaying() final {
        pwmPin->set(0);
        BeeperSourceBase::finishPlaying();
    }
};

#define PLAY_THEME(source, theme, blocking) \
    source.playTheme(theme, blocking);

#define BEEPER_SOURCE_IO(name, IOPin) \
    extern BeeperSourceIO<IOPin> name;

#define BEEPER_SOURCE_PWM(name, PWMPin) \
    extern BeeperSourcePWM name;

#define TONE_THEME(name, theme) \
    extern ToneTheme name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES

#define TONES(...) \
    { __VA_ARGS__ }

#define BEEPER_SOURCE_IO(name, IOPin) \
    BeeperSourceIO<IOPin> name; \
    static_assert(NUM_BEEPERS, "\"" #name "\" created in config_io but NUM_BEEPERS is zero!");

#define BEEPER_SOURCE_PWM(name, PWMPin) \
    BeeperSourcePWM name(&PWMPin); \
    static_assert(NUM_BEEPERS, "\"" #name "\" created in config_io but NUM_BEEPERS is zero!");

#define TONE_THEME(name, theme) \
    constexpr TonePacket name##_theme[] PROGMEM = theme; \
    ToneTheme name(name##_theme);

#elif IO_TARGET == IO_TARGET_PERIODICAL_ACTIONS

#define BEEPER_SOURCE_IO(name, IOPin) \
    name.process();

#define BEEPER_SOURCE_PWM(name, PWMPin) \
    name.process();

#define TONE_THEME_COND(source, cond, theme, playTimes) \
    if (cond && !source.isMuted()) { \
        source.condValidIndex++; \
        if (!source.isBlocking() && source.condValidIndex == source.condLastValidIndex) { \
            if (!playTimes || source.condNumPlays < playTimes) { \
                source.playTheme(theme, true); \
                if (playTimes) { \
                    source.condNumPlays++; \
                } \
            } \
        } \
    }

#elif IO_TARGET == IO_TARGET_BEEPER_LOOP

#define BEEPER_SOURCE_IO(name, IOPin) \
    if (name.isPlaying() && !name.isHalted() && name.freqCnt++ >= name.getFreqDiv()) { \
        name.toggle(); \
    }

#endif

#ifndef PLAY_THEME
#define PLAY_THEME(source, theme, blocking)
#endif

#ifndef TONE_THEME_COND
#define TONE_THEME_COND(source, cond, theme, playTimes)
#endif

#ifndef TONE_THEME
#define TONE_THEME(name, theme)
#endif

#ifndef BEEPER_SOURCE_IO
#define BEEPER_SOURCE_IO(name, IOPin)
#endif

#ifndef BEEPER_SOURCE_PWM
#define BEEPER_SOURCE_PWM(name, PWMPin)
#endif
