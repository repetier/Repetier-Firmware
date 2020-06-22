#include "Repetier.h"

// BASE --
bool BeeperSourceBase::pushTone(const TonePacket packet) {
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

fast8_t BeeperSourceBase::process() {
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
                    refreshBeepFreq();
                } else {
                    finishPlaying();
                }
            }
        }
    }
    return toneTail;
}

void BeeperSourceBase::finishPlaying() {
    playing = halted = blocking = false;
    toneHead = toneTail = -1;
    playingFreq = 0;
}

bool BeeperSourceBase::playTheme(ToneTheme& theme, bool block) {
    if (!isMuted() && !isBlocking()) {
        for (fast8_t i = 0; i < theme.getSize(); i++) {
            pushTone(theme.getTone(i));
        }
        if (block) {
            blocking = true; // Block until finishPlaying
        }
        return true;
    }
    return false;
}

void BeeperSourceBase::runConditions() {
    if (!isMuted()) {
        if (lastCondTheme != finalCondTheme || (finalCondPlays == 0)) { // Different theme or looping
            if (isPlaying() && (lastCondTheme != finalCondTheme)) {
                if (isBlocking() || (curConditionStep > 0)) {
                    // If playing a blocking theme or about to play a new condition's theme, kill any beeps.
                    finishPlaying();
                }
            }
            if (finalCondTheme != nullptr) {
                if ((lastConditionStep < curConditionStep) || (finalCondPlays == 0)) {
                    // Unless looping, don't replay any lower hierarchy condition themes
                    if (finalCondPlays == 0) {                    // Looping
                        if (getHeadDist() <= (beepBufSize / 2)) { // don't overwrite ourselves if the buffer wraps.
                            blocking = false;
                            finalCondPlays = 1; // Keep adding theme
                        }
                    }
                    while (finalCondPlays--) {
                        playTheme(*finalCondTheme, (finalCondPlays == 0)); // set blocking once done
                    }
                }
            }
            lastCondTheme = finalCondTheme;
        }
        lastConditionStep = curConditionStep;
        curConditionStep = 0;
        finalCondTheme = nullptr; // Forces finishPlaying check if no valid condition on next reset
    }
}

void BeeperSourceBase::setCondition(ToneTheme& theme, fast8_t playTimes) {
    curConditionStep++;
    finalCondPlays = playTimes; // If set 0, it's a loop
    finalCondTheme = &theme;
}
// BASE --

// PWM --
void BeeperSourcePWM::refreshBeepFreq() {
    if (playingFreq > 0) {
        pwmPin->setFreq(playingFreq);
        pwmPin->set(255 / 2);
        halted = false;
    } else { // Turn off and just wait if we have no frequency.
        halted = true;
        pwmPin->set(0);
    }
}
// PWM --
