#include "Repetier.h"

// BASE --
void BeeperSourceBase::muteAll(bool set) {
    for (size_t i = 0; i < NUM_BEEPERS; i++) {
        beepers[i]->mute(set);
    }
}
bool BeeperSourceBase::pushTone(const TonePacket packet) {
    if (isMuted() || isBlocking() || !packet.duration) {
        return false;
    }
    if (++toneHead >= beepBufSize) {
        toneHead = 0;
    }
    beepBuf[toneHead] = packet;
    if (!isPlaying()) {
        prevToneTime = HAL::timeInMilliseconds();
        toneTail = 0;
        playingFreq = packet.frequency;
        InterruptProtectedBlock noInts;
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

inline void BeeperSourceBase::finishPlaying() {
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
        if (curValidCondition != lastValidCondition
            || (curValidCondition != nullptr && (curValidCondition->loop || curValidCondition->plays > 0))) {
            if (lastValidCondition != nullptr) {
                lastValidCondition->started = false;
            }
            if (isPlaying() && curValidCondition != lastValidCondition) {
                if (isBlocking() || (curConditionStep > 0)) {
                    // If playing a blocking theme or about to play a new condition's theme, kill any beeps.
                    finishPlaying();
                }
            }
            if (curValidCondition != nullptr) {
                if (curConditionStep >= lastConditionStep || !curValidCondition->heard || curValidCondition->loop) {
                    if (!curValidCondition->started) { // Just started
                        curValidCondition->started = true;
                        if (curValidCondition->loop) {
                            if (getHeadDist() <= (beepBufSize / 2)) {
                                curValidCondition->plays = 1;
                            }
                        } else if (curValidCondition->plays <= 0) { // Only reset to playcount if we're empty
                            curValidCondition->plays = curValidCondition->playCount;
                        }
                    }
                    if (curValidCondition->plays > 0) {
                        blocking = false; // Unblock in case we're finishing filling the buffer
                        while (curValidCondition->plays
                               && getHeadDist() < (beepBufSize - curValidCondition->theme->getSize())) {
                            playTheme(*(curValidCondition->theme), (curValidCondition->plays == 1)); // set blocking once done
                            curValidCondition->plays--;
                        }
                    }
                }

                if (curValidCondition->plays <= 0) { // Only mark as heard if the buffer is done filling
                    curValidCondition->heard = true;
                } else {
                    blocking = true; // Block since we're still not done
                    curValidCondition->heard = false;
                }
            }
        }
        lastValidCondition = curValidCondition;
        curValidCondition = nullptr;
        lastConditionStep = curConditionStep > 0 ? curConditionStep : -1;
        curConditionStep = 0;
    }
}

void BeeperSourceBase::setCondition(bool set, ToneCondition& condition) {
    if (set) {
        curValidCondition = &condition;
        curConditionStep++;
    } else {
        // We can skip the hierarchy check if we've never been heard before
        condition.heard = condition.started = false;
        condition.plays = 0;
    }
}
// BASE --

// PWM --
void BeeperSourcePWM::refreshBeepFreq() {
    if (playingFreq > 0) {
        pwmPin->setFreq(playingFreq);
        pwmPin->set((Printer::toneVolume * 127) / 100);
        halted = false;
    } else { // Turn off and just wait if we have no frequency.
        halted = true;
        pwmPin->set(0);
    }
}
// PWM --
// IO --
template <class IOPin>
void BeeperSourceIO<IOPin>::refreshBeepFreq() {
    InterruptProtectedBlock noInts;
    if (playingFreq > 0) {
        halted = false;
        freqDiv = 0;
        HAL::tone(playingFreq);
    } else { // Turn off and just wait if we have no frequency.
        halted = true;
        HAL::noTone();
        IOPin::set((freqCnt = 0));
    }
}
template <class IOPin>
void BeeperSourceIO<IOPin>::finishPlaying() {
    InterruptProtectedBlock noInts;
    BeeperSourceBase::finishPlaying();
    HAL::noTone();
    IOPin::off();
    freqDiv = freqCnt = 0;
}
// IO --

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
