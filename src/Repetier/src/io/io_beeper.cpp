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

void BeeperSourceBase::finishPlaying() {
    playing = halted = false;
    toneHead = toneTail = -1;
    playingFreq = blockNewFor = 0;
}

bool BeeperSourceBase::playTheme(ToneTheme& theme, bool block) {
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
// BASE --

// PWM -- 
void BeeperSourcePWM::refreshBeepFreq() {
    if (playingFreq) {
        pwmPin->set(255 / 2);
        pwmPin->setFreq(playingFreq);
        halted = false;
    } else {
        halted = true;
        pwmPin->set(0);
    }
}
// PWM --