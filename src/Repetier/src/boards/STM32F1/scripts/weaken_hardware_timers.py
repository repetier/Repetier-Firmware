# Small py script based off a github comment (https://github.com/platformio/platform-ststm32/issues/283#issuecomment-533585597)
# marks hardware timer functions inside HardwareTimer.cpp as weak, allowing overrides.

Import('env') 
import os, re

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoststm32") + "/cores/arduino/"

if os.path.isdir(FRAMEWORK_DIR):
  for root, dirs, files in os.walk(FRAMEWORK_DIR):
    for file in files:
      if 'HardwareTimer.h' in file:
        with open(os.path.join(root, file), "r") as sources:
          lines = sources.readlines()
        with open(os.path.join(root, file), "r") as sources:
          for line in lines:
            weakenedTimers = re.search(r'.*WEAK_HARDWARE_TIMERS 1.*', line)
            if weakenedTimers: 
              break
        if not weakenedTimers: 
          with open(os.path.join(root, file), "r") as sources:
            lines = sources.readlines()
          with open(os.path.join(root, file), "w") as sources:
            for line in lines:
              sources.write(re.sub(r'.*#define  TIMER_CHANNELS(.*)', r'#define WEAK_HARDWARE_TIMERS 1\n#define  TIMER_CHANNELS\1', line)) 
    if not weakenedTimers: 
      for file in files:
        if 'HardwareTimer.cpp' in file: 
          with open(os.path.join(root, file), "r") as sources:
            lines = sources.readlines()
          with open(os.path.join(root, file), "w") as sources:
            for line in lines:
              sources.write(re.sub(r'.*void TIM(\d)_IRQHandler', r'__attribute__ ((weak)) void TIM\1_IRQHandler', line))
          with open(os.path.join(root, file), "r") as sources:
            lines = sources.readlines()
          with open(os.path.join(root, file), "w") as sources:
            for line in lines:
              sources.write(re.sub(r'.*void TIM(\d)_CC_IRQHandler', r'__attribute__ ((weak)) void TIM\1_CC_IRQHandler', line)) 
    else:
      break
