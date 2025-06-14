#include <Arduino.h>
#include <TeensyTimerTool.h>  // Library to simplify using FlexTimers on Teensy

using namespace TeensyTimerTool;

#define NUM_STEPPERS 6

// Define step and direction pins for each stepper motor
const int stepPins[NUM_STEPPERS] = { 0, 1, 2, 4, 5, 6 };
const int dirPins[NUM_STEPPERS] = { 22, 23, 24, 25, 26, 27 };

enum Direction { DIR_FORWARD = 1,
                 DIR_BACKWARD = 0 };

// Data structure to hold each stepper motor's movement state
struct stepperInfo {
  uint32_t steps_remaining;
  bool direction;
  uint32_t step_interval_us;  // microseconds between steps
  volatile bool step_pin_state;
};

volatile stepperInfo steppers[NUM_STEPPERS];  // One per motor
volatile byte remainingSteppersFlag = 0;      // Bitmask: which motors are still moving

// Trigger a step pulse
void doStep(int index) {
  digitalWriteFast(stepPins[index], HIGH);
  digitalWriteFast(stepPins[index], LOW);
}

// Set direction pin
void setDirection(int index, int dir) {
  digitalWriteFast(dirPins[index], dir);
}

// Reset key internal values (for motion planning)
void resetStepperInfo(stepperInfo& si) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.rampUpStepTime = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

// Reset movement profile for new move
void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;  // Basic formula assumes 1st interval = accel
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
  si.speedScale = 1;

  // Estimate number of steps needed to reach full speed
  float a = si.minStepInterval / (float)si.c0;
  a *= 0.676;  // Empirical tuning factor
  float m = ((a * a - 1) / (-2 * a));
  float n = m * m;
  si.estStepsToSpeed = n;
}

// Used to calculate how long ramp-up or ramp-down will take
float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {
  float d = s.c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);  // From stepper acceleration equations
    totalDuration += d;
  }
  return totalDuration;
}

// Initialize stepper movement for a specific motor
void prepareMovement(int whichMotor, long steps) {
  volatile stepperInfo& si = steppers[whichMotor];

  // Set motor direction and internal sign
  setDirection(whichMotor, steps < 0 ? HIGH : LOW);
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);

  resetStepper(si);

  remainingSteppersFlag |= (1 << whichMotor);  // Mark this motor as active

  unsigned long stepsAbs = abs(steps);

  // Estimate total move time, accounting for ramp-up/down phases
  if ((2 * si.estStepsToSpeed) < stepsAbs) {
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
    si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
  } else {
    float accelDecelTime = getDurationOfAcceleration(si, stepsAbs / 2);
    si.estTimeForMove = 2 * accelDecelTime;
  }
}

// Scale all stepper speeds so they finish at the same time
void adjustSpeedScales() {
  float maxTime = 0;

  // Find the longest move time
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ((1 << i) & remainingSteppersFlag) {
      if (steppers[i].estTimeForMove > maxTime)
        maxTime = steppers[i].estTimeForMove;
    }
  }

  // Set speed scale factors accordingly
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ((1 << i) & remainingSteppersFlag) {
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}

// Pick the motor(s) with the shortest delay and schedule the timer
void setNextInterruptInterval() {
  unsigned long mind = 0xFFFFFFFF;
  nextStepperFlag = 0;

  // Find the minimum delay interval
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ((1 << i) & remainingSteppersFlag && steppers[i].di < mind) {
      mind = steppers[i].di;
    }
  }

  // Mark all motors that match this interval
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ((1 << i) & remainingSteppersFlag && steppers[i].di == mind) {
      nextStepperFlag |= (1 << i);
    }
  }

  if (remainingSteppersFlag == 0) {
    controlTimer.stop();  // Stop timer when all moves complete
    return;
  }

  controlTimer.setPeriod(mind);  // Set timer for shortest delay
}

// This ISR is triggered by FlexTimer periodically
void stepperISR() {
  unsigned int interval = controlTimer.getPeriod();

  for (int i = 0; i < NUM_STEPPERS; i++) {
    if (!((1 << i) & remainingSteppersFlag))
      continue;

    // If not ready to step yet, decrement delay counter
    if (!(nextStepperFlag & (1 << i))) {
      steppers[i].di -= interval;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    // Do step and update position
    if (s.stepCount < s.totalSteps) {
      doStep(i);
      s.stepCount++;
      s.stepPosition += s.dir;
      if (s.stepCount >= s.totalSteps) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);  // Clear bit: this motor done
      }
    }

    // Acceleration phase
    if (s.rampUpStepCount == 0) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);  // Decrease delay (faster)
      if (s.d <= s.minStepInterval) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if (s.stepCount >= s.totalSteps / 2) {
        s.rampUpStepCount = s.stepCount;
      }
      s.rampUpStepTime += s.d;
    }
    // Deceleration phase
    else if (s.stepCount >= s.totalSteps - s.rampUpStepCount) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);  // Increase delay (slower)
      s.n--;
    }

    s.di = s.d * s.speedScale;  // Scale step delay for synchronization
  }

  setNextInterruptInterval();  // Reschedule timer
}

// Main execution loop: sync start and wait until all moves complete
void runAndWait() {
  adjustSpeedScales();                  // Sync all stepper speeds
  setNextInterruptInterval();           // Choose timer interval
  controlTimer.begin(stepperISR, 100);  // Start timer with placeholder interval
  while (remainingSteppersFlag)
    ;  // Wait until all stepper moves are done
}

// Arduino setup()
void setup() {
  for (int i = 0; i < NUM_STEPPERS; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    steppers[i].acceleration = 1000;   // Default accel (adjust as needed)
    steppers[i].minStepInterval = 50;  // Min step interval in µs (max speed)
  }
}

// Arduino loop(): runs motion sequences
void loop() {
  // Move each stepper one at a time
  for (int i = 0; i < NUM_STEPPERS; i++) {
    prepareMovement(i, 800);
    runAndWait();
  }

  // Move all steppers together with different distances
  prepareMovement(0, 8000);
  prepareMovement(1, 800);
  prepareMovement(2, 2400);
  prepareMovement(3, 800);
  prepareMovement(4, 1600);
  prepareMovement(5, 800);
  runAndWait();

  delay(1000);

  // Move all back in reverse
  prepareMovement(0, -8000);
  prepareMovement(1, 1600);
  prepareMovement(2, -2400);
  prepareMovement(3, -800);
  prepareMovement(4, 2400);
  prepareMovement(5, -800);
  runAndWait();

  while (true)
    ;  // End of program
}
