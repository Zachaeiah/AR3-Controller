#include "stepperControlls.h"

// Declare the timer objects
IntervalTimer PointTimer;

// add comment here
volatile unsigned char limit_triggered = 0x0;

// Declare and initialize the current point index
volatile int CurrentPoint = 0;


void SetupmotorLimit() {

  for (int pin = 0; pin < 6; pin++) {
    pinMode(StepperDirPins[pin], OUTPUT);
    pinMode(StepperStepsPins[pin], OUTPUT);
  }

  // setup limit switch
  for (int i = 0; i < JOINT_NUM; i++) {
    pinMode(LimitPins[i], INPUT);
    pinMode(LimitPins[i], INPUT);
  }

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LimitPins[0]), LimitSwitchISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(LimitPins[1]), LimitSwitchISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(LimitPins[2]), LimitSwitchISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(LimitPins[3]), LimitSwitchISR3, RISING);
  attachInterrupt(digitalPinToInterrupt(LimitPins[4]), LimitSwitchISR4, RISING);
  attachInterrupt(digitalPinToInterrupt(LimitPins[5]), LimitSwitchISR5, RISING);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Executes a planned motion by setting up an IntervalTimer to call the newPointISR function.
// ARGUMENTS:   strCommandLine - Not used in this function.
// RETURN VALUE: Always returns false.
bool executPlanedMove(char* strCommandLine) {

  CurrentPoint = 0;

  // Begin the IntervalTimer with newPointISR
  if (PointTimer.begin(newPointISR, 1)) {

    // Handle error condition if timer initialization fails
    print_error(BAD_TIMMER_SETUP, __func__, __LINE__, "HIGH", "could not setup PointTimer");
    return false;  // return faild
  } 

  return true;  // Indicate that the function always returns false
}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Interrupt Service Routine for processing motion points.
//              This function updates the output pins and frequencies for stepper motors based on motion points.
// ARGUMENTS:   None
// RETURN VALUE: None
void newPointISR() {
  PointTimer.end();  // Stop the timer
  // Cache frequently accessed values
  volatile int moveCnt = RobotMoshionPlan.MOVECNT;                              // Total number of motion points
  volatile int* frequencies = RobotMoshionPlan.Points[CurrentPoint].Frequency;  // Pointer to frequencies array
  volatile int currentTime = RobotMoshionPlan.Points[CurrentPoint].TIME;        // Current time

  // Check if CurrentPoint exceeds the total number of motion points
  if (CurrentPoint >= moveCnt) {
    PointTimer.end();  // End the timer if all motion points have been executed

    // Unroll the loop manually for better performance
    for (volatile int i = 0; i < 6; i++) {
      // Set duty cycle for both channels
      analogWrite(StepperStepsPins[i], 0);
      digitalWriteFast(SepperDirPins[i], 0);
    }
    // update the Moshion States machine that the moshion has finished and ready for the nest one
    MOSHIOSTATE = SETTINGUP;
    Serial.printf("MoshionState changed to: %d\n", MOSHIOSTATE);  // tell the uP that the MoshionState has changed
    free(RobotMoshionPlan.Points);
    return;
  }

  // Unroll the loop manually for better performance
  for (volatile int i = 0; i < 6; i++) {
    // Process two channels simultaneously
    volatile int frequency = frequencies[i];  // Get the Frequency channel
    volatile int step = StepperStepsPins[i];  // step Pin for current channel
    volatile int dir = SepperDirPins[i];      // Direction pin for current channel

    digitalWriteFast(dir, frequency <= 0 ? HIGH : LOW);      // Set direction based on frequency sign
    analogWriteFrequency(step, abs(frequency));              // Set frequency
    analogWrite(step, abs(frequency) <= 0 ? 0 : dutyCycle);  // Set duty cycle (if frequency is greater than 0)
  }


  PointTimer.begin(newPointISR, currentTime);  // Begin the timer for the next motion point
  // Move to the next motion point
  CurrentPoint++;
}


//---------------------------------------------------------------------------------------------------------------------
void LimitSwitchISR0() {
  // check if that switch has already been set
  if (~limit_triggered & BIT0) {
    // if not then set it
    limit_triggered |= BIT0;
  }
}
void LimitSwitchISR1() {
  if (~limit_triggered & BIT1) {
    limit_triggered |= BIT1;
  }
}
void LimitSwitchISR2() {
  if (~limit_triggered & BIT2) {
    limit_triggered |= BIT2;
  }
}
void LimitSwitchISR3() {
  if (~limit_triggered & BIT3) {
    limit_triggered |= BIT3;
  }
}
void LimitSwitchISR4() {
  if (~limit_triggered & BIT4) {
    limit_triggered |= BIT4;
  }
}
void LimitSwitchISR5() {
  if (~limit_triggered & BIT5) {
    limit_triggered |= BIT5;
  }
}
