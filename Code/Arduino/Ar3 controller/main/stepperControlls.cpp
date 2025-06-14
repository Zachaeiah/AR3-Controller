#include "core_pins.h"
#include "stepperControlls.h"

// Last known target
static StepTarget lastTarget = { { 0, 0, 0, 0, 0, 0 } };

// add comment here
volatile unsigned char limit_triggered = 0x0;

AccelStepper s1(AccelStepper::DRIVER, StepperStepsPins[0], StepperDirPins[0]);
AccelStepper s2(AccelStepper::DRIVER, StepperStepsPins[1], StepperDirPins[1]);
AccelStepper s3(AccelStepper::DRIVER, StepperStepsPins[2], StepperDirPins[2]);
AccelStepper s4(AccelStepper::DRIVER, StepperStepsPins[3], StepperDirPins[3]);
AccelStepper s5(AccelStepper::DRIVER, StepperStepsPins[4], StepperDirPins[4]);
AccelStepper s6(AccelStepper::DRIVER, StepperStepsPins[5], StepperDirPins[5]);

StepperGroup stepGroup;  // privet StepperGroup for controlling all the stepper motors
MotionQueue motionQueue;


/**
 * @brief Initializes a group of stepper motors.
 *
 * This function assigns stepper motor pointers to the StepperGroup, sets default max speed
 * and acceleration, and adds them to the MultiStepper object for synchronized control.
 *
 */
void initStepperGroup() {

  // assine the stepper motors to the stepGroup
  stepGroup.steppers[0] = &s1;
  stepGroup.steppers[1] = &s2;
  stepGroup.steppers[2] = &s3;
  stepGroup.steppers[3] = &s4;
  stepGroup.steppers[4] = &s5;
  stepGroup.steppers[5] = &s6;

  for (int i = 0; i < JOINT_NUM; i++) {
    stepGroup.steppers[i]->setAcceleration(100);         // Disable acceleration for linear timing
    stepGroup.multi.addStepper(*stepGroup.steppers[i]);  // Add stepper to the synchronized group
    pinMode(LimitPins[i], INPUT);
  }
  stepGroup.moving = false;  // Mark the group as idle

  stepGroup.steppers[0]->setMaxSpeed(10000);  // Set the maximum speed
  stepGroup.steppers[1]->setMaxSpeed(10000);  // Set the maximum speed
  stepGroup.steppers[2]->setMaxSpeed(10000);  // Set the maximum speed
  stepGroup.steppers[3]->setMaxSpeed(10000);  // Set the maximum speed
  stepGroup.steppers[4]->setMaxSpeed(10000);  // Set the maximum speed
  stepGroup.steppers[5]->setMaxSpeed(10000);  // Set the maximum speed

  // Attach interrupts for the limit switch for each stepper motor
  attachInterrupt(digitalPinToInterrupt(LimitPins[0]), LimitSwitchISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LimitPins[1]), LimitSwitchISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LimitPins[2]), LimitSwitchISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LimitPins[3]), LimitSwitchISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LimitPins[4]), LimitSwitchISR4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LimitPins[5]), LimitSwitchISR5, CHANGE);
}


/**
 * @brief Runs the MultiStepper group synchronously.
 *
 * Should be called repeatedly in a loop to update motor positions.
 *
 */
void runStepperGroup() {
  stepGroup.multi.run();
}

/**
 * @brief Checks if all steppers have arrived at their target positions.
 *
 * @return true if all motors have no distance left to move; false otherwise.
 */
bool allSteppersArrived() {
  for (int i = 0; i < 6; i++) {
    if (stepGroup.steppers[i]->distanceToGo() != 0) return false;
  }
  return true;
}

/**
 * @brief Starts a move to the specified StepTarget for all motors.
 *
 * Uses MultiStepper's moveTo() method to plan the next synchronized movement.
 *
 * @param target Pointer to the StepTarget containing the desired positions.
 */
void startMove(StepTarget* target) {
  stepGroup.multi.moveTo(target->positions);  // Queue a synchronized move
  stepGroup.moving = true;                    // Flag the group as currently moving
}
/**
 * @brief Checks whether the motion queue is empty.
 *
 * @return true if the queue is empty; false otherwise.
 */
bool isQueueEmpty() {
  return motionQueue.head == motionQueue.tail;
}

/**
 * @brief Checks whether the motion queue is full.
 *
 * @return true if the queue is full; false otherwise.
 */
bool isQueueFull() {
  return ((motionQueue.tail + 1) % MOVE_QUEUE_SIZE) == motionQueue.head;
}

/**
 * @brief Adds a new StepTarget to the motion queue.
 *
 * @param target Pointer to the StepTarget to enqueue.
 * @return true if the enqueue succeeded; false if the queue was full.
 */
bool enqueueTarget(StepTarget* target) {
  if (isQueueFull()) return false;
  motionQueue.buffer[motionQueue.tail] = *target;               // Copy the new target
  motionQueue.tail = (motionQueue.tail + 1) % MOVE_QUEUE_SIZE;  // Move tail forward
  return true;
}

/**
 * @brief Removes a StepTarget from the motion queue.
 *
 * @param outTarget Pointer to StepTarget that will receive the dequeued value.
 * @return true if the dequeue succeeded; false if the queue was empty.
 */
bool dequeueTarget(StepTarget* outTarget) {
  if (isQueueEmpty()) return false;
  *outTarget = motionQueue.buffer[motionQueue.head];            // Copy from the head
  motionQueue.head = (motionQueue.head + 1) % MOVE_QUEUE_SIZE;  // Advance head
  return true;
}


/**
 * @brief Generates a new StepTarget with each motor moved 10 steps from the last position.
 *
 * Useful for simulation and testing stepper movement.
 *
 * @return StepTarget with updated motor positions.
 */
StepTarget generateNextTarget(void) {
  StepTarget next;
  for (int i = 0; i < 6; i++) {
    lastTarget.positions[i] += 10;  // Increment each axis by 10 steps
    next.positions[i] = lastTarget.positions[i];
  }
  return next;
}

/**
 * @brief Homes all steppers by moving each to its limit switch and then to position 0.
 *
 * This function homes each stepper motor in the StepperGroup by:
 * 1. Moving rapidly toward the limit switch.
 * 2. Backing off until the switch is released.
 * 3. Slowly creeping forward to re-trigger the switch for precision.
 * 4. Setting the known home position.
 * 5. Returning to absolute position 0.
 *
 * The function includes timeouts for safety and supports direction inversion per motor.
 * It blocks until each motor completes homing.
 */
void performHoming() {
  const float FAST_HOMING_SPEED = 10000.0;  // Fast approach speed (steps/sec)
  const float SLOW_HOMING_SPEED = 30.0;     // Slow creep speed (steps/sec)
  const float BACKOFF_DISTANCE = 500.0;     // Distance to back off from limit switch
  const unsigned long TIMEOUT_MS = 10000;   // Timeout for each step (ms)
  const float MIN_ACCEL = 1.0;              // Small acceleration for precise moves

  AccelStepper* motor = stepGroup.steppers[0];
  if (motor == nullptr) {
    Serial.println("Motor pointer is null!");
    return;
  }

  motor->enableOutputs();              // Ensure driver is active
  motor->setMaxSpeed(128000);          // Set speed
  motor->setAcceleration(10000);       // Set acceleration
  motor->moveTo(128000);               // Target position

  Serial.println("Starting move...");

  while (motor->distanceToGo() != 0) {
    motor->run();
  }

  Serial.println("Move complete.");
}

//   for (int i = 0; i < 1; i++) {  // Only home the first motor (motor 0)
//     AccelStepper* motor = stepGroup.steppers[i];
//     if (motor == nullptr) {
//       Serial.printf("Error: Motor %d not initialized!\n", i);
//       continue;
//     }

//     motor->enableOutputs();  // Ensure motor driver is active

//     Serial.printf("Homing motor %d...\n", i);

//     int homingDir = (RETURN_HOME_DISTANCE[i] > 0) ? 1 : -1;
//     Serial.printf("Initial limit state: %d\n", digitalReadFast(LimitPins[i]));

//     // Step 1: Move fast toward limit switch
//     Serial.println("Step 1: Fast approach to limit switch...");
//     motor->setMaxSpeed(FAST_HOMING_SPEED);
//     motor->setAcceleration(FAST_HOMING_SPEED * 2);
//     motor->setSpeed(homingDir * FAST_HOMING_SPEED);

//     unsigned long start = millis();
//     while (!digitalReadFast(LimitPins[i])) {
//       if (millis() - start > TIMEOUT_MS) {
//         Serial.printf("Timeout on fast approach: motor %d\n", i);
//         break;
//       }
//       motor->runSpeed();
//     }

//     motor->stop();
//     while (motor->isRunning()) motor->run();
//     Serial.println("Fast approach complete.");

//     // Step 2: Back off slowly until switch is released
//     Serial.println("Step 2: Backing off from switch...");
//     motor->setSpeed(-homingDir * SLOW_HOMING_SPEED);
//     motor->move(BACKOFF_DISTANCE);

//     start = millis();
//     while (digitalReadFast(LimitPins[i]) && motor->distanceToGo() != 0) {
//       if (millis() - start > TIMEOUT_MS) {
//         Serial.printf("Timeout on backoff: motor %d\n", i);
//         break;
//       }
//       motor->run();
//     }

//     motor->stop();
//     while (motor->isRunning()) motor->run();
//     Serial.println("Backoff complete.");

//     // Step 3: Creep forward to re-trigger the switch
//     Serial.println("Step 3: Creep forward to re-trigger limit...");
//     motor->setMaxSpeed(SLOW_HOMING_SPEED);
//     motor->setAcceleration(MIN_ACCEL);

//     start = millis();
//     while (!digitalReadFast(LimitPins[i])) {
//       if (millis() - start > TIMEOUT_MS) {
//         Serial.printf("Timeout on creep: motor %d\n", i);
//         break;
//       }
//       motor->move(homingDir);  // Move one step
//       while (motor->distanceToGo() != 0) {
//         motor->run();
//       }
//     }

//     motor->stop();
//     while (motor->isRunning()) motor->run();
//     Serial.println("Creep complete.");

//     // Step 4: Set known home position
//     Serial.println("Step 4: Setting known home position...");
//     motor->setCurrentPosition(RETURN_HOME_DISTANCE[i]);
//     motor->setMaxSpeed(MOTOR_MAX_SPEED[i]);
//     motor->setAcceleration(MOTOR_ACCELERATION[i]);

//     // Step 5: Move to absolute zero
//     Serial.println("Step 5: Moving to absolute position 0...");
//     motor->moveTo(0);
//     while (motor->distanceToGo() != 0) {
//       motor->run();
//     }

//     Serial.printf("Motor %d homed successfully.\n", i);
//   }
//  }





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
