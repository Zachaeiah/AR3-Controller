// === stepper_control.h ===
#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

//---------------------------- include library ------------------------------------------------------------------------
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "global.h"

//---------------------------- Program Constants ----------------------------------------------------------------------

#define MOVE_QUEUE_SIZE 16

const int LimitPins[JOINT_NUM] = { 23, 22, 21, 20, 19, 18 };
const int StepperStepsPins[JOINT_NUM] = {  0, 1, 2, 4,  5,  6};
const int StepperDirPins[JOINT_NUM] =    { 26, 7, 8, 9, 24, 25};

//---------------------------- Structure Definitions ------------------------------------------------------------------

/**
 * @brief Structure for holding a single motion target.
 *
 * Represents the absolute step positions for each joint/motor.
 */
typedef struct StepTarget{
    long positions[JOINT_NUM]; ///< Absolute step positions for each joint
} StepTarget;

/**
 * @brief Circular queue to hold upcoming motion targets.
 *
 * Used to store and manage a list of StepTarget items in a FIFO manner
 * for sequential execution by the motion planner.
 */
typedef struct MotionQueue{
    StepTarget buffer[MOVE_QUEUE_SIZE]; ///< Circular buffer for queued motion targets
    int head; ///< Index of the first item in the queue
    int tail; ///< Index where the next item will be inserted
} MotionQueue;

/**
 * @brief Represents a group of stepper motors managed as a synchronized system.
 *
 * Contains motor pointers, a MultiStepper instance for coordinated moves,
 * and a flag indicating whether a move is in progress.
 */
typedef struct StepperGroup{
    AccelStepper* steppers[JOINT_NUM]; // Pointers to individual motor drivers
    MultiStepper multi;                // MultiStepper instance for synchronized control
    bool moving;                       // Flag indicating whether a move is currently running
} StepperGroup;

//---------------------------- Program Globals ----------------------------------------------------------------------
extern volatile unsigned char limit_triggered;
extern MotionQueue StepperQueue;

//----------------------------- Function Prototypes -------------------------------------------------------------------

/**
 * @brief Initializes a group of stepper motors.
 *
 * This function assigns stepper motor pointers to the StepperGroup, sets default max speed
 * and acceleration, and adds them to the MultiStepper object for synchronized control.
 *
 * @param group Pointer to the StepperGroup structure.
 * @param steppers Array of pointers to AccelStepper objects (length must be 6).
 */
void initStepperGroup();

/**
 * @brief Runs the MultiStepper group synchronously.
 *
 * Should be called repeatedly in a loop to update motor positions.
 *
 * @param group Pointer to the StepperGroup.
 */
void runStepperGroup();

/**
 * @brief Checks if all steppers have arrived at their target positions.
 *
 * @param group Pointer to the StepperGroup.
 * @return true if all motors have no distance left to move; false otherwise.
 */
bool allSteppersArrived();

/**
 * @brief Starts a move to the specified StepTarget for all motors.
 *
 * Uses MultiStepper's moveTo() method to plan the next synchronized movement.
 *
 * @param target Pointer to the StepTarget containing the desired positions.
 */
void startMove(StepTarget* target);

/**
 * @brief Checks whether the motion queue is empty.
 *
 * @param q Pointer to the MotionQueue.
 * @return true if the queue is empty; false otherwise.
 */
bool isQueueEmpty();

/**
 * @brief Checks whether the motion queue is full.
 *
 * @param q Pointer to the MotionQueue.
 * @return true if the queue is full; false otherwise.
 */
bool isQueueFull();

/**
 * @brief Adds a new StepTarget to the motion queue.
 *
 * @param target Pointer to the StepTarget to enqueue.
 * @return true if the enqueue succeeded; false if the queue was full.
 */
bool enqueueTarget(StepTarget* target);

/**
 * @brief Removes a StepTarget from the motion queue.
 *
 * @param outTarget Pointer to StepTarget that will receive the dequeued value.
 * @return true if the dequeue succeeded; false if the queue was empty.
 */
bool dequeueTarget(StepTarget* outTarget);

/**
 * @brief Generates a new StepTarget with each motor moved 10 steps from the last position.
 *
 * Useful for simulation and testing stepper movement.
 *
 * @return StepTarget with updated motor positions.
 */
StepTarget generateNextTarget(void);


/**
 * @brief Homes all steppers by moving each to its limit switch and then to position 0.
 *
 * Each stepper in the StepperGroup is driven one at a time. First it runs in the
 * negative direction (toward its limit) until the corresponding bit in `limit_triggered`
 * becomes 1 (switch hit). The motor is then stopped and commanded to move in the
 * positive direction back to absolute position 0. We repeatedly call `runSpeed()` and
 * `run()` to step the motors (as required by AccelStepper). Note: this routine
 * blocks until each motor finishes homing. The array index i (0..5) corresponds to
 * BIT0..BIT5 of `limit_triggered`, and `group.steppers[i]` accesses the i‑th motor.
 *
 */
void performHoming();


/**
 * @brief ISR funshion calls to trigger the limits switches
 */
void LimitSwitchISR0();
void LimitSwitchISR1();
void LimitSwitchISR2();
void LimitSwitchISR3();
void LimitSwitchISR4();
void LimitSwitchISR5();


#endif