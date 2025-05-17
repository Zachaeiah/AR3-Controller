#include <stddef.h>
#ifndef STEPPERCONTROLLS_H
#define STEPPERCONTROLLS_H

//---------------------------- include library ------------------------------------------------------------------------
#include "global.h"

//---------------------------- Program Constants ----------------------------------------------------------------------

#define DUTYCYCLE_HALF 127

const int LimitPins[JOINT_NUM] = { 23, 22, 21, 20, 19, 18 };
const int StepperStepsPins[] = {  0, 1, 2, 4,  5,  6};
const int StepperDirPins[] =    { 26, 7, 8, 9, 24, 25};

//---------------------------- Structure Definitions ------------------------------------------------------------------

// Structure to hold a motion point with joint interpolations
typedef struct POINT_INTERP {
  float32_t Frequency[6];  // Array of frequencies 
  size_t TIME;          // Time at which the frequencies are applied
} POINT_INTERP;

//---------------------------- Program Globals ----------------------------------------------------------------------
extern volatile unsigned char limit_triggered;
extern volatile int CurrentPoint;  // Current motion point index
extern IntervalTimer PointTimer;   // Declare the timer object globally

// Enumeration for Moshion States machine
enum MoshionStates { SETTING_UP,
                     READY,
                     IN_MOSHION,
                     MOSHION_ERROR };

//----------------------------- Function Prototypes -------------------------------------------------------------------
/**
 * @brief set up motor driver fields
 */
void SetupmotorLimit();

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief TBD
 *
 * @param Link
 * @param Homeing
 *
 */
void HomeMotor(unsigned char Link, bool Homeing);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief TBD
 *
 * @param Link
 * @param Homeing
 *
 */
void CorceHome(unsigned char Link, bool Homeing);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief TBD
 *
 * @param Link
 * @param Homeing
 *
 */
void FineHome(unsigned char Link, bool Homeing);




//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Executes a planned motion by setting up an IntervalTimer to call the newPointISR function.
 *
 * @return TBD
 *
 */
bool executPlanedMove();

// Define the ISR function
void newPointISR();

/**
 * @brief ISR funshion calls to trigger the limits switches
 */
void LimitSwitchISR0();
void LimitSwitchISR1();
void LimitSwitchISR2();
void LimitSwitchISR3();
void LimitSwitchISR4();
void LimitSwitchISR5();

void stepper_isr_0();
void stepper_isr_1();
void stepper_isr_2();
void stepper_isr_3();
void stepper_isr_4();
void stepper_isr_5();


#endif