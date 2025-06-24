#include <Arduino.h>
#include <TeensyTimerTool.h>  // Library to simplify using FlexTimers on Teensy

#define NUM_STEPPERS 6
#define MAX_SEGMENTS 1024
#define MOTION_UPDATE_INTERVAL_MS 30  // Timer 7 update rate

// Define step and direction pins for each stepper motor
const char StepperStepsPins[NUM_STEPPERS] = { 0, 1, 2, 4, 5, 6 };
const char SepperDirPins[NUM_STEPPERS] = { 26, 7, 8, 9, 24, 25 };

StepperMotor steppers[NUM_STEPPERS];
MotionSegment motion_buffer[MAX_SEGMENTS];
uint32_t shared_step_interval_us = 800;  // Set globally per segment
volatile uint16_t current_segment_index = 0;

typedef struct {
  uint16_t steps[NUM_STEPPERS];
} MotionSegment;


typedef struct {
  char step_pin;
  char dir_pin;
  volatile uint32_t step_interval_us;    // Time between steps (shared for all steppers per segment)
  volatile int32_t steps_remaining;      // Steps left to do
  volatile int32_t position;             // Current step position
  volatile bool direction;               // true = forward
  void char moveing;                     // motor controll bit for Bitmask
  void (*start_timer)();                 // Function to start the timer
  void (*stop_timer)();                  // Function to stop the timer
  void (*set_timer_interval)(uint32_t);  // Function to set timer match interval
} StepperMotor;

// === FORWARD DECLARATIONS FOR TIMER FUNCTIONS (These must be defined for each motor) ===
void make_timer_functions();
void motion_update_isr();

// Arduino setup()
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_STEPPERS; i++) {
    pinMode(StepperStepsPins[i], OUTPUT);
    pinMode(SepperDirPins[i], OUTPUT);

    StepperMotor[i].step_pin = StepperStepsPins[i];
    StepperMotor[i].dir_pin = SepperDirPins[i];
    steppers[i].position = 0;
    steppers[i].steps_remaining = 0;
    steppers[i].direction = true;
  }

  make_timer_functions();

  // Setup motion update timer (Timer 7)
  IntervalTimer motionTimer;
  motionTimer.begin(motion_update_isr, MOTION_UPDATE_INTERVAL_MS * 1000);  // in microseconds
}

// Arduino loop(): runs motion sequences
void loop() {
  // idle - real work is done in ISRs

  // while (true)
  //   ;  // End of program
}

// === MOCK TIMER FUNCTIONS (Replace with real FTM/GPT setup on Teensy 4.1) ===
// For now, we simulate with IntervalTimers (less efficient)
IntervalTimer motorTimers[NUM_STEPPERS];

void make_timer_functions() {
  for (int i = 0; i < NUM_STEPPERS; ++i) {
    steppers[i].start_timer = [i]() {
      motorTimers[i].begin([i]() {
        stepper_isr_handler(i);
      },
                           shared_step_interval_us);
    };

    steppers[i].stop_timer = [i]() {
      motorTimers[i].end();
    };

    steppers[i].set_timer_interval = [i](uint32_t us) {
      motorTimers[i].update(us);
    };
  }
}

// === MOTION UPDATE ISR ===
void motion_update_isr() {
  // This should load new step counts, direction, and step interval for each stepper
  // Example stub: set dummy motion profile

  for (int i = 0; i < NUM_STEPPERS; ++i) {
    StepperMotor *m = &steppers[i];

    m->direction = toggle;
    digitalWrite(m->dir_pin, m->direction);

    m->step_interval_us = 800 + i * 100;  // Different speed per motor
    m->steps_remaining = 200 + i * 20;

    m->set_timer_interval(m->step_interval_us);
    m->start_timer();
  }
}

// This ISR is triggered by FlexTimer periodically
void stepper_isr_handler(int index) {
  static bool step_pin_state = false;

  StepperMotor *m = &steppers[index];

  if (m->steps_remaining <= 0) {
    m->stop_timer();
    return;
  }

  step_pin_state = step_pin_state;
  digitalWriteFast(m->step_pin, step_pin_state);

  if (step_pin_state) {
    m->steps_remaining--;
    m->position += m->direction ? 1 : -1;
  }
}
