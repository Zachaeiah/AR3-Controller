#ifndef GLOBLE_H
#define GLOBLE_H

//---------------------------- include library ------------------------------------------------------------------------

#include <Arduino.h>
#include <arm_math.h>  // ARM CMSIS DSP library for mathematical operations
#include <math.h>      // Standard math library
#include <SD.h>        // SD card library
#include "core_pins.h"
#include <stdbool.h>  // bool definitions
#include <stdarg.h>
#include <cfloat>  // character functions

//---------------------------- Program Constants ----------------------------------------------------------------------

extern File dataFile;                   // Declare the external File object
const int chipSelect = BUILTIN_SDCARD;  // where to stor the log file

#define VEL_UPDATE_STIME 0.01f  // Time interval for velocity updates (in seconds)
#define VEL_UPDATE_UTIME 10000 // Time interval for velocity updates (in microseconds)
#define VEL_UPDATE_freq 100 // Frequency in Hz to update stepper motor stepping Frequency

#define CMD_BUFFER_SIZE 1024
#define baudRate 115200
#define JOINT_NUM 6

// Link 1 parameters
#define JOINT1_MAX 1.96706f    // Maximum joint angle in radians (170°)
#define JOINT1_MIN -1.96706f   // Minimum joint angle in radians (-170°)
#define JOINT1_SPR 600000l     // steps per rev
#define JOINT1_MAX_VEL 128000  // Max Velcity (steps / sec)
#define JOINT1_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

// Link 2 parameters
#define JOINT2_MAX HALF_PI     // Maximum joint angle in radians (90°)
#define JOINT2_MIN -0.733038f  // Minimum joint angle in radians (-42°)
#define JOINT2_SPR 600000l     // steps per rev
#define JOINT2_MAX_VEL 125000  // Max Velcity (steps / sec)
#define JOINT2_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

// Link 3 parameters
#define JOINT3_MAX 0.90751f    // Maximum joint angle in radians (52°)
#define JOINT3_MIN -1.55334f   // Minimum joint angle in radians (-89°)
#define JOINT3_SPR 600000l     // steps per rev
#define JOINT3_MAX_VEL 125000  // Max Velcity (steps / sec)
#define JOINT3_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

// Link 4 parameters
#define JOINT4_MAX 2.87979f    // Maximum joint angle in radians (165°)
#define JOINT4_MIN -2.87979f   // Minimum joint angle in radians (-165°)
#define JOINT4_SPR 501760l     // steps per rev
#define JOINT4_MAX_VEL 125440  // Max Velcity (steps / sec)
#define JOINT4_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

// Link 5 parameters
#define JOINT5_MAX 1.8326f     // Maximum joint angle in radians (105°)
#define JOINT5_MIN -1.8326f    // Minimum joint angle in radians (-105°)
#define JOINT5_SPR 603185l     // steps per rev
#define JOINT5_MAX_VEL 150796  // Max Velcity (steps / sec)
#define JOINT5_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

// Link 6 parameters
#define JOINT6_MAX 2.70526f    // Maximum joint angle in radians (155°)
#define JOINT6_MIN -2.70526f   // Minimum joint angle in radians (-155°)
#define JOINT6_SPR 486400l     // steps per rev
#define JOINT6_MAX_VEL 121600  // Max Velcity (steps / sec)
#define JOINT6_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

// Wild Link 7 parameters
#define W_JOINT7_MAX HALF_PI    // Maximum joint angle in radians (155°)
#define W_JOINT7_MIN -HALF_PIf   // Minimum joint angle in radians (-155°)
#define W_JOINT7_SPR 12800l     // steps per rev
#define W_JOINT7_MAX_VEL 3750  // Max Velcity (steps / sec)
#define W_JOINT7_MAX_ACC 1000    // Max Acceleration (steps / sec^2)

//
enum Links {
  JOINT1,
  JOINT2,
  JOINT3,
  JOINT4,
  JOINT5,
  JOINT6
};

// Error codes
enum Error_codes {
  NO_ERROR,
  QUEUE_FULL,
  COMMAND_FAILED,
  UNKNOWN_COMMAND,
  MMULT_FAILED,
  MINV_FAILD, // inversion if the input matrix is singular and cannot be inverted
  SQURT_FAILED,
  TRIG_RANGE,
  iS_NAN,
  FK_FAILE,
  IK_SOL_FAIL,
  MEMORY_ALLOCATION_FAILD,
  MISSING_COMMAND,
  BAD_TIMMER_SETUP,
  NULL_KSATE,
  UNKMOWN_KSATE_VALUE
};

// Precomputed bit masks for bits 0 to 27
#define BIT0 0x01  // (1 << 0)
#define BIT1 0x02  // (1 << 1)
#define BIT2 0x04  // (1 << 2)
#define BIT3 0x08  // (1 << 3)
#define BIT4 0x10  // (1 << 4)
#define BIT5 0x20  // (1 << 5)
#define BIT6 0x40  // (1 << 6)
#define BIT7 0x80  // (1 << 7)

//======================================== Structure Definitions ====================================

/**
 * @brief Represents a 6-DOF tool center point (TCP) position and orientation.
 */
typedef struct TCP_point {
  float32_t x;   // X-coordinate in mm
  float32_t y;   // Y-coordinate in mm
  float32_t z;   // Z-coordinate in mm
  float32_t rx;  // Rotation about X-axis in radians
  float32_t ry;  // Rotation about Y-axis in radians
  float32_t rz;  // Rotation about Z-axis in radians
} TCP_point;

//----------------------------- Function Prototypes -------------------------------------------------------------------
/**
 * @brief Prints a formatted error message to the serial console and log file.
 *
 * This function formats and prints an error message with detailed information 
 * including the function where the error occurred, the line number, the severity 
 * of the error, and additional user-supplied error details. The message is printed 
 * to the serial console and, if a valid log file is open, written to the file as well.
 *
 * @param error_index - The index representing the specific error in a predefined 
 * @param funcError - The name of the function (obtained using `__func__`)
 * @param line - The line number in the code (typically supplied by `__LINE__`)
 * @param severity - The severity of the error message, such as "INFO", "WARNING", 
 *                   or "ERROR".
 * @param strError - A string containing the error message format,
 * @param ... - Additional arguments for the formatted error message, matching the 
 *              placeholders in `strError`.
 */
void print_error(int error_index, const char* funcError, int line, const char* severity, const char* strError, ...);

/**
 * @brief Prints a formatted string to the serial console and logs it to a file if the file is open.
 *
 * This function allows formatted strings to be printed to both the serial console and, 
 * if a valid log file (`dataFile`) is open, written to the file. It also includes a 
 * persistent call count that is appended to each logged message to help track the 
 * order of log entries.
 *
 * @param fmt - The format string used for the message, similar to `printf`. 
 * @param ... - Additional arguments that will replace the placeholders in `fmt`.
 * @return int - The length of the formatted string that was printed, 
 */
int dsprintf(const char* fmt, ...);




#endif