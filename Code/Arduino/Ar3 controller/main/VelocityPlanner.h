#ifndef VELOCITYPLANNER_H
#define VELOCITYPLANNER_H
#include "global.h"
//============================ Program Constants ======================================================================
#define MAX_ACC 10.0f  // TCP (mm/sec^2)
#define MAX_VEL 20.0f  // TCP (mm/sec)

enum TimeSegmentIndices { ACCELERATION,
                          DECELERATION,
                          FULL_TIME };

enum DisplacementIndex { X_DIS,
                         Y_DIS,
                         Z_DIS,
                         RX_DIS,
                         RY_DIS,
                         RZ_DIS,
                         TOTEL_DIS };

//---------------------------- Structure Definitions ------------------------------------------------------------------

/**
 * @brief hold all linear Displacements between two points
 */
typedef struct Lin_TCP_Dis {
  TCP_point start;
  TCP_point end;
  float32_t x_dis;      // X-coordinate Displacementin mm
  float32_t y_dis;      // Y-coordinate Displacementin mm
  float32_t z_dis;      // Z-coordinate Displacementin mm
  float32_t rx_dis;     // Rotation Displacement about X-axis in radians
  float32_t ry_dis;     // Rotation Displacement about Y-axis in radians
  float32_t rz_dis;     // Rotation Displacement about Z-axis in radians
  float32_t totel_dis;  // totel linear Displacement
} Lin_TCP_Dis;

/**
 * @brief Structure containing parameters for real-time velocity profiling.
 *
 * This structure holds the coefficients used for acceleration and deceleration phases,
 * the motion time segments (acceleration, cruise, deceleration), and the target feed rate.
 */
typedef struct TrajectoryInfo_RT {
  float32_t VelocityCoeff_RT[2];   // Coefficients for velocity profile [accel, decel]
  float32_t DisplacementCoeff[4];  // Coefficients for Displacement profile [accel, decel]
  float32_t TimeSegments[3];       // Pointer to array: [Acceleration, Deceleration, FullTime]
  float32_t feedRate;              // Target linear velocity (mm/s)
  Lin_TCP_Dis Displacements;

} TrajectoryInfo_RT;


//======================================== Globals ==================================================

extern TrajectoryInfo_RT VI_RT;

//======================================== Function Prototypes ======================================

/**
 * @brief Calculates the linear Displacement of the start and end TCP points
 * 
 * @param start The current or start point.
 * @param end The desired end point.
 * @return Lin_TCP_Dis The linear Displacement between the two points
 */
Lin_TCP_Dis LinCalculateDisplacement(const TCP_point start, const TCP_point end);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Calculates the feedrate of the TCp
 * 
 * @param Displacement The total displacement of one axis (in mm).
 * @param feedRate The desired feed rate of the TCP (in mm/sec).
 * @param optimal If true, minimizes total time to reach the target; otherwise, follows `feedRate`.
 * @return the best feedrate
 */
float32_t calculateFeedRate(const float32_t Displacement, const float32_t feedRate, const bool optimal);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Calculates the three primary time segments of a 7-segment velocity profile.
 * 
 * This function determines the acceleration, full motion, and deceleration times 
 * for a given displacement and feed rate.
 * 
 * @param Displacement The total displacement of one axis (in mm).
 * @param feedRate The desired feed rate of the TCP (in mm/sec).
 * @return Pointer to an array containing three time segments: [Acceleration, FullTime, Deceleration].
 *         Returns NULL if invalid inputs are given or memory allocation fails.
 * 
 * @note The caller is responsible for freeing the allocated memory.
 */
float32_t* calculateTimeSegments(const float32_t Displacement, const float32_t feedRate);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Generates the velocity profile for a given motion trajectory.
 * 
 * This function computes the velocity profile of the motion using the given time segments
 * for acceleration, constant velocity, and deceleration. The velocity profile is sampled
 * at fixed time intervals (`VEL_UPDATE_TIME`).
 * 
 * @param TimeSegments Pointer to the array containing time segments [Acceleration, FullTime, Deceleration].
 * @param feedRate The target velocity of the TCP (mm/sec).
 * @param VelocityLen Pointer to an size_t where the number of elements will be stored.
 * @return Pointer to an array containing the velocity profile sampled at regular intervals.
 *         Returns NULL if inputs are invalid or memory allocation fails.
 * 
 * @note The caller is responsible for freeing the allocated memory.
 */
float32_t* calculateVelocityProfile(const float32_t* TimeSegments, const float32_t feedRate, size_t* VelocityLen);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Generates the Displacement profile for a given motion trajectory.
 * 
 * This function computes the Displacement profile of the motion using the given time segments
 * for acceleration, constant velocity, and deceleration. The Displacement profile is sampled
 * at fixed time intervals (`VEL_UPDATE_TIME`).
 * 
 * @param TimeSegments Pointer to the array containing time segments [Acceleration, FullTime, Deceleration].
 * @param Displacement The total displacement of the motion (in mm).
 * @param feedRate The target Displacement of the TCP (in mm/sec).
 * @param DisplacementLen Pointer to an size_t where the number of elements will be stored.
 * @return Pointer to an array containing the Displacement profile sampled at regular intervals.
 *         Returns NULL if inputs are invalid or memory allocation fails.
 * 
 * @note The caller is responsible for freeing the allocated memory.
 */
float32_t* calculateDisplacementProfile(const float32_t* TimeSegments, const float32_t Displacement, const float32_t feedRate, size_t* VelocityLen);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Initializes the real-time velocity profile parameters.
 *
 * Computes the velocity polynomial coefficients for the acceleration and deceleration phases
 * based on the provided motion segment durations and target feed rate. The feed rate is clamped
 * to MAX_VEL if necessary.
 * 
 * @param Displacements has all the axial Displacements
 * @param TimeSegments Pointer to time segments array: [Acceleration, Deceleration, FullTime]
 * @param feedRate Desired target velocity (mm/s)
 */
void setupTrajectory_RT(const Lin_TCP_Dis Displacements, const float32_t* TimeSegments, const float32_t feedRate);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Computes velocity at a given time based on the velocity profile.
 *
 * Returns the instantaneous velocity for the current time within the motion profile. This includes:
 * - Cubic acceleration phase
 * - Constant velocity phase
 * - Cubic deceleration phase
 *
 * @param C_time Current time within the motion profile (seconds)
 * @return Instantaneous velocity (mm/s)
 */
float32_t calculateVelocity_RT(const float32_t C_time);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Computes Displacement at a given time based on the displacement profile.
 *
 * Returns the displacement for the current time within the motion profile. This includes:
 * - Cubic acceleration phase
 * - Constant velocity phase
 * - Cubic deceleration phase
 *
 * @param C_time Current time within the motion profile (seconds)
 * @return Displacement (m/s)
 */
float32_t calculateDisplacement_RT(const float32_t C_time);


#endif