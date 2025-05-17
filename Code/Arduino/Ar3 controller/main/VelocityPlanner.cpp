#include "VelocityPlanner.h"

TrajectoryInfo_RT VI_RT;

/**
 * @brief Calculates the linear Displacement of the start and end TCP points
 * 
 * @param start The current or start point.
 * @param end The desired end point.
 * @return Lin_TCP_Dis The linear Displacement between the two points
 */
Lin_TCP_Dis LinCalculateDisplacement(const TCP_point start, const TCP_point end) {
  Lin_TCP_Dis T_Displacement = { 0 };
  T_Displacement.x_dis = end.x - start.x;
  T_Displacement.y_dis = end.y - start.y;
  T_Displacement.z_dis = end.z - start.z;
  T_Displacement.rx_dis = end.rx - start.rx;
  T_Displacement.ry_dis = end.ry - start.ry;
  T_Displacement.rz_dis = end.rz - start.rz;

  if (arm_sqrt_f32(powf(T_Displacement.x_dis, 2) + powf(T_Displacement.y_dis, 2) + powf(T_Displacement.z_dis, 2), &T_Displacement.totel_dis) != ARM_MATH_SUCCESS) {
    T_Displacement = { 0, 0, 0, 0, 0, 0, 0 };
    return T_Displacement;  // Square root calculation failed (should not happen in normal cases)
  }

  return T_Displacement;
}

/**
 * @brief Calculates the feedrate of the TCP
 * 
 * @param Displacement The total displacement of one axis (in mm).
 * @param feedRate The desired feed rate of the TCP (in mm/sec).
 * @param optimal If true, minimizes total time to reach the target; otherwise, follows `feedRate`.
 * @return the best feedrate
 */
float32_t calculateFeedRate(const float32_t Displacement, const float32_t feedRate, const bool optimal) {
  float32_t actualFeedRate = 0;
  float32_t sqrt_result = 0;
  float32_t optimal_feedRate = 0;

  // Check for invalid input (displacement and feed rate must be positive)
  if (Displacement <= 0 || feedRate <= 0) {
    return -1.0;
  }

  if (arm_sqrt_f32(6.0f * MAX_ACC * Displacement, &sqrt_result) != ARM_MATH_SUCCESS) {
    actualFeedRate = (feedRate <= MAX_VEL) ? feedRate : MAX_VEL;
    print_error(SQURT_FAILED, __func__, __LINE__, "MED", "squrt faild when calqulating optimal FeedRate, setting FeedRate to %9.4f", actualFeedRate);
    return actualFeedRate;  // Square root calculation failed (should not happen in normal cases)
  }
  optimal_feedRate = sqrt_result / 3.0f;
  optimal_feedRate = (optimal_feedRate < MAX_VEL) ? optimal_feedRate : MAX_VEL;

  if (!optimal) {
    // Use the given feed rate, ensuring it does not exceed the max velocity
    actualFeedRate = (feedRate <= optimal_feedRate) ? feedRate : optimal_feedRate;
  } else {
    // Calculate the optimal feed rate based on max acceleration
    // Ensure the optimal feed rate does not exceed max velocity
    actualFeedRate = optimal_feedRate;
  }

  return actualFeedRate;
}

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
float32_t* calculateTimeSegments(const float32_t Displacement, const float32_t feedRate) {
  float32_t actualFeedRate = 0;
  float32_t* TimeSegments = NULL;

  // Check for invalid input (displacement and feed rate must be positive)
  if (Displacement <= 0 || feedRate <= 0) {
    return NULL;
  }

  // Allocate memory for storing time segments [Acceleration, FullTime, Deceleration]
  TimeSegments = (float32_t*)malloc(3 * sizeof(float32_t));
  if (!TimeSegments) {
    return NULL;  // Memory allocation failed
  }

  // Use the given feed rate, ensuring it does not exceed the max velocity
  actualFeedRate = (feedRate <= MAX_VEL) ? feedRate : MAX_VEL;

  // Compute acceleration time based on optimal velocity
  TimeSegments[ACCELERATION] = (3.0f * actualFeedRate) / (2.0f * MAX_ACC);

  // Compute deceleration time as total time minus acceleration time
  TimeSegments[DECELERATION] = (Displacement / actualFeedRate);

  // Compute total motion time based on optimal velocity
  TimeSegments[FULL_TIME] = TimeSegments[ACCELERATION] + TimeSegments[DECELERATION];

  return TimeSegments;  // Caller must free the allocated memory
}

/**
 * @brief Generates the velocity profile for a given motion trajectory.
 * 
 * This function computes the velocity profile of the motion using the given time segments
 * for acceleration, constant velocity, and deceleration. The velocity profile is sampled
 * at fixed time intervals (`VEL_UPDATE_TIME`).
 * 
 * @param TimeSegments Pointer to the array containing time segments [Acceleration, FullTime, Deceleration].
 * @param Displacement The total displacement of the motion (in mm).
 * @param feedRate The target velocity of the TCP (in mm/sec).
 * @param VelocityLen Pointer to an size_t where the number of elements will be stored.
 * @return Pointer to an array containing the velocity profile sampled at regular intervals.
 *         Returns NULL if inputs are invalid or memory allocation fails.
 * 
 * @note The caller is responsible for freeing the allocated memory.
 */
float32_t* calculateVelocityProfile(const float32_t* TimeSegments, const float32_t feedRate, size_t* VelocityLen) {

  size_t size = 0;
  float32_t time = 0;  // Current time step in the loop
  float32_t tDecel = 0;
  float32_t actualFeedRate = 0;
  float32_t* VelocityProfile = NULL;
  float32_t VelocityCoeff[2] = { 0 };


  // Check for invalid input (displacement and feed rate must be positive)
  if (TimeSegments == NULL || feedRate <= 0) {
    return NULL;
  }

  // Calculate the number of samples for the velocity profile
  size = (size_t)(TimeSegments[FULL_TIME] / VEL_UPDATE_STIME) + 1;
  *VelocityLen = size;

  // Allocate memory for storing time segments [Acceleration, FullTime, Deceleration]
  VelocityProfile = (float32_t*)malloc(size * sizeof(float32_t));
  if (!TimeSegments) {
    return NULL;  // Memory allocation failed
  }

  // Use the given feed rate, ensuring it does not exceed the max velocity
  actualFeedRate = (feedRate <= MAX_VEL) ? feedRate : MAX_VEL;

  // Coefficients for the acceleration and deceleration phases
  VelocityCoeff[0] = (2.0 * actualFeedRate) / powf(TimeSegments[ACCELERATION], 3);
  VelocityCoeff[1] = (3.0 * actualFeedRate) / powf(TimeSegments[ACCELERATION], 2);


  // Generate the velocity profile at each time step
  for (size_t i = 0; i < size; i++) {
    time = i * VEL_UPDATE_STIME;

    // Acceleration phase: 0 ≤ time < TimeSegments[ACCELERATION]
    if (0.0f < time && time <= TimeSegments[ACCELERATION]) {
      VelocityProfile[i] = -VelocityCoeff[0] * powf(time, 3) + VelocityCoeff[1] * powf(time, 2);
    }
    // Constant velocity phase: TimeSegments[ACCELERATION] ≤ time < TimeSegments[DECELERATION]
    else if (TimeSegments[ACCELERATION] < time && time <= TimeSegments[DECELERATION]) {
      VelocityProfile[i] = actualFeedRate;
    }
    // Deceleration phase: TimeSegments[DECELERATION] ≤ time ≤ TimeSegments[FullTime]
    else if (TimeSegments[DECELERATION] < time && time <= TimeSegments[FULL_TIME]) {
      tDecel = time - TimeSegments[DECELERATION];
      VelocityProfile[i] = VelocityCoeff[0] * powf(tDecel, 3) - VelocityCoeff[1] * powf(tDecel, 2) + actualFeedRate;
    }
    // Safety check for any out-of-bounds case (should not occur)
    else {
      VelocityProfile[i] = 0;
    }
  }
  return VelocityProfile;
}

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
float32_t* calculateDisplacementProfile(const float32_t* TimeSegments, const float32_t Displacement, const float32_t feedRate, size_t* DisplacementLen) {
  size_t size = 0;
  float32_t time = 0;  // Current time step in the loop
  float32_t tDecel = 0;
  float32_t actualFeedRate = 0;
  float32_t* DisplacementProfile = NULL;
  float32_t DisplacementCoeff[4] = { 0 };


  // Check for invalid input (displacement and feed rate must be positive)
  if (TimeSegments == NULL || feedRate <= 0) {
    return NULL;
  }

  // Calculate the number of samples for the Displacement profile
  size = (size_t)(TimeSegments[FULL_TIME] / VEL_UPDATE_STIME) + 1;
  *DisplacementLen = size;

  // Allocate memory for storing time segments [Acceleration, FullTime, Deceleration]
  DisplacementProfile = (float32_t*)malloc(size * sizeof(float32_t));
  if (!TimeSegments) {
    return NULL;  // Memory allocation failed
  }

  // Use the given feed rate, ensuring it does not exceed the max velocity
  actualFeedRate = (feedRate <= MAX_VEL) ? feedRate : MAX_VEL;

  // Coefficients for the acceleration and deceleration phases
  DisplacementCoeff[0] = (4.0f * powf(MAX_ACC, 3)) / (27.0f * powf(actualFeedRate, 2));
  DisplacementCoeff[1] = (4.0f * powf(MAX_ACC, 2)) / (9.0f * actualFeedRate);
  DisplacementCoeff[2] = (TimeSegments[ACCELERATION] * actualFeedRate / 2);
  DisplacementCoeff[3] = (3.0f * powf(actualFeedRate, 2)) / (4.0f * MAX_ACC);

  // Generate the Displacement profile at each time step
  for (size_t i = 0; i < size; i++) {
    time = i * VEL_UPDATE_STIME;

    // Acceleration phase: 0 ≤ time < TimeSegments[ACCELERATION]
    if (0.0f < time && time <= TimeSegments[ACCELERATION]) {
      DisplacementProfile[i] = -DisplacementCoeff[0] * powf(time, 4) + DisplacementCoeff[1] * powf(time, 3);
    }
    // Constant velocity phase: TimeSegments[ACCELERATION] ≤ time < TimeSegments[DECELERATION]
    else if (TimeSegments[ACCELERATION] < time && time <= TimeSegments[DECELERATION]) {
      DisplacementProfile[i] = actualFeedRate * time - DisplacementCoeff[2];
    }
    // Deceleration phase: TimeSegments[DECELERATION] ≤ time ≤ TimeSegments[FullTime]
    else if (TimeSegments[DECELERATION] < time && time <= TimeSegments[FULL_TIME]) {
      tDecel = time - TimeSegments[DECELERATION];
      DisplacementProfile[i] = DisplacementCoeff[0] * powf(tDecel, 4) - DisplacementCoeff[1] * powf(tDecel, 3) + actualFeedRate * time - DisplacementCoeff[3];
    }
    // Safety check for any out-of-bounds case (should not occur)
    else {
      DisplacementProfile[i] = 0;
    }
  }
  return DisplacementProfile;
}

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
void setupTrajectory_RT(const Lin_TCP_Dis Displacements, const float32_t* TimeSegments, const float32_t feedRate) {
  float32_t actualFeedRate = (feedRate <= MAX_VEL) ? feedRate : MAX_VEL;

   // copy theTDisplacements
  VI_RT.Displacements = Displacements;

  // copy the Time Segments
  VI_RT.TimeSegments[ACCELERATION] = TimeSegments[ACCELERATION];
  VI_RT.TimeSegments[DECELERATION] = TimeSegments[DECELERATION];
  VI_RT.TimeSegments[FULL_TIME] = TimeSegments[FULL_TIME];

  //copy the feedrate
  VI_RT.feedRate = feedRate;

  // Velocity Coefficients for the acceleration and deceleration phases
  VI_RT.VelocityCoeff_RT[0] = (2.0f * actualFeedRate) / powf(TimeSegments[ACCELERATION], 3);
  VI_RT.VelocityCoeff_RT[1] = (3.0f * actualFeedRate) / powf(TimeSegments[ACCELERATION], 2);

  // Displacement Coefficients for the acceleration and deceleration phases
  VI_RT.DisplacementCoeff[0] = (4.0f * powf(MAX_ACC, 3)) / (27.0f * powf(actualFeedRate, 2));
  VI_RT.DisplacementCoeff[1] = (4.0f * powf(MAX_ACC, 2)) / (9.0f * actualFeedRate);
  VI_RT.DisplacementCoeff[2] = ((TimeSegments[ACCELERATION] * actualFeedRate) / 2);
  VI_RT.DisplacementCoeff[3] = (3.0f * powf(actualFeedRate, 2)) / (4.0f * MAX_ACC);
}

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
float32_t calculateVelocity_RT(const float32_t C_time) {
  float32_t t, t2, t3;

  if (C_time < 0.0f || C_time > VI_RT.TimeSegments[FULL_TIME]) {
    Serial.printf("domain errer in calculateVelocity_RT at %9.4f\n", C_time);
    return 0.0f;
  }

  // Acceleration phase
  if (C_time <= VI_RT.TimeSegments[ACCELERATION]) {
    t2 = C_time * C_time;
    t3 = t2 * C_time;
    return -VI_RT.VelocityCoeff_RT[0] * t3 + VI_RT.VelocityCoeff_RT[1] * t2;
  }

  // Constant velocity phase
  if (C_time <= VI_RT.TimeSegments[DECELERATION]) {
    return VI_RT.feedRate;
  }

  // Deceleration phase
  t = C_time - VI_RT.TimeSegments[DECELERATION];
  t2 = t * t;
  t3 = t2 * t;
  return VI_RT.VelocityCoeff_RT[0] * t3 - VI_RT.VelocityCoeff_RT[1] * t2 + VI_RT.feedRate;

}

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
float32_t calculateDisplacement_RT(const float32_t C_time) {
  float32_t t, t2, t3, t4;

  if ((C_time < 0.0f) || (C_time > VI_RT.TimeSegments[FULL_TIME])) {
    Serial.println("domain errer in calculateDisplacement_RT");
    return 0.0f;
  }

  // Acceleration phase
  if (C_time <= VI_RT.TimeSegments[ACCELERATION]) {
    t2 = C_time * C_time;
    t3 = t2 * C_time;
    t4 = t3 * C_time;
    return -VI_RT.DisplacementCoeff[0] * t4 + VI_RT.DisplacementCoeff[1] * t3;
  }

  // Constant velocity phase
  if (C_time <= VI_RT.TimeSegments[DECELERATION]) {
    return VI_RT.feedRate * C_time - VI_RT.DisplacementCoeff[2];
  }

  // Deceleration phase
  t = C_time - VI_RT.TimeSegments[DECELERATION];
  t2 = t * t;
  t3 = t2 * t;
  t4 = t3 * t;
  return VI_RT.DisplacementCoeff[0] * t4 - VI_RT.DisplacementCoeff[1] * t3 + VI_RT.feedRate * C_time - VI_RT.DisplacementCoeff[3];
}
