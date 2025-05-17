//---------------------------- include library ------------------------------------------------------------------------

#include "global.h"
#include "queue.h"
#include "shell.h"
#include "kinamatics.h"
#include "stepperControlls.h"
#include "VelocityPlanner.h"
#include "motionPlanner.h"
#define ITERATIONS 100000

#define DAMPING_FACTOR 0.001f  // You can tune this
arm_status arm_mat_damped_inverse_f32(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst);
bool pseudoInverse6x6(const arm_matrix_instance_f32 *jacobian, arm_matrix_instance_f32 *pseudoInverse);
arm_status svd_pseudo_inverse_f32(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst);

void setup() {
  // Initialize Serial communication
  Serial.begin(baudRate);
  while (!Serial)
    ;  // Wait for Serial Monitor (for debugging)

  // Flush Serial input
  while (Serial.available()) {
    Serial.read();
  }

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  // Check if the log file exists; if not, create it
  if (!SD.exists("log.txt")) {
    Serial.println("File does not exist. Creating...");
    dataFile = SD.open("log.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Log file created.");
      Serial.println("File successfully created.");
    } else {
      Serial.println("Failed to create file!");
      return;
    }
  } else {
    // Open the existing file for writing
    dataFile = SD.open("log.txt", FILE_WRITE);
  }

  // Initialize motor limit settings
  SetupmotorLimit();

  // Print a message indicating the Arduino Shell is ready
  // dsprintf("Arduino Shell Ready\n");
}



void loop() {

  FORWARD_SOLUTION resultFK;
  INVERSE_SOLUTION resultIK;
  // Define End-Effector target position
  TCP_point End_effector = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
  set_tool_frame(&End_effector);

  // Initialize Jacobian matrix
  arm_matrix_instance_f32 jacobian, INV_jacobian, TCP_vel, TCP_angle_vel;
  float32_t TCP_feedRate = 0;
  float32_t feedRate = 10;
  float32_t *TimeSegments = NULL;
  ;

  float32_t jacobian_data[6 * 6] = { 0 };
  float32_t INV_jacobian_data[6 * 6] = { 0 };
  float32_t TCP_vel_data[6] = { 0 };
  float32_t TCP_angle_vel_data[6] = { 0 };

  bool optimal = false;

  arm_mat_init_f32(&jacobian, 6, 6, jacobian_data);
  arm_mat_init_f32(&INV_jacobian, 6, 6, INV_jacobian_data);
  arm_mat_init_f32(&TCP_vel, 6, 1, TCP_vel_data);
  arm_mat_init_f32(&TCP_angle_vel, 6, 1, TCP_angle_vel_data);

  // Compute Inverse Kinematics for point
  TCP_point point1 = { 323.080f, 10.0f, 464.770f, -2.35619f, 1.57081f, -2.35619f };

  TCP_point point2 = { 343.080f, -10.0f, 484.770f, -2.35619f, 1.57081f, -2.35619f };

  TCP_point next_TCP_point;
  TCP_point current_TCP_point;


  Lin_TCP_Dis TCP_Displacements = LinCalculateDisplacement(point1, point2);
  printf("TCP Displacements: %9.4f\n", TCP_Displacements.totel_dis);

  TCP_feedRate = calculateFeedRate(TCP_Displacements.totel_dis, feedRate, optimal);
  printf("TCP Feed Rate : %9.4f\n", TCP_feedRate);

  TimeSegments = calculateTimeSegments(TCP_Displacements.totel_dis, TCP_feedRate);
  printf("ACCELERATION time:     0.0 - %9.4f\n", TimeSegments[ACCELERATION]);
  printf("const velocity time: %9.4f - %9.4f\n", TimeSegments[ACCELERATION], TimeSegments[DECELERATION]);
  printf("DECELERATION time:   %9.4f - %9.4f\n\n", TimeSegments[DECELERATION], TimeSegments[FULL_TIME]);

  setupTrajectory_RT(TCP_Displacements, TimeSegments, TCP_feedRate);



  float32_t time = 0;
  float32_t velocity = 0;
  float32_t disp = 0;
  float32_t MG_disp;
  float32_t XCP_disp, YCP_disp, ZCP_disp;
  float32_t radicamd_disp = 0;
  size_t LoopCalls = 0;  //(size_t)(TimeSegments[FULL_TIME] / VEL_UPDATE_STIME) + 1;

  // set the current point to the start point
  current_TCP_point = point1;
  for (size_t i = 0; i <= LoopCalls; i++) {

    // get the current time in the loop
    time = VEL_UPDATE_STIME * i;

    // Displacement at a given time based on the displacement profile
    disp = calculateDisplacement_RT(time);

    // velocity at a given time based on the velocity profile
    velocity = calculateVelocity_RT(time);

    // TCP_point at a specified displacement along the linear path from start to end
    next_TCP_point = point_along_tcp_line(&point1, &point2, disp);

    printf("next Position x= %7.3f, y= %7.3f, z= %7.3f\n", next_TCP_point.x, next_TCP_point.y, next_TCP_point.z);

    // get the componect displacemt
    XCP_disp = next_TCP_point.x - current_TCP_point.x;  // get the x displacemt
    YCP_disp = next_TCP_point.y - current_TCP_point.y;  // get the y displacemt
    ZCP_disp = next_TCP_point.z - current_TCP_point.z;  // get the x displacemt

    // get the totle displacement between the current the new point
    radicamd_disp = XCP_disp * XCP_disp + YCP_disp * YCP_disp + ZCP_disp * ZCP_disp;
    arm_sqrt_f32(radicamd_disp, &MG_disp);

    //obtain joint angles for a given TCP position and orientation
    resultIK = INVERSE_KINEMATICS(next_TCP_point);
    TCP_angles angles_sol1 = resultIK.angles[IK_SOL1];
    TCP_angles angles_sol2 = resultIK.angles[IK_SOL2];


    printf("inverse Kinematics result:\n");
    printf("sol1: ");
    printf("J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f, J5 = %.4f, J6 = %.4f\n", angles_sol1.theta1, angles_sol1.theta2, angles_sol1.theta3, angles_sol1.theta4, angles_sol1.theta5, angles_sol1.theta6);
    printf("sol2: ");
    printf("J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f, J5 = %.4f, J6 = %.4f\n", angles_sol2.theta1, angles_sol2.theta2, angles_sol2.theta3, angles_sol2.theta4, angles_sol2.theta5, angles_sol2.theta6);


    // determine the TCP position from given joint angles
    // bJacobian if true will only calqulate the Jacobia
    resultFK = FORWARD_KINEMATICS(resultIK.angles[0], false, &jacobian);
    printf("\nForward Kinematics result:\n");
    printf("Can reach: %d\n", resultFK.bCanReach);
    printf("Position: x = %.3f, y = %.3f, z = %.3f\n", resultFK.point.x, resultFK.point.y, resultFK.point.z);
    printf("Orientation (Euler angles): rx = %.3f, ry = %.3f, rz = %.3f\n", resultFK.point.rx, resultFK.point.ry, resultFK.point.rz);

    // // print data for debug
    // printf("Disp: %7.3f point on path XYZ: (%7.3f, %7.3f, %7.3f) ", disp, next_TCP_point.x, next_TCP_point.y, next_TCP_point.z);
    // printf("TCP vel: %7.3f joint vel: %7.3f, %7.3f, %7.3f ", velocity, TCP_angle_vel.pData[0], TCP_angle_vel.pData[1], TCP_angle_vel.pData[2]);
    // printf(" %7.3f, %7.3f, %7.3f\n", TCP_angle_vel.pData[3], TCP_angle_vel.pData[4], TCP_angle_vel.pData[5]);

    //Serial.printf("Time: %9.4f\t Phace: %d Velocity: %9.4f\t Disp: %9.4f\n", time, phace, velocity, disp);

    next_TCP_point = current_TCP_point;  // update the nect point as the new point
  }



  while (1)
    ;
}

// Function to compute pseudo-inverse using SVD (simplified for 6x6)
arm_status svd_pseudo_inverse_f32(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst) {
    // Ensure the matrix is square
    if (src->numRows != src->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    uint32_t n = src->numRows;
    float U[n * n], S[n], V[n * n];
    float tolerance = 1e-6f; // Tolerance for small singular values

    // Step 1: Perform SVD (simplified, requires an SVD library)
    // You would typically call an SVD library function here.
    // For now, let's assume we have U, S, and V as the decomposition matrices.

    // Step 2: Compute the pseudo-inverse from SVD
    for (uint32_t i = 0; i < n; i++) {
        // Check for small singular values and ignore them if they're below the threshold
        if (fabs(S[i]) > tolerance) {
            S[i] = 1.0f / S[i];  // Invert the singular values
        } else {
            S[i] = 0.0f;  // Treat very small singular values as zero
        }
    }

    // Step 3: Build the pseudo-inverse
    // V * S^+ * U^T
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            dst->pData[i * n + j] = 0.0f;
            for (uint32_t k = 0; k < n; k++) {
                dst->pData[i * n + j] += V[i * n + k] * S[k] * U[j * n + k];
            }
        }
    }

    return ARM_MATH_SUCCESS;
}

// Create a safe inverse using damping
arm_status arm_mat_damped_inverse_f32(const arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst) {
  if (src->numRows != src->numCols) {
    return ARM_MATH_SIZE_MISMATCH;  // Must be square
  }

  uint32_t n = src->numRows;
  float damped_data[n * n];

  // Make a copy of src into damped_data
  for (uint32_t i = 0; i < n * n; i++) {
    damped_data[i] = src->pData[i];
  }

  // Add damping to the diagonal
  for (uint32_t i = 0; i < n; i++) {
    damped_data[i * n + i] += DAMPING_FACTOR;
  }

  arm_matrix_instance_f32 damped_matrix;
  arm_mat_init_f32(&damped_matrix, n, n, damped_data);

  // Now try to invert
  return arm_mat_inverse_f32(&damped_matrix, dst);
}

bool pseudoInverse6x6(const arm_matrix_instance_f32 *jacobian, arm_matrix_instance_f32 *pseudoInverse) {
  arm_matrix_instance_f32 JT, JTJ, JTJ_reg, JTJ_inv;
  float32_t JT_data[6 * 6];
  float32_t JTJ_data[6 * 6];
  float32_t JTJ_reg_data[6 * 6];
  float32_t JTJ_inv_data[6 * 6];
  float32_t lambda = 1e-6f;  // regularization factor

  arm_mat_init_f32(&JT, 6, 6, JT_data);
  arm_mat_init_f32(&JTJ, 6, 6, JTJ_data);
  arm_mat_init_f32(&JTJ_reg, 6, 6, JTJ_reg_data);
  arm_mat_init_f32(&JTJ_inv, 6, 6, JTJ_inv_data);

  // Step 1: JT = Transpose(J)
  if (arm_mat_trans_f32(jacobian, &JT) != ARM_MATH_SUCCESS) {
    return false;
  }

  // Step 2: JTJ = JT * J
  if (arm_mat_mult_f32(&JT, jacobian, &JTJ) != ARM_MATH_SUCCESS) {
    return false;
  }

  // Step 3: JTJ_reg = JTJ + lambda * I
  for (int i = 0; i < 6 * 6; i++) {
    JTJ_reg_data[i] = JTJ_data[i];
  }
  for (int i = 0; i < 6; i++) {
    JTJ_reg_data[i * 6 + i] += lambda;  // add lambda to diagonal
  }

  // Step 4: Invert JTJ_reg
  if (arm_mat_inverse_f32(&JTJ_reg, &JTJ_inv) != ARM_MATH_SUCCESS) {
    return false;
  }

  // Step 5: pseudoInverse = JTJ_inv * JT
  if (arm_mat_mult_f32(&JTJ_inv, &JT, pseudoInverse) != ARM_MATH_SUCCESS) {
    return false;
  }

  return true;  // success
}







// start = micros();
// for (int i = 0; i < ITERATIONS; i++) {

// // Compute Inverse Kinematics for point
// resultIK = INVERSE_KINEMATICS(test_point);

// // Compute Forward Kinematics and get Jacobian
// resultFK = FORWARD_KINEMATICS(resultIK.angles[0], true, &jacobian);
// arm_mat_inverse_f32(&jacobian, &INV_jacobian);
// arm_mat_mult_f32(&INV_jacobian, &TCP_vel, &TCP_angle_vel);
// }
// end = micros();
// Serial.print(" FK, IK, INV_mat, mult_max kinematics time for ");
// Serial.print(ITERATIONS);
// Serial.print(" iterations (us): ");
// Serial.println(end - start);
// printArmMatrix(&TCP_angle_vel);

// Output results
// printf("Forward Kinematics result:\n");
// printf("Can reach: %d\n", resultFK.bCanReach);
// printf("Position: x = %.3f, y = %.3f, z = %.3f\n", resultFK.point.x, resultFK.point.y, resultFK.point.z);
// printf("Orientation (Euler angles): rx = %.3f, ry = %.3f, rz = %.3f\n", resultFK.point.rx, resultFK.point.ry, resultFK.point.rz);
// printf("\nJacobian matrix:\n");
// printArmMatrix(resultFK.Jacobian);

// Infinite loop to keep the program running

// TCP_point test_point = { -203.000d, 203.340d, 123.456d, 1.0d, 1.0d, 1.0d };
// INVERSE_SOLUTION resultIK = INVERSE_KINEMATICS(test_point);
// TCP_angles angles_sol1 = resultIK.angles[IK_SOL1];
// TCP_angles angles_sol2 = resultIK.angles[IK_SOL2];

// printf("inverse Kinematics result:\n");
// printf("sol1: ");
// printf("J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f, J5 = %.4f, J6 = %.4f\n", angles_sol1.theta1, angles_sol1.theta2, angles_sol1.theta3, angles_sol1.theta4, angles_sol1.theta5, angles_sol1.theta6);
// printf("sol2: ");
// printf("J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f, J5 = %.4f, J6 = %.4f\n", angles_sol2.theta1, angles_sol2.theta2, angles_sol2.theta3, angles_sol2.theta4, angles_sol2.theta5, angles_sol2.theta6);

// float32_t start, end;

// start = micros();
// for (int i = 0; i < ITERATIONS; i++) {
//   FORWARD_SOLUTION resultFK = FORWARD_KINEMATICS(test_angles, true);
// }
// end = micros();
// Serial.print("forward kinematics time for ");
// Serial.print(ITERATIONS);
// Serial.print(" iterations (us): ");
// Serial.println(end - start);


// start = micros();
// for (int i = 0; i < ITERATIONS; i++) {
//   INVERSE_SOLUTION resultIK = INVERSE_KINEMATICS(test_point);
//   if (!resultIK.bCanReach[IK_SOL1] || !resultIK.bCanReach[IK_SOL2]) {
//     break;
//     Serial.print("ik failed");
//   }
// }
// end = micros();
// Serial.print("inverse kinematics time for ");
// Serial.print(ITERATIONS);
// Serial.print(" iterations (us): ");
// Serial.println(end - start);








// //HomeMotor(JOINT1, true);
// if (limit_triggered & BIT0) {
//   Serial.println("switch 1 pressed");
//   // limit_triggered &= ~BIT1;
// }
// if (limit_triggered & BIT1) {
//   Serial.println("switch 2 pressed");
//   // limit_triggered &= ~BIT2;
// }

// if (limit_triggered & BIT2) {
//   Serial.println("switch 3 pressed");
//   // limit_triggered &= ~BIT3;
// }
//}


void testVP() {
  // float32_t Displacement = 20;
  // float32_t feedRate = 6;
  // bool optimal = false;
  // float32_t TCP_feedRate = calculateFeedRate(Displacement, feedRate, optimal);
  // printf("TCP feed Rate: %.3f\n", TCP_feedRate);

  // float32_t* TimeSegments = calculateTimeSegments(Displacement, TCP_feedRate);
  // printf("ACCELERATION time: 0.0 - %.3f\n", TimeSegments[ACCELERATION]);
  // printf("const velocity time: %.3f - %.3f\n", TimeSegments[ACCELERATION], TimeSegments[DECELERATION]);
  // printf("DECELERATION time: %.3f - %.3f\n", TimeSegments[DECELERATION], TimeSegments[FULL_TIME]);

  // size_t VelocityLen = 0;
  // size_t DisplacementLen = 0;
  // float32_t* VelocityProfile = calculateVelocityProfile(TimeSegments, Displacement, TCP_feedRate, &VelocityLen);
  // float32_t* DisplacementProfile = calculateDisplacementProfile(TimeSegments, Displacement, TCP_feedRate, &DisplacementLen);

  // float32_t time = 0;
  // for (size_t i = 0; i < VelocityLen; i += 5) {
  //   time = i * VEL_UPDATE_TIME;
  //   printf("Time %.3f,\t Displacement %.3f,\t Velocity %.3f\n", time, DisplacementProfile[i], VelocityProfile[i]);
  // }
}

void shell_test() {
  // if (limit_triggered != BIT0) {
  //       Serial.println("Rotating LINK1...");
  //       controller.rotateAsync(LINK1); // Rotate at speed 50

  //       while (!limit_reached) {
  //           // Wait for the limit switch to trigger
  //       }
  //       controller.stop();

  //       Serial.println("Limit reached! Stopping LINK1.");
  //       controller.emergencyStop();
  //   }


  // static char buffer[CMD_BUFFER_SIZE];

  // // Read all available serial data and enqueue commands
  // while (Serial.available()) {
  //   Serial.readBytesUntil('\n', buffer, sizeof(buffer));
  //   enqueueCommand(buffer);
  // }

  // // Process queued commands one by one
  // if (dequeueCommand(buffer)) {
  //   processCommand(buffer);
  // }
}

void testIKFK() {
  // TCP_point End_efector = { 0.0d, 0.0d, 10.0d, 0.0d, 0.0d, 0.0d };
  // set_tool_frame(&End_efector);

  // TCP_angles test_angles = { 0.0d, 0.0d, 0.0d, 0.0d, 0.0d, 0.0d };  // Example joint angles in degrees
  // FORWARD_SOLUTION resultFK = FORWARD_KINEMATICS(test_angles, true);

  // printf("Forward Kinematics result:\n");
  // printf("can reach: %d \n", resultFK.bCanReach);
  // printf("Position: x = %.3f, y = %.3f, z = %.3f\n", resultFK.point.x, resultFK.point.y, resultFK.point.z);
  // printf("Orientation (Euler angles): rx = %.3f, ry = %.3f, rz = %.3f\n", resultFK.point.rx, resultFK.point.ry, resultFK.point.rz);
  // printf("\nJacobian matrix:\n");
  // printArmMatrix(resultFK.Jacobian);


  // TCP_point test_point = { -203.000d, 203.340d, 123.456d, 1.0d, 1.0d, 1.0d };
  // INVERSE_SOLUTION resultIK = INVERSE_KINEMATICS(test_point);
  // TCP_angles angles_sol1 = resultIK.angles[IK_SOL1];
  // TCP_angles angles_sol2 = resultIK.angles[IK_SOL2];

  // printf("inverse Kinematics result:\n");
  // printf("sol1: ");
  // printf("J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f, J5 = %.4f, J6 = %.4f\n", angles_sol1.theta1, angles_sol1.theta2, angles_sol1.theta3, angles_sol1.theta4, angles_sol1.theta5, angles_sol1.theta6);
  // printf("sol2: ");
  // printf("J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f, J5 = %.4f, J6 = %.4f\n", angles_sol2.theta1, angles_sol2.theta2, angles_sol2.theta3, angles_sol2.theta4, angles_sol2.theta5, angles_sol2.theta6);

  // float32_t start, end;

  // start = micros();
  // for (int i = 0; i < ITERATIONS; i++) {
  //   FORWARD_SOLUTION resultFK = FORWARD_KINEMATICS(test_angles, true);
  // }
  // end = micros();
  // Serial.print("forward kinematics time for ");
  // Serial.print(ITERATIONS);
  // Serial.print(" iterations (us): ");
  // Serial.println(end - start);


  // start = micros();
  // for (int i = 0; i < ITERATIONS; i++) {
  //   INVERSE_SOLUTION resultIK = INVERSE_KINEMATICS(test_point);
  //   if (!resultIK.bCanReach[IK_SOL1] || !resultIK.bCanReach[IK_SOL2]) {
  //     break;
  //     Serial.print("ik failed");
  //   }
  // }
  // end = micros();
  // Serial.print("inverse kinematics time for ");
  // Serial.print(ITERATIONS);
  // Serial.print(" iterations (us): ");
  // Serial.println(end - start);
}