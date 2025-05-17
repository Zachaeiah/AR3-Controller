//======================================== Include Libraries ========================================
#include "kinamatics.h"

//======================================== Global Variable Definitions ==============================
kinematics_state Home_pose = { 333.080f, 0.0f, 474.770f, -2.35619f, 1.57081f, -2.35619f, 0, 0, 0, 0, 0, 0 };
kinematics_state END_STOP_pose = { -59.61273, -13.93499, 322.74166, -1.64245 - 1.09169, -1.97535, JOINT1_MAX, JOINT2_MIN, JOINT3_MAX, JOINT4_MIN, JOINT5_MIN, JOINT6_MIN };


float32_t ToolFrame[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
arm_matrix_instance_f32 Tool_Matrix;


//======================================== Function Implementations =================================
/**
 * @brief Computes inverse kinematics to obtain joint angles for a given TCP position and orientation.
 *
 * @param point The desired end-effector position and orientation (TCP_point).
 * @return INVERSE_SOLUTION The computed joint angles required to reach the specified TCP position.
 */
INVERSE_SOLUTION INVERSE_KINEMATICS(const TCP_point point) {

  INVERSE_SOLUTION isolF{};     // Structure to store the inverse kinematics solution
  TCP_angles isol1{}, isol2{};  // Two possible solutions for joint angles

  // Define transformation matrices
  arm_matrix_instance_f32 R0_T_Matrix, IV_ToolFrame;
  arm_matrix_instance_f32 R0_6_Matrix, IV_R0_6_Matrix;
  arm_matrix_instance_f32 R0_5_Matrix;
  arm_matrix_instance_f32 J1_Matrix, J2_Matrix, J3_Matrix;
  arm_matrix_instance_f32 R0_2_Matrix, R0_3_Matrix;
  arm_matrix_instance_f32 R0_6_Rotashion_Matrix, R0_3_Transposed_Matrix;
  arm_matrix_instance_f32 R3_6_Rotashion_Matrix;

  // Define float variables for calculations
  float32_t sqrt_result;
  float32_t X_prime;         // Projections of wrist center on the XY plane
  float32_t L1, L2, L3, L4;  // Distance variables for geometric calculations
  float32_t OB, OC, OD, OE;  // Angles used in triangle calculations
  float acos_input1 = 0;
  float acos_input2 = 0;

  // Storage arrays for transformation matrices
  float32_t R0_3_Transposed[9] = { 0 };
  float32_t R0_6_Rotashion[9] = { 0 };
  float32_t R3_6_Matrix_data[9] = { 0 };
  float32_t R0_T_transformMatrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_6_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };

  // Transformation matrix to offset for the tool frame
  float32_t IV_R0_6_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 1.0, 0.0, 0.0, 0.0,
                                                               0.0, 1.0, 0.0, 0.0,
                                                               0.0, 0.0, 1.0, -LINK6_Z_OFFSET,
                                                               0.0, 0.0, 0.0, 1.0 };

  // Initialize rotation transformation matrices
  float32_t R0_5_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J1_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J2_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J3_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t IV_J3_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_2_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_3_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t IV_ToolFrame_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };  // inverted tool frame

  // Compute the transformation matrix from base frame to TCP
  SetRotationTranslatgeMatrix(R0_T_transformMatrix_data, &point);

  // Compute the inverse of the tool frame matrix
  HMinvertMatrix(IV_ToolFrame_data, ToolFrame);

  // Initialize matrices for transformations and computations
  arm_mat_init_f32(&R0_6_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_6_Matrix_data);
  arm_mat_init_f32(&IV_R0_6_Matrix, MATRIX_SIZE, MATRIX_SIZE, IV_R0_6_Matrix_data);
  arm_mat_init_f32(&R0_5_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_5_Matrix_data);
  arm_mat_init_f32(&R0_T_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_T_transformMatrix_data);
  arm_mat_init_f32(&IV_ToolFrame, MATRIX_SIZE, MATRIX_SIZE, IV_ToolFrame_data);

  // Compute transformation to wrist center
  if (arm_mat_mult_f32(&R0_T_Matrix, &IV_ToolFrame, &R0_6_Matrix) != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(MMULT_FAILED, __func__, __LINE__, "HIGH", "Matrix multiplication failed");
    return isolF;
  }



  if (arm_mat_mult_f32(&R0_6_Matrix, &IV_R0_6_Matrix, &R0_5_Matrix) != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(MMULT_FAILED, __func__, __LINE__, "HIGH", "Matrix multiplication failed");
    return isolF;
  }


  if (R0_5_Matrix.pData[3] == 0 && R0_5_Matrix.pData[7] == 0) {
    return (isolF.bCanReach[IK_SOL1] = isolF.bCanReach[IK_SOL2] = false,
            print_error(FK_FAILE, __func__, __LINE__, "Msg tbt", "LOW"), isolF);
  }
  isol1.theta1 = atan2f(R0_5_Matrix.pData[7], R0_5_Matrix.pData[3]);
  isol2.theta1 = isol1.theta1;

  // Compute the projection of the wrist center on the XY plane
  X_prime = R0_5_Matrix.pData[3] * arm_cos_f32(-isol1.theta1) - R0_5_Matrix.pData[7] * arm_sin_f32(-isol1.theta1);
  //Y_prime = R0_5_Matrix.pData[7] * arm_cos_f32(-isol1.theta1) + R0_5_Matrix.pData[3] * arm_sin_f32(-isol1.theta1);

  // Compute distances for triangle geometry calculations
  L1 = fabs(X_prime - LINK1_X_OFFSET);
  L4 = R0_5_Matrix.pData[11] - LINK1_Z_OFFSET;

  // Compute distances using Pythagorean theorem
  arm_status sqrt_status1 = arm_sqrt_f32(LINK3_X_OFFSET * LINK3_X_OFFSET + LINK4_Z_OFFSET * LINK4_Z_OFFSET, &L3);
  arm_status sqrt_status2 = arm_sqrt_f32(L1 * L1 + L4 * L4, &L2);
  if (sqrt_status1 != ARM_MATH_SUCCESS || sqrt_status2 != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = isolF.bCanReach[IK_SOL2] = false;
    print_error(SQURT_FAILED, __func__, __LINE__, "HIGH", "Input into sqrt is a negative value");
    return isolF;
  }


  // Validate inputs for acosf function
  acos_input1 = (LINK2_X_OFFSET * LINK2_X_OFFSET + L2 * L2 - L3 * L3) / (2 * LINK2_X_OFFSET * L2);
  acos_input2 = (L3 * L3 + LINK2_X_OFFSET * LINK2_X_OFFSET - L2 * L2) / (2 * L3 * LINK2_X_OFFSET);
  if (fabsf(acos_input1) > 1.0f || fabsf(acos_input2) > 1.0f) {
    isolF.bCanReach[IK_SOL1] = isolF.bCanReach[IK_SOL2] = false;
    print_error(TRIG_RANGE, __func__, __LINE__, "HIGH", "Input for acos() is out of range");
    return isolF;
  }

  // Compute joint angles using trigonometric calculations
  OC = acosf(acos_input1);
  OD = acosf(acos_input2);
  OB = atanf(L1 / L4);
  OE = atanf(LINK3_X_OFFSET / LINK4_Z_OFFSET);

  // Compute final joint angles using quadrant-based logic
  if (X_prime > LINK1_X_OFFSET) {
    if (L4 > 0) {
      isol1.theta2 = OB - OC;
      isol2.theta2 = isol1.theta2;
    } else {
      isol1.theta2 = OB - OC + PI;
      isol2.theta2 = isol1.theta2;
    }
  } else {
    isol1.theta2 = -(OB + OC);
    isol2.theta2 = isol1.theta2;
  }


  isol1.theta3 = -(OD + OE) + HALF_PI;
  isol2.theta3 = isol1.theta3;

  computeTransformMatrix(J1_transformMatrix, isol1.theta1, LINK1_ALFA, LINK1_X_OFFSET, LINK1_Z_OFFSET);
  computeTransformMatrix(J2_transformMatrix, isol1.theta2 - HALF_PI, LINK2_ALFA, LINK2_X_OFFSET, LINK2_Z_OFFSET);
  computeTransformMatrix(J3_transformMatrix, isol1.theta3 + PI, LINK3_ALFA, LINK3_X_OFFSET, LINK3_Z_OFFSET);

  arm_mat_init_f32(&J1_Matrix, MATRIX_SIZE, MATRIX_SIZE, J1_transformMatrix);
  arm_mat_init_f32(&J2_Matrix, MATRIX_SIZE, MATRIX_SIZE, J2_transformMatrix);
  arm_mat_init_f32(&J3_Matrix, MATRIX_SIZE, MATRIX_SIZE, J3_transformMatrix);
  arm_mat_init_f32(&R0_2_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_2_Matrix_data);
  arm_mat_init_f32(&R0_3_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_3_Matrix_data);

  if (arm_mat_mult_f32(&J1_Matrix, &J2_Matrix, &R0_2_Matrix) != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(MMULT_FAILED, __func__, __LINE__, "HIGH", "Matrix multiplication failed");
    return isolF;
  }

  if (arm_mat_mult_f32(&R0_2_Matrix, &J3_Matrix, &R0_3_Matrix) != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(MMULT_FAILED, __func__, __LINE__, "HIGH", "Matrix multiplication failed");
    return isolF;
  }

  HMinvertMatrix(IV_J3_transformMatrix, R0_3_Matrix.pData);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R0_3_Transposed[i * 3 + j] = IV_J3_transformMatrix[i * 4 + j];
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R0_6_Rotashion[i * 3 + j] = R0_6_Matrix.pData[i * 4 + j];
    }
  }

  arm_mat_init_f32(&R0_6_Rotashion_Matrix, 3, 3, R0_6_Rotashion);
  arm_mat_init_f32(&R0_3_Transposed_Matrix, 3, 3, R0_3_Transposed);
  arm_mat_init_f32(&R3_6_Rotashion_Matrix, 3, 3, R3_6_Matrix_data);

  if (arm_mat_mult_f32(&R0_3_Transposed_Matrix, &R0_6_Rotashion_Matrix, &R3_6_Rotashion_Matrix) != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(MMULT_FAILED, __func__, __LINE__, "HIGH", "Matrix multiplication failed");
    return isolF;
  }


  isol1.theta4 = -atan2f(R3_6_Rotashion_Matrix.pData[5], -R3_6_Rotashion_Matrix.pData[2]);
  isol2.theta4 = -atan2f(-R3_6_Rotashion_Matrix.pData[5], R3_6_Rotashion_Matrix.pData[2]);

  // Error check for sqrt before theta5 calculation
  if (arm_sqrt_f32(1 - R3_6_Rotashion_Matrix.pData[8] * R3_6_Rotashion_Matrix.pData[8], &sqrt_result) != ARM_MATH_SUCCESS) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(SQURT_FAILED, __func__, __LINE__, "HIGH", "Input into squrt is a negative value");
    return isolF;
  }

  isol1.theta5 = atan2f(-sqrt_result, R3_6_Rotashion_Matrix.pData[8]);
  isol2.theta5 = atan2f(sqrt_result, R3_6_Rotashion_Matrix.pData[8]);

  isol1.theta6 = atan2f(-R3_6_Rotashion_Matrix.pData[7], R3_6_Rotashion_Matrix.pData[6]);
  isol2.theta6 = atan2f(R3_6_Rotashion_Matrix.pData[7], -R3_6_Rotashion_Matrix.pData[6]);

  // Additional error checking for invalid angles
  if (isnan(isol1.theta1) || isnan(isol1.theta2) || isnan(isol1.theta3) || isnan(isol1.theta4) || isnan(isol1.theta5) || isnan(isol1.theta6) || isnan(isol2.theta1) || isnan(isol2.theta2) || isnan(isol2.theta3) || isnan(isol2.theta4) || isnan(isol2.theta5) || isnan(isol2.theta6)) {
    isolF.bCanReach[IK_SOL1] = false;
    isolF.bCanReach[IK_SOL2] = false;
    print_error(iS_NAN, __func__, __LINE__, "HIGH", "an IK angle is nan");
    return isolF;
  }

  // Store solutions if valid
  isolF.angles[IK_SOL1] = isol1;
  isolF.angles[IK_SOL2] = isol2;
  isolF.bCanReach[IK_SOL1] = true;
  isolF.bCanReach[IK_SOL2] = true;

  return isolF;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Computes forward kinematics to determine the TCP position from given joint angles.
 *
 * @param angles The joint angles of the robotic arm (TCP_angles).
 * @param bJacobian if true will only calqulate the Jacobia, else it will do both FK and bJacobian
 * @return FORWARD_SOLUTION The computed TCP position and orientation.
 */
FORWARD_SOLUTION FORWARD_KINEMATICS(const TCP_angles angles, bool bJacobian, arm_matrix_instance_f32* jacobian) {

  FORWARD_SOLUTION fsol;  // Structure to store the forward kinematics solution

  // Declare transformation matrices for each joint
  arm_matrix_instance_f32 J1_Matrix, J2_Matrix, J3_Matrix, J4_Matrix, J5_Matrix, J6_Matrix;
  arm_matrix_instance_f32 R0_2_Matrix, R0_3_Matrix, R0_4_Matrix, R0_5_Matrix, R0_6_Matrix, R0_T_Matrix;
  // arm_matrix_instance_f32 jacobian;
  TCP_point FK_solution;  // Structure to store the calculated TCP position
  int arm_mul_status[6] = {};
  float32_t sqrt_result;  // Variable to store the result of square root calculation

  // Initialize transformation matrices for each joint
  float32_t J1_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J2_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J3_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J4_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J5_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t J6_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_T_transformMatrix[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_2_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_3_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_4_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_5_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  float32_t R0_6_Matrix_data[MATRIX_SIZE * MATRIX_SIZE] = { 0 };
  // float32_t jacobian_data[6 * 6] = { 0 };  // Storage for 6x6 Jacobian matrix


  // Additional error checking for invalid angles
  if (isnan(angles.theta1) || isnan(angles.theta2) || isnan(angles.theta3) || isnan(angles.theta4) || isnan(angles.theta5) || isnan(angles.theta6)) {
    fsol.point = { 0 };
    fsol.bCanReach = false;
    print_error(iS_NAN, __func__, __LINE__, "HIGH", "FK input agle is nan");
    return fsol;
  }

  // Compute transformation matrices for each joint using DH parameters
  computeTransformMatrix(J1_transformMatrix, angles.theta1, LINK1_ALFA, LINK1_X_OFFSET, LINK1_Z_OFFSET);
  computeTransformMatrix(J2_transformMatrix, angles.theta2 - HALF_PI, LINK2_ALFA, LINK2_X_OFFSET, LINK2_Z_OFFSET);
  computeTransformMatrix(J3_transformMatrix, angles.theta3 + PI, LINK3_ALFA, LINK3_X_OFFSET, LINK3_Z_OFFSET);
  computeTransformMatrix(J4_transformMatrix, angles.theta4, LINK4_ALFA, LINK4_X_OFFSET, LINK4_Z_OFFSET);
  computeTransformMatrix(J5_transformMatrix, angles.theta5, LINK5_ALFA, LINK5_X_OFFSET, LINK5_Z_OFFSET);
  computeTransformMatrix(J6_transformMatrix, angles.theta6, LINK6_ALFA, LINK6_X_OFFSET, LINK6_Z_OFFSET);


  // Initialize matrix structures with transformation data
  arm_mat_init_f32(&J1_Matrix, MATRIX_SIZE, MATRIX_SIZE, J1_transformMatrix);
  arm_mat_init_f32(&J2_Matrix, MATRIX_SIZE, MATRIX_SIZE, J2_transformMatrix);
  arm_mat_init_f32(&J3_Matrix, MATRIX_SIZE, MATRIX_SIZE, J3_transformMatrix);
  arm_mat_init_f32(&J4_Matrix, MATRIX_SIZE, MATRIX_SIZE, J4_transformMatrix);
  arm_mat_init_f32(&J5_Matrix, MATRIX_SIZE, MATRIX_SIZE, J5_transformMatrix);
  arm_mat_init_f32(&J6_Matrix, MATRIX_SIZE, MATRIX_SIZE, J6_transformMatrix);
  arm_mat_init_f32(&R0_2_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_2_Matrix_data);
  arm_mat_init_f32(&R0_3_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_3_Matrix_data);
  arm_mat_init_f32(&R0_4_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_4_Matrix_data);
  arm_mat_init_f32(&R0_5_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_5_Matrix_data);
  arm_mat_init_f32(&R0_6_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_6_Matrix_data);
  arm_mat_init_f32(&R0_T_Matrix, MATRIX_SIZE, MATRIX_SIZE, R0_T_transformMatrix);
  //arm_mat_init_f32(&jacobian, 6, 6, jacobian_data);


  // Perform matrix multiplications to compute the transformation matrix from base to TCP
  arm_mul_status[0] = arm_mat_mult_f32(&J1_Matrix, &J2_Matrix, &R0_2_Matrix);
  arm_mul_status[1] = arm_mat_mult_f32(&R0_2_Matrix, &J3_Matrix, &R0_3_Matrix);
  arm_mul_status[2] = arm_mat_mult_f32(&R0_3_Matrix, &J4_Matrix, &R0_4_Matrix);
  arm_mul_status[3] = arm_mat_mult_f32(&R0_4_Matrix, &J5_Matrix, &R0_5_Matrix);
  arm_mul_status[4] = arm_mat_mult_f32(&R0_5_Matrix, &J6_Matrix, &R0_6_Matrix);
  arm_mul_status[5] = arm_mat_mult_f32(&R0_6_Matrix, &Tool_Matrix, &R0_T_Matrix);

  for (int i = 0; i < 6; i++) {
    if (arm_mul_status[i] != ARM_MATH_SUCCESS) {
      fsol.point = { 0 };
      fsol.bCanReach = false;
      print_error(MMULT_FAILED, __func__, __LINE__, "HIGH", "Matrix multiplication failed, step %d in FK", i);
      return fsol;
    }
  }

  compute_jacobian(&J1_Matrix, &R0_2_Matrix, &R0_3_Matrix, &R0_4_Matrix, &R0_5_Matrix, &R0_6_Matrix, jacobian);

  if (bJacobian) {
    return fsol;
  }


  // Extract the TCP position from the final transformation matrix
  FK_solution.x = R0_T_Matrix.pData[3];   // X position
  FK_solution.y = R0_T_Matrix.pData[7];   // Y position
  FK_solution.z = R0_T_Matrix.pData[11];  // Z position

  // Compute roll, pitch, and yaw from the rotation matrix
  if (arm_sqrt_f32(R0_T_Matrix.pData[9] * R0_T_Matrix.pData[9] + R0_T_Matrix.pData[10] * R0_T_Matrix.pData[10], &sqrt_result) != ARM_MATH_SUCCESS) {
    fsol.point = { 0 };
    fsol.bCanReach = false;
    Serial.printf("could not do squrt\n");
    print_error(SQURT_FAILED, __func__, __LINE__, "HIGH", "Input into squrt is a negative value");
    return fsol;
  }

  FK_solution.ry = atan2f(-R0_T_Matrix.pData[8], sqrt_result);

  if (fabs(FK_solution.ry - HALF_PI) <= epsilon) {
    FK_solution.ry += epsilon;
  }

  FK_solution.rx = atan2f(R0_T_Matrix.pData[9] / arm_cos_f32(FK_solution.ry), R0_T_Matrix.pData[10] / arm_cos_f32(FK_solution.ry));  // Roll (rotation around X-axis)
  FK_solution.rz = atan2f(R0_T_Matrix.pData[4] / arm_cos_f32(FK_solution.ry), R0_T_Matrix.pData[0] / arm_cos_f32(FK_solution.ry));   // Yaw (rotation around Z-axis)

  // Additional error checking for invalid angles
  if (isnan(FK_solution.x) || isnan(FK_solution.y) || isnan(FK_solution.z) || isnan(FK_solution.rx) || isnan(FK_solution.ry) || isnan(FK_solution.rz)) {
    fsol.point = { 0 };
    fsol.bCanReach = false;
    print_error(iS_NAN, __func__, __LINE__, "HIGH", "FK XYZ solution param is nan");
    return fsol;
  }

  // Store the computed TCP point in the solution structure
  fsol.point = FK_solution;
  fsol.bCanReach = true;

  return fsol;  // Return the computed forward kinematics solution
}

//----------------------------------------------------------------------------------------------------------------
/**
 * @brief Computes the numeric Jacobian matrix for the robotic arm.
 *
 * @param 
 * @param J The Jacobian matrix (3x6) to be computed.
 */
void compute_jacobian(const arm_matrix_instance_f32 *T1, const arm_matrix_instance_f32 *T2,
                      const arm_matrix_instance_f32 *T3, const arm_matrix_instance_f32 *T4,
                      const arm_matrix_instance_f32 *T5, const arm_matrix_instance_f32 *T6,
                      arm_matrix_instance_f32 *jacobian) {

  /**
   * @brief effector position
   */
  float32_t p_e[3] = { T6->pData[3], T6->pData[7], T6->pData[11] };

  // Z-axes of each joint
  float32_t z0[3] = { 0, 0, 1 };
  float32_t z1[3] = { T2->pData[2], T2->pData[6], T2->pData[10] };
  float32_t z2[3] = { T3->pData[2], T3->pData[6], T3->pData[10] };
  float32_t z3[3] = { T4->pData[2], T4->pData[6], T4->pData[10] };
  float32_t z4[3] = { T5->pData[2], T5->pData[6], T5->pData[10] };
  float32_t z5[3] = { T6->pData[2], T6->pData[6], T6->pData[10] };


  // Joint positions
  float32_t p1[3] = { T2->pData[3], T2->pData[7], T2->pData[11] };
  float32_t p2[3] = { T3->pData[3], T3->pData[7], T3->pData[11] };
  float32_t p3[3] = { T4->pData[3], T4->pData[7], T4->pData[11] };
  float32_t p4[3] = { T5->pData[3], T5->pData[7], T5->pData[11] };
  float32_t p5[3] = { T6->pData[3], T6->pData[7], T6->pData[11] };

  float32_t Cross_temp[3] = { 0 };
  float32_t Sub_temp[3] = { 0 };

  // Jv columns
  cross_product(z0, p_e, Cross_temp);
  jacobian->pData[0] = Cross_temp[0];
  jacobian->pData[6] = Cross_temp[1];
  jacobian->pData[12] = Cross_temp[2];
  memset(Cross_temp, 0, 3 * sizeof(float32_t));

  vector_subtract(p_e, p1, Sub_temp);
  cross_product(z1, Sub_temp, Cross_temp);
  jacobian->pData[1] = Cross_temp[0];
  jacobian->pData[7] = Cross_temp[1];
  jacobian->pData[13] = Cross_temp[2];
  memset(Sub_temp, 0, 3 * sizeof(float32_t));
  memset(Cross_temp, 0, 3 * sizeof(float32_t));

  vector_subtract(p_e, p2, Sub_temp);
  cross_product(z2, Sub_temp, Cross_temp);
  jacobian->pData[2] = Cross_temp[0];
  jacobian->pData[8] = Cross_temp[1];
  jacobian->pData[14] = Cross_temp[2];
  memset(Sub_temp, 0, 3 * sizeof(float32_t));
  memset(Cross_temp, 0, 3 * sizeof(float32_t));

  vector_subtract(p_e, p3, Sub_temp);
  cross_product(z3, Sub_temp, Cross_temp);
  jacobian->pData[3] = Cross_temp[0];
  jacobian->pData[9] = Cross_temp[1];
  jacobian->pData[15] = Cross_temp[2];
  memset(Sub_temp, 0, 3 * sizeof(float32_t));
  memset(Cross_temp, 0, 3 * sizeof(float32_t));

  vector_subtract(p_e, p4, Sub_temp);
  cross_product(z4, Sub_temp, Cross_temp);
  jacobian->pData[4] = Cross_temp[0];
  jacobian->pData[10] = Cross_temp[1];
  jacobian->pData[16] = Cross_temp[2];
  memset(Sub_temp, 0, 3 * sizeof(float32_t));
  memset(Cross_temp, 0, 3 * sizeof(float32_t));

  vector_subtract(p_e, p5, Sub_temp);
  cross_product(z5, Sub_temp, Cross_temp);
  jacobian->pData[5] = Cross_temp[0];
  jacobian->pData[11] = Cross_temp[1];
  jacobian->pData[17] = Cross_temp[2];
  memset(Sub_temp, 0, 3 * sizeof(float32_t));
  memset(Cross_temp, 0, 3 * sizeof(float32_t));

  // jacobian->pDataw columns
  jacobian->pData[18] = z0[0];
  jacobian->pData[24] = z0[1];
  jacobian->pData[30] = z0[2];
  jacobian->pData[19] = z1[0];
  jacobian->pData[25] = z1[1];
  jacobian->pData[31] = z1[2];
  jacobian->pData[20] = z2[0];
  jacobian->pData[26] = z2[1];
  jacobian->pData[32] = z2[2];
  jacobian->pData[21] = z3[0];
  jacobian->pData[27] = z3[1];
  jacobian->pData[33] = z3[2];
  jacobian->pData[22] = z4[0];
  jacobian->pData[28] = z4[1];
  jacobian->pData[34] = z4[2];
  jacobian->pData[23] = z5[0];
  jacobian->pData[29] = z5[1];
  jacobian->pData[35] = z5[2];
}

//---------------------------------------------------------------------------------------------------------------------
void cross_product(const float32_t a[3], const float32_t b[3], float32_t result[3]) {
  result[0] = a[1] * b[2] - a[2] * b[1];
  result[1] = a[2] * b[0] - a[0] * b[2];
  result[2] = a[0] * b[1] - a[1] * b[0];
}

//---------------------------------------------------------------------------------------------------------------------
void vector_subtract(const float32_t a[3], const float32_t b[3], float32_t result[3]) {
  result[0] = a[0] - b[0];
  result[1] = a[1] - b[1];
  result[2] = a[2] - b[2];
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Computes the transformation matrix using Denavit-Hartenberg (DH) parameters.
 *
 * This function constructs a 4x4 homogeneous transformation matrix based on the 
 * given joint parameters (theta, alpha) and link offsets (X_offset, Z_offset).
 * The transformation matrix follows the DH convention commonly used in robotics.
 *
 * @param transformMatrix Pointer to a 16-element array where the transformation matrix will be stored.
 * @param theta Rotation angle around the Z-axis (joint angle in DH parameters).
 * @param alpha Rotation angle around the X-axis (link twist in DH parameters).
 * @param X_offset Translation along the X-axis (link length in DH parameters).
 * @param Z_offset Translation along the Z-axis (link offset in DH parameters).
 */
void computeTransformMatrix(float32_t *transformMatrix, const float32_t theta, const float32_t alpha, const float32_t X_offset, const float32_t Z_offset) {
  // Compute trigonometric values for efficiency
  float32_t cosTheta = arm_cos_f32(theta);
  float32_t sinTheta = arm_sin_f32(theta);
  float32_t cosAlpha = arm_cos_f32(alpha);
  float32_t sinAlpha = arm_sin_f32(alpha);

  // Construct the transformation matrix following the DH convention
  transformMatrix[0] = cosTheta;
  transformMatrix[1] = -sinTheta * cosAlpha;
  transformMatrix[2] = sinTheta * sinAlpha;
  transformMatrix[3] = X_offset * cosTheta;

  transformMatrix[4] = sinTheta;
  transformMatrix[5] = cosTheta * cosAlpha;
  transformMatrix[6] = -cosTheta * sinAlpha;
  transformMatrix[7] = X_offset * sinTheta;

  transformMatrix[8] = 0;
  transformMatrix[9] = sinAlpha;
  transformMatrix[10] = cosAlpha;
  transformMatrix[11] = Z_offset;

  transformMatrix[12] = 0;
  transformMatrix[13] = 0;
  transformMatrix[14] = 0;
  transformMatrix[15] = 1;  // Homogeneous transformation matrix bottom row
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Sets the rotation matrix using Euler angles and translation offsets.
 *
 * This function generates a 4x4 homogeneous transformation matrix that applies 
 * rotations around X, Y, and Z axes followed by translations along the X, Y, and Z axes.
 *
 * @param transformMatrix Pointer to a 16-element array where the transformation matrix will be stored.
 * @param point position and orientation of a frame.
 */
void SetRotationTranslatgeMatrix(float32_t *transformMatrix, const TCP_point *point) {
  // Compute trigonometric values for efficiency
  float32_t cosRx = arm_cos_f32(point->rx);
  float32_t sinRx = arm_sin_f32(point->rx);
  float32_t cosRy = arm_cos_f32(point->ry);
  float32_t sinRy = arm_sin_f32(point->ry);
  float32_t cosRz = arm_cos_f32(point->rz);
  float32_t sinRz = arm_sin_f32(point->rz);

  // Construct the rotation and translation transformation matrix
  transformMatrix[0] = cosRz * cosRy;
  transformMatrix[1] = cosRz * sinRy * sinRx - sinRz * cosRx;
  transformMatrix[2] = cosRz * sinRy * cosRx + sinRz * sinRx;
  transformMatrix[3] = point->x;

  transformMatrix[4] = sinRz * cosRy;
  transformMatrix[5] = sinRz * sinRy * sinRx + cosRz * cosRx;
  transformMatrix[6] = sinRz * sinRy * cosRx - cosRz * sinRx;
  transformMatrix[7] = point->y;

  transformMatrix[8] = -sinRy;
  transformMatrix[9] = cosRy * sinRx;
  transformMatrix[10] = cosRy * cosRx;
  transformMatrix[11] = point->z;

  transformMatrix[12] = 0;
  transformMatrix[13] = 0;
  transformMatrix[14] = 0;
  transformMatrix[15] = 1;  // Homogeneous transformation matrix bottom row
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Sets the tool frame transformation matrix.
 *
 * This function initializes the tool frame transformation matrix using a specified
 * translation and rotation.
 *
 * @param point position and orientation of a frame.
 */
void set_tool_frame(const TCP_point *point) {
  // Compute the tool frame transformation matrix
  SetRotationTranslatgeMatrix(ToolFrame, point);

  // Initialize the ARM CMSIS-DSP matrix structure with the transformation matrix
  arm_mat_init_f32(&Tool_Matrix, MATRIX_SIZE, MATRIX_SIZE, ToolFrame);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Perform a homogeneous matrix inversion
 *
 * This function transposes the rotation part of the matrix and inverts the translation.
 *
 * @param Matrix Pointer to the 4x4 transformation matrix to be inverted.
 * @param invertedMatrix Pointer to the 4x4 inverted matrix.
 */
void HMinvertMatrix(float32_t *invertedMatrix, float32_t *Matrix) {

  // Transpose the rotation part (upper-left 3x3)
  invertedMatrix[0] = Matrix[0];
  invertedMatrix[1] = Matrix[4];
  invertedMatrix[2] = Matrix[8];
  invertedMatrix[3] = -(invertedMatrix[0] * Matrix[3] + invertedMatrix[1] * Matrix[7] + invertedMatrix[2] * Matrix[11]);

  invertedMatrix[4] = Matrix[1];
  invertedMatrix[5] = Matrix[5];
  invertedMatrix[6] = Matrix[9];
  invertedMatrix[7] = -(invertedMatrix[4] * Matrix[3] + invertedMatrix[5] * Matrix[7] + invertedMatrix[6] * Matrix[11]);

  invertedMatrix[8] = Matrix[2];
  invertedMatrix[9] = Matrix[6];
  invertedMatrix[10] = Matrix[10];
  invertedMatrix[11] = -(invertedMatrix[8] * Matrix[3] + invertedMatrix[9] * Matrix[7] + invertedMatrix[10] * Matrix[11]);

  // Invert the translation part
  invertedMatrix[12] = 0;
  invertedMatrix[13] = 0;
  invertedMatrix[14] = 0;
  invertedMatrix[15] = 1;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Prints a 4x4 transformation matrix.
 *
 * This function prints the elements of a 4x4 matrix stored as a linear array.
 *
 * @param matrix Pointer to a 16-element array representing the matrix.
 */
void printMatrix(float32_t *matrix) {
  Serial.printf("Matrix:\n");
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      Serial.printf("%.4f ", matrix[i * MATRIX_SIZE + j]);
    }
    Serial.printf("\n");
  }
  Serial.printf("\n\n");
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Prints an ARM CMSIS-DSP matrix instance.
 *
 * This function prints the elements of an ARM CMSIS-DSP matrix instance.
 *
 * @param matrix Pointer to an ARM CMSIS-DSP matrix instance.
 */
void printArmMatrix(const arm_matrix_instance_f32 *matrix) {
  if (matrix == NULL || matrix->pData == NULL) {
    Serial.println("Matrix is NULL or uninitialized!");
    return;
  }

  Serial.printf("Matrix [%dx%d]:\n", matrix->numRows, matrix->numCols);
  Serial.println("[");

  for (uint16_t i = 0; i < matrix->numRows; i++) {
    Serial.print("  [");
    for (uint16_t j = 0; j < matrix->numCols; j++) {
      Serial.printf("%8.4f", matrix->pData[i * matrix->numCols + j]);
      if (j < matrix->numCols - 1) Serial.print(", ");
    }
    Serial.println("]");
  }

  Serial.println("]\n");
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief get or update current robot kinematics state
 * 
 * @param pState pointer to read or write the the kinematics state
 * @param getOrUpdate set to UPDATE_CURRENT_KSTATE to update the kinematics state
 *                    set to GET_CURRENT_KSTATE to retrieve the kinematics state
 */
void robotKstate(kinematics_state *pState, int getOrUpdate) {
  static kinematics_state currentState = Home_pose;  // NOTE:  robot must be in home position when program starts!

  if (pState == NULL)  // safety
  {
    print_error(NULL_KSATE, __func__, __LINE__, "MED", "NULL kinematics_state pointer!");
    return;
  }

  if (getOrUpdate == UPDATE_CURRENT_KSTATE)
    currentState = *pState;
  else if (getOrUpdate == GET_CURRENT_KSTATE)
    *pState = currentState;
  else
    print_error(UNKMOWN_KSATE_VALUE, __func__, __LINE__, "MED", "Unknown value for getOrUpdate");
}
