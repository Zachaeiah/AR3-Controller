#ifndef KINEMATICS_H
#define KINEMATICS_H

//======================================== Include Libraries ========================================

#include "global.h"  // global varibles for program

//============================ Program Constants ======================================================================
#define MATRIX_SIZE 4
#define IK_SOL1 0
#define IK_SOL2 1

#define NUM_JOINTS 6
#define DELTA_THETA 0.0001f  // Small perturbation

enum CURRENT_ANGLES { GET_CURRENT_KSTATE,
                      UPDATE_CURRENT_KSTATE };  // used to get/update current angles

#define epsilon 1e-5

//======================================== DH Parameters ============================================
// Link 1 parameters
#define LINK1_ALFA -HALF_PI      // Alpha (twist angle) in radians (-90°)
#define LINK1_Z_OFFSET 169.770f  // Link offset in mm (d)
#define LINK1_X_OFFSET 64.20f    // Link length in mm (a)


// Link 2 parameters
#define LINK2_ALFA 0.0f        // Alpha (twist angle) in radians
#define LINK2_Z_OFFSET 0.0f    // Link offset in mm (d)
#define LINK2_X_OFFSET 305.0f  // Link length in mm (a)

// Link 3 parameters
#define LINK3_ALFA HALF_PI   // Alpha (twist angle) in radians (90°)
#define LINK3_Z_OFFSET 0.0f  // Link offset in mm (d)
#define LINK3_X_OFFSET 0.0f  // Link length in mm (a)


// Link 4 parameters
#define LINK4_ALFA -HALF_PI     // Alpha (twist angle) in radians (-90°)
#define LINK4_Z_OFFSET 222.63f  // Link offset in mm (d)
#define LINK4_X_OFFSET 0.0f     // Link length in mm (a)

// Link 5 parameters
#define LINK5_ALFA HALF_PI   // Alpha (twist angle) in radians (90°)
#define LINK5_Z_OFFSET 0.0f  // Link offset in mm (d)
#define LINK5_X_OFFSET 0.0f  // Link length in mm (a)

// Link 6 parameters
#define LINK6_ALFA 0.0f         // Alpha (twist angle) in radians
#define LINK6_Z_OFFSET 36.250f  // Link offset in mm (d)
#define LINK6_X_OFFSET 0.0f     // Link length in mm (a)

//======================================== Structure Definitions ====================================
/**
 * @brief Represents the joint angles of a 6-DOF robotic arm.
 */
typedef struct TCP_angles {
  float32_t theta1;  // Joint 1 angle in radians
  float32_t theta2;  // Joint 2 angle in radians
  float32_t theta3;  // Joint 3 angle in radians
  float32_t theta4;  // Joint 4 angle in radians
  float32_t theta5;  // Joint 5 angle in radians
  float32_t theta6;  // Joint 6 angle in radians
} TCP_angles;

/**
 * @brief forward kinematics solushion
 */
typedef struct FORWARD_SOLUTION {
  TCP_point point;
  arm_matrix_instance_f32* Jacobian;
  bool bCanReach = false;
} FORWARD_SOLUTION;

/**
 * @brief inverse kinematics solushion
 */
typedef struct INVERSE_SOLUTION {
  TCP_angles angles[2];  // two posible solshions
  bool bCanReach[2];
} INVERSE_SOLUTION;

/**
 * @brief Represents of the state of the robot configuration
 */
typedef struct kinematics_state {
  TCP_point TCP;
  TCP_angles Angles;
} kinematics_state;

// /**
//  * @brief Represents a 4x4 transformation matrix for kinematics calculations.
//  */
// typedef float32_t Matrix4x4[MATRIX_SIZE * MATRIX_SIZE];

//======================================== Globals ==================================================
extern kinematics_state Home_pose;
extern kinematics_state END_STOP_pose;
extern float32_t ToolFrame[MATRIX_SIZE * MATRIX_SIZE];
extern arm_matrix_instance_f32 Tool_Matrix;

//======================================== Function Prototypes ======================================
/**
 * @brief Computes inverse kinematics to obtain joint angles for a given TCP position and orientation.
 *
 * @param point The desired end-effector position and orientation (TCP_point).
 * @return FORWARD_SOLUTION The computed joint angles required to reach the specified TCP position.
 */
INVERSE_SOLUTION INVERSE_KINEMATICS(const TCP_point point);

/**
 * @brief Computes forward kinematics to determine the TCP position from given joint angles.
 *
 * @param angles The joint angles of the robotic arm (TCP_angles).
 * @param bJacobian if true will only calqulate the Jacobia, else it will do both FK and bJacobian
 * @return FORWARD_SOLUTION The computed TCP position and orientation.
 */
FORWARD_SOLUTION FORWARD_KINEMATICS(const TCP_angles angles, bool bJacobian, arm_matrix_instance_f32* jacobian);

/**
 * @brief Computes the numeric Jacobian matrix for the robotic arm.
 *
 * @param 
 * @param J The Jacobian matrix (3x6) to be computed.
 */
void compute_jacobian(const arm_matrix_instance_f32 *T1, const arm_matrix_instance_f32 *T2,
                      const arm_matrix_instance_f32 *T3, const arm_matrix_instance_f32 *T4,
                      const arm_matrix_instance_f32 *T5, const arm_matrix_instance_f32 *T6,
                      arm_matrix_instance_f32 *jacobian);

void set_jacobian_column(arm_matrix_instance_f32 *J, int col,
                         const float32_t *jv, const float32_t *jw);

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
  void computeTransformMatrix(float32_t *transformMatrix, const float32_t theta, const float32_t alpha, const float32_t X_offset, const float32_t Z_offset);

/**
 * @brief Sets the rotation matrix using Euler angles and translation offsets.
 *
 * This function generates a 4x4 homogeneous transformation matrix that applies 
 * rotations around X, Y, and Z axes followed by translations along the X, Y, and Z axes.
 *
 * @param transformMatrix Pointer to a 16-element array where the transformation matrix will be stored.
 * @param point position and orientation of a frame.
 */
void SetRotationTranslatgeMatrix(float32_t *transformMatrix, const TCP_point *point);

/**
 * @brief Sets the tool frame transformation matrix.
 *
 * This function initializes the tool frame transformation matrix using a specified
 * translation and rotation.
 *
 * @param point position and orientation of a frame.
 */
void set_tool_frame(const TCP_point *point);
/**
 * @brief Perform a homogeneous matrix inversion
 *
 * This function transposes the rotation part of the matrix and inverts the translation.
 *
 * @param Matrix Pointer to the 4x4 transformation matrix to be inverted.
 * @param invertedMatrix Pointer to the 4x4 inverted matrix.
 */
void HMinvertMatrix(float32_t *invertedMatrix, float32_t *Matrix);


/**
 * @brief Prints a 4x4 transformation matrix.
 *
 * This function prints the elements of a 4x4 matrix stored as a linear array.
 *
 * @param matrix Pointer to a 16-element array representing the matrix.
 */
void printMatrix(float32_t *matrix);

/**
 * @brief Prints an ARM CMSIS-DSP matrix instance.
 *
 * This function prints the elements of an ARM CMSIS-DSP matrix instance.
 *
 * @param matrix Pointer to an ARM CMSIS-DSP matrix instance.
 */
void printArmMatrix(const arm_matrix_instance_f32 *matrix);

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief get or update current robot angles
 * 
 * @param pState pointer to read or write the the kinematics state
 * @param getOrUpdate set to UPDATE_CURRENT_KSTATE to update the kinematics state
 *                    set to GET_CURRENT_KSTATE to retrieve the kinematics state
 */
void robotKstate(kinematics_state *pState, int getOrUpdate);

void cross_product(const float32_t a[3], const float32_t b[3], float32_t result[3]);

void vector_subtract(const float32_t a[3], const float32_t b[3], float32_t result[3]);

#endif  // KINEMATICS_H
