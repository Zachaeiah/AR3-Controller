
## Overview

The `kinematics` library provides functions for computing forward and inverse kinematics for a 6-degree-of-freedom (6-DOF) robotic arm. It also includes utility functions for Jacobian computation, transformation matrix calculations, and tool frame transformations.

## Constants

- **NUM_JOINTS** (`6`): Number of joints in the robotic arm.
- **DELTA_THETA** (`0.0001f`): Small perturbation used in numerical calculations.
## Structure Definitions

### `TCP_angles`

```c
typedef struct TCP_angles {
  float32_t theta1;
  float32_t theta2;
  float32_t theta3;
  float32_t theta4;
  float32_t theta5;
  float32_t theta6;
} TCP_angles;
```

Represents joint angles of the robotic arm in radians.
### `FORWARD_SOLUTION`

```c
typedef struct FORWARD_SOLUTION {
  TCP_point point;
  arm_matrix_instance_f32* Jacobian;
  bool bCanReach;
} FORWARD_SOLUTION;
```

Stores forward kinematics results, including the computed TCP position and Jacobian matrix.
### `INVERSE_SOLUTION`

```c
typedef struct INVERSE_SOLUTION {
  TCP_angles angles[2];
  bool bCanReach[2];
} INVERSE_SOLUTION;
```

Stores two possible inverse kinematics solutions, indicating whether the target position is reachable.
## Function Implementations

### `INVERSE_KINEMATICS`

```c
INVERSE_SOLUTION INVERSE_KINEMATICS(const TCP_point point);
```

Computes inverse kinematics to obtain joint angles for a given TCP position.

#### Parameters:
- `point`: Desired end-effector position and orientation.
#### Returns:
- `INVERSE_SOLUTION`: Contains possible joint angles and reachability flags.

---

### `FORWARD_KINEMATICS`

```c
FORWARD_SOLUTION FORWARD_KINEMATICS(const TCP_angles angles, bool bJacobian);
```

Computes forward kinematics to determine the TCP position from given joint angles.

#### Parameters:
- `angles`: Joint angles of the robotic arm.
- `bJacobian`: If `true`, only the Jacobian is calculated; otherwise, both FK and the Jacobian are computed.

#### Returns:
- `FORWARD_SOLUTION`: Contains the computed TCP position and Jacobian.
    

---

### `compute_jacobian`

```c
void compute_jacobian(arm_matrix_instance_f32 *T1, arm_matrix_instance_f32 *T2,
                      arm_matrix_instance_f32 *T3, arm_matrix_instance_f32 *T4,
                      arm_matrix_instance_f32 *T5, arm_matrix_instance_f32 *T6,
                      arm_matrix_instance_f32 *jacobian);
```

Computes the numerical Jacobian matrix.
#### Parameters:
- `T1-T6`: Transformation matrices for each joint.
- `jacobian`: Output Jacobian matrix.
    

---

### `computeTransformMatrix`

```c
void computeTransformMatrix(float32_t* transformMatrix, float32_t theta, float32_t alpha, float32_t X_offset, float32_t Z_offset);
```
Computes a transformation matrix using Denavit-Hartenberg (DH) parameters.
#### Parameters:
- `transformMatrix`: Output transformation matrix.
- `theta`: Rotation angle around the Z-axis.
- `alpha`: Rotation angle around the X-axis.
- `X_offset`: Translation along the X-axis.
- `Z_offset`: Translation along the Z-axis.

---

### `set_tool_frame`

```c
void set_tool_frame(const TCP_point* point);
```

Sets the tool frame transformation matrix.
#### Parameters:
- `point`: Position and orientation of the tool frame.

---

### `HMinvertMatrix`

```c
void HMinvertMatrix(float32_t* invertedMatrix, float32_t* Matrix);
```
Computes the inverse of a homogeneous transformation matrix.

#### Parameters:
- `Matrix`: Input 4x4 transformation matrix.
- `invertedMatrix`: Output inverted matrix.

---

## Future Improvements
- Improve inverse kinematics to handle singularities.
- Optimize Jacobian computation for real-time control.
- Implement additional transformation utilities for various kinematic configurations.