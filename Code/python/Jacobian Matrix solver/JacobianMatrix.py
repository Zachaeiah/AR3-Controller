import numpy as np
import matplotlib.pyplot as plt

def dh_transform(theta:float, alpha:float, d:float, a:float):
    """
    Compute the homogeneous transformation matrix using DH parameters.
    
    Parameters
    ---
    a     : Link length (distance along x-axis)
    alpha : Link twist (angle around x-axis) in radians
    d     : Link offset (distance along z-axis)
    theta : Joint angle (rotation around z-axis) in radians
    
    Returns
    ---
    T : 4x4 NumPy transformation matrix
    """
    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])
    return T

def euler_to_rotation(alpha, beta, gamma):
    """
    Compute the 3D rotation matrix from Euler angles (ZYX order).
    
    Parameters:
    alpha : Rotation around X-axis (radians)
    beta  : Rotation around Y-axis (radians)
    gamma : Rotation around Z-axis (radians)
    
    Returns:
    R : 3x3 NumPy rotation matrix
    """
    # Compute cosines and sines of the angles
    ca, cb, cg = np.cos(alpha), np.cos(beta), np.cos(gamma)
    sa, sb, sg = np.sin(alpha), np.sin(beta), np.sin(gamma)
    
    # Construct the rotation matrix
    R = np.array([
        [ca * cb,  ca * sb * sg - sa * cg,  ca * sb * cg + sa * sg],
        [sa * cb,  sa * sb * sg + ca * cg,  sa * sb * cg - ca * sg],
        [-sb,      cb * sg,                cb * cg]
    ])
    
    return R

def to_homogeneous(R, xyz):
    """
    Convert a 3x3 rotation matrix and a translation vector into a 4x4 homogeneous transformation matrix.

    Parameters:
    R   : 3x3 NumPy array (rotation matrix)
    xyz : List or array-like [x, y, z] (translation vector)

    Returns:
    T : 4x4 NumPy array (homogeneous transformation matrix)
    """
    # Ensure R is 3x3 and xyz is a list of length 3
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"
    assert len(xyz) == 3, "Translation vector must have three elements"

    # Construct the 4x4 transformation matrix
    T = np.eye(4)  # Start with an identity matrix
    T[:3, :3] = R  # Set the rotation part
    T[:3, 3] = xyz  # Set the translation part

    return T

def parametric_line_3d(start, end, velocity):
    """
    Returns a function that gives the parametric position of a 3D line.
    
    :param start: Tuple (x0, y0, z0) representing the starting point.
    :param end: Tuple (x1, y1, z1) representing the ending point.
    :param velocity: Scalar velocity along the line.
    :return: Function position(t) that returns (x, y, z) at time t.
    """
    start = np.array(start)
    end = np.array(end)
    direction = end - start
    unit_direction = direction / np.linalg.norm(direction)  # Normalize direction
    speed_vector = velocity * unit_direction
    
    def position(t):
        return tuple(start + speed_vector * t)
    
    return position

class KINAMATICS():
    def __init__(self, JT):
        self.alpha1, self.d1, self.a1 = np.radians(-90) , 169.770, 64.2
        self.alpha2, self.d2, self.a2 = 0.0             , 0.0,     305.0
        self.alpha3, self.d3, self.a3 = np.radians(90)  , 0.0,     0.0        
        self.alpha4, self.d4, self.a4 = np.radians(-90) , 222.630, 0.0
        self.alpha5, self.d5, self.a5 = np.radians(90)  , 0.0,     0.0      
        self.alpha6, self.d6, self.a6 = 0.0             , 36.250,  0.0 

        self.JT = JT
        self.TCP = np.zeros(6)

    def FK(self, joints)->list:
        
        # Compute transformation matrices for each joint
        self.J1 = dh_transform(joints[0], self.alpha1, self.d1, self.a1)
        self.J2 = dh_transform(joints[1] - np.radians(90), self.alpha2, self.d2, self.a2 )
        self.J3 = dh_transform(joints[2] + np.radians(180), self.alpha3, self.d3, self.a3)
        self.J4 = dh_transform(joints[3], self.alpha4, self.d4, self.a4)
        self.J5 = dh_transform(joints[4], self.alpha5, self.d5, self.a5)
        self.J6 = dh_transform(joints[5], self.alpha6, self.d6, self.a6)

        self.R_02 = np.dot(self.J1,self.J2)
        self.R_03 = np.dot(self.R_02,self.J3)
        self.R_04 = np.dot(self.R_03,self.J4)
        self.R_05 = np.dot(self.R_04,self.J5)
        self.R_06 = np.dot(self.R_05,self.J6)
        self.R_0T = np.dot(self.R_06,self.JT)

        self.TCP[0] = self.R_0T[0][3] # X cor
        self.TCP[1] = self.R_0T[1][3] # Y cor
        self.TCP[2] = self.R_0T[2][3] # Z cor

        # print(np.sqrt(R_0T[2][1]**2 + R_0T[2][2]**2), -R_0T[2][0])
        self.TCP[4] = np.arctan2(-self.R_0T[2][0], np.sqrt(self.R_0T[2][1]**2 +self.R_0T[2][2]**2)) # R_Y rot
        self.TCP[3] = np.arctan2(self.R_0T[2][1]/np.cos(self.TCP[4]), self.R_0T[2][2]/np.cos(self.TCP[4])) # R_X rot
        self.TCP[5] = np.arctan2(self.R_0T[1][0]/np.cos(self.TCP[4]), self.R_0T[0][0]/np.cos(self.TCP[4])) # R_Z rot

        return self.TCP

    def Get_jacobian_matrices(self):


        self.jacobian = np.zeros((6, 6))

        position_06 = self.R_06[:3, 3]  # End-effector position

        # Compute JM for theta1 (base joint, rotating around z-axis)
        cross = np.cross(np.array([0, 0, 1]), position_06)
        self.jacobian[:3, 0] = cross
        self.jacobian[3:6, 0] = np.array([0, 0, 1])

        # Compute JM for theta2
        self.jacobian[:3, 1] = np.cross(self.R_02[:3, 2], position_06 - self.R_02[:3, 3])
        self.jacobian[3:6, 1] = self.R_02[:3, 2]

        # Compute JM for theta3
        self.jacobian[:3, 2] = np.cross(self.R_03[:3, 2], position_06 - self.R_03[:3, 3])
        self.jacobian[3:6, 2] = self.R_03[:3, 2]

        # Compute JM for theta4
        self.jacobian[:3, 3] = np.cross(self.R_04[:3, 2], position_06 - self.R_04[:3, 3])
        self.jacobian[3:6, 3] = self.R_04[:3, 2]

        # Compute JM for theta5
        self.jacobian[:3, 4] = np.cross(self.R_05[:3, 2], position_06 - self.R_05[:3, 3])
        self.jacobian[3:6, 4] = self.R_05[:3, 2]

        # Compute JM for theta6 (fixing the incorrect reference to R_0T)
        self.jacobian[:3, 5] = np.cross(self.R_06[:3, 2], position_06 - self.R_06[:3, 3])
        self.jacobian[3:6, 5] = self.R_06[:3, 2]

        return self.jacobian
    
    
    def IK(self, TCP)->list:

        joints = np.zeros((2,6))

        R_0T =  to_homogeneous(euler_to_rotation(TCP[3], TCP[4], TCP[5]), [TCP[0],TCP[1],TCP[2]])
        self.JT_inv = np.linalg.inv(self.JT)

        R_06 = np.dot(R_0T, self.JT_inv)
        R_06_neg = np.eye(4)
        R_06_neg[2][3] = -self.d6
        R_05 = np.dot(R_06, R_06_neg)

        joints[0][0]=np.arctan2(R_05[1][3],R_05[0][3])
        X_prime = R_05[0][3]*np.cos(-joints[0][0]) - R_05[1][3]*np.sin(-joints[0][0])
        L1 = np.abs(X_prime - self.a1)
        L3 = np.sqrt(self.d4**2 + self.a3**2)
        L4 = R_05[2][3] - self.d1
        L2 = np.sqrt(L1**2 + L4**2)

        OB = np.arctan(L1/L4)
        OC = np.arccos((self.a2**2 + L2**2 - L3**2 )/(2*self.a2*L2))
        OD = np.arccos((L3**2 + self.a2**2  - L2**2 )/(2*self.a2*L3))
        OE = np.arctan(self.a3/self.d4)

        if (X_prime > self.a1):
            if (L4 > 0):
                joints[0][1] = OB - OC
            else:
                joints[0][1] = OB - OC + np.pi
        else:
            joints[0][1] = -(OB + OC) 

        joints[0][2] = -(OD + OE) + np.pi/2
        joints[1] = joints[0]

        # Compute transformation matrices for each joint
        J1 = dh_transform(joints[0][0], self.alpha1, self.d1, self.a1)
        J2 = dh_transform(joints[0][1] - np.radians(90), self.alpha2, self.d2, self.a2 )
        J3 = dh_transform(joints[0][2] + np.radians(180), self.alpha3, self.d3, self.a3)

        R_02 = np.dot(J1,J2)
        R_03 = np.dot(R_02,J3)


        R_03_t = np.transpose(R_03)[:3, :3]
        R_36 = np.dot(R_03_t, R_06[:3, :3])

        joints[0][3] = -np.arctan2(R_36[1][2], -R_36[0][2])
        joints[1][3] = -np.arctan2(-R_36[1][2], R_36[0][2])

        joints[0][4] = np.arctan2(-np.sqrt(1-R_36[2][2]**2), R_36[2][2])
        joints[1][4] = np.arctan2(np.sqrt(1-R_36[2][2]**2), R_36[2][2])

        joints[0][5] = np.arctan2(-R_36[2][1], R_36[2][0])
        joints[1][5] = np.arctan2(R_36[2][1], -R_36[2][0])

        return joints




np.set_printoptions(precision=4)  # Format output

TOOL_FRAME = to_homogeneous(euler_to_rotation(0.0, 0.0, 0.0), [0.0, 0.0, 0.0])  # Homogeneous transformation of the tool frame
Solver = KINAMATICS(TOOL_FRAME)

# # Example Usage
# start_point = (80, 100, 93)
# end_point = (-80, 81, 82)
# velocity = 2
# distance = np.linalg.norm(np.array(end_point) - np.array(start_point))
# T = distance / velocity
# position_func = parametric_line_3d(start_point, end_point, velocity)

# ts = np.linspace(0, T, 100)  # Generate time values
# positions = np.array([position_func(t) for t in ts])


Solver.FK([0.0348, 0.0013, 0.0436, 0.6599, -0.0575, -0.6591])  # Forward Kinematics
jacobian = Solver.Get_jacobian_matrices()  # Compute Jacobian
print("jacobian")
print(jacobian)
print("inf jacobian")
print(np.linalg.inv(jacobian))
# print( np.sqrt( np.linalg.det(jacobian)))

# # Ensure correct orientation (assume zero rotation)
# orientations = np.zeros((positions.shape[0], 3))
# positions = np.hstack((positions, orientations))

# joint_velocities = []

# for pos in positions:
#     joints = Solver.IK(pos)  # Inverse Kinematics
    
#     if joints is None:
#         print("No IK solution found for", pos)
#         continue
    
#     Solver.FK(joints[0])  # Forward Kinematics
#     jacobian = Solver.Get_jacobian_matrices()  # Compute Jacobian
    
#     xdot = [velocity, velocity, velocity, 1, 1, 1]  # Example velocity vector
#     qdot = np.linalg.pinv(jacobian) @ xdot  # Use pseudoinverse
#     #qdot = np.linalg.pinv(jacobian)
    
#     joint_velocities.append(qdot)

# joint_velocities = np.array(joint_velocities)
# joint_velocities = np.degrees(joint_velocities)  # Convert rad/s to deg/s


# # Plotting
# plt.figure(figsize=(10, 6))

# for joint_index in range(joint_velocities.shape[1]):
#     plt.plot(ts[:len(joint_velocities)], joint_velocities[:, joint_index], label=f'Joint {joint_index+1}')

# plt.title('Joint Velocities Over Time')
# plt.xlabel('Time (s)')
# plt.ylabel('Joint Velocity (deg/s)')
# plt.legend()
# plt.grid(True)
# plt.show()

