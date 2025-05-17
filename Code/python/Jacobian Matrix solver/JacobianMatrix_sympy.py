import sympy as sp
import matplotlib.pyplot as plt

def dh_transform(a, alpha, d, theta):
    """
    Compute the homogeneous transformation matrix using DH parameters.
    
    Parameters:
    a     : Link length (distance along x-axis)
    alpha : Link twist (angle around x-axis)
    d     : Link offset (distance along z-axis)
    theta : Joint angle (rotation around z-axis)
    
    Returns:
    T : 4x4 symbolic transformation matrix
    """
    # # Define symbolic variables if necessary
    # a, alpha, d, theta = sp.symbols(f'{a} {alpha} {d} {theta}')
    
    # Compute the transformation matrix
    T = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,             sp.sin(alpha),               sp.cos(alpha),               d],
        [0,             0,                            0,                            1]
    ])
    
    return T


class KINAMATICS:
    def __init__(self):
        self.alpha1, self.d1, self.a1 = -sp.pi/2 , 169.770, 64.2
        self.alpha2, self.d2, self.a2 = 0.0             , 0.0,     305.0
        self.alpha3, self.d3, self.a3 = sp.pi/2  , 0.0,     0.0        
        self.alpha4, self.d4, self.a4 = -sp.pi/2 , 222.630, 0.0
        self.alpha5, self.d5, self.a5 = sp.pi/2  , 0.0,     0.0      
        self.alpha6, self.d6, self.a6 = 0.0             , 36.250,  0.0 

    def FK(self, joints):
        """
        Compute the forward kinematics of the robot given the joint angles.
        """
        # Define joint angles from input
        theta1, theta2, theta3, theta4, theta5, theta6 = joints

        # Compute transformation matrices for each joint
        J1 = dh_transform(theta1, self.alpha1, self.d1, self.a1)
        J2 = dh_transform(theta2 - sp.pi/2, self.alpha2, self.d2, self.a2)
        J3 = dh_transform(theta3 + sp.pi, self.alpha3, self.d3, self.a3)
        J4 = dh_transform(theta4, self.alpha4, self.d4, self.a4)
        J5 = dh_transform(theta5, self.alpha5, self.d5, self.a5)
        J6 = dh_transform(theta6, self.alpha6, self.d6, self.a6)

        # Calculate the overall transformation matrix from base to end-effector
        T = J1 * J2 * J3 * J4 * J5 * J6

        # Extract position (TCP)
        tcp_pos = T[:3, 3]

        # Extract rotation (R)
        rotation = T[:3, :3]

        return tcp_pos, rotation
    
    def Get_jacobian_matrices(self, joints):
        """
        Compute the Jacobian matrix symbolically for the 6-DOF robot.
        """
        # Forward kinematics computation
        tcp_pos, rotation = self.FK(joints)

        # Initialize Jacobian matrix
        jacobian = sp.zeros(6, 6)

        # Position of the end effector
        position_06 = tcp_pos

        # Compute Jacobian for each joint (theta1 to theta6)
        for i in range(6):
            # Each joint contributes linear and angular velocities
            theta = joints[i]

            # Rotation axis (z-axis) for each joint (assuming each joint rotates around its own z-axis)
            z_axis = rotation[:, 2]  # R_z for each joint

            # Compute the linear velocity part (cross product of z_axis and position vector)
            jacobian[:3, i] = sp.Matrix([0, 0, 1]).cross(position_06)

            # Compute the angular velocity part (just the z-axis for each joint)
            jacobian[3:6, i] = z_axis

        return jacobian

        

# Create an instance of the KINAMATICS class
kinematics = KINAMATICS()

# Example joint angles (in radians)
joints = [sp.symbols('theta1'), sp.symbols('theta2'), sp.symbols('theta3'), 
          sp.symbols('theta4'), sp.symbols('theta5'), sp.symbols('theta6')]

# Get the Jacobian matrix
jacobian_matrix = kinematics.Get_jacobian_matrices(joints)

# Define the end-effector velocity (linear and angular velocities as a 6D vector)
dot_X = sp.Matrix([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Example velocity vector

# Use the pseudo-inverse (in case the Jacobian is singular or near-singular)
jacobian_pinv = jacobian_matrix.pinv()

print()

# # Calculate the joint velocities using the pseudo-inverse
# dot_theta = jacobian_pinv * dot_X

# # Substitute numerical values for the joint angles (for example, theta1=0, theta2=pi/4, etc.)
# numerical_dot_theta = [dot_theta[i].subs({joints[0]: 0, joints[1]: sp.pi/4, joints[2]: sp.pi/2, 
#                                            joints[3]: -sp.pi/3, joints[4]: sp.pi/6, joints[5]: -sp.pi/2})
#                        for i in range(6)]

# # Plot the joint velocities
# joint_indices = [1, 2, 3, 4, 5, 6]  # Joint indices (theta1 to theta6)
# plt.plot(joint_indices, numerical_dot_theta, marker='o', linestyle='-', color='b')

# # Add labels and title
# plt.title("Joint Velocities vs Joint Indices")
# plt.xlabel("Joint Index")
# plt.ylabel("Joint Velocity (rad/s or m/s)")
# plt.xticks(joint_indices)
# plt.grid(True)

# # Show the plot
# plt.show()

# Show the plot