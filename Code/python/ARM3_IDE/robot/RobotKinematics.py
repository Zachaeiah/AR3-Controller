import numpy as np
from typing import List, Optional, Union
import logging
from robot.RobotArmConfig import RobotArmConfig 
from robot.MotorLink import MotorLink
from robot.ToolLink import ToolLink



class RobotKinematics:
    
    def __init__(self, arm_config: RobotArmConfig, logger: Optional[logging.Logger] = None):
        """_summary_

        Args:
            arm_config (RobotArmConfig): _description_
            logger (Optional[logging.Logger], optional): _description_. Defaults to None.
        
        Note:
            [w, x, y, z] → [x, y, z, w]
            qx = np.array([qw, qx, qy, qz])
            qx_xyzw = np.concatenate((qx[1:], [qx[0]]))
        
        """

        self.arm_config: RobotArmConfig = arm_config
        self.links: List[Union[MotorLink, ToolLink]] = self.arm_config.links
        self.logger = logger

    def DH_matrix(self, angle: float, link_params: dict[str, float]) -> np.ndarray[float, float]:

        """
        Create the Denavit-Hartenberg transformation matrix for a single joint/link.

        Args:
            angle (float): Joint angle θ (in radians).
            link_params (dict[str, float]): Dictionary with DH parameters:
                - a: Link length (along x-axis)
                - alpha: Link twist (in radians, about x-axis)
                - d: Link offset (along z-axis)

        Returns:
            np.ndarray: 4x4 Denavit-Hartenberg transformation matrix
        """
        a: float = link_params.get("a", 0.0)
        alpha: float = link_params.get("alpha", 0.0)
        d:float = link_params.get("d", 0.0)
        angle_offset:float = link_params.get("Input_angle_offset", 0.0)

        ca: float = np.cos(alpha)
        sa: float = np.sin(alpha)
        ct: float = np.cos(angle + angle_offset)
        st: float = np.sin(angle + angle_offset)

        DenaHartMatrix: np.ndarray[float,float] = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,      ca,      d],
            [0,        0,       0,      1]
        ])

        return DenaHartMatrix
    
    def rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        Convert a 3x3 rotation matrix into a quaternion.

        Args:
            rotation_matrix (np.array): A 3x3 rotation matrix.

        Returns:
            np.array: A 4-element array representing the quaternion.
        """
        R:np.ndarray = np.array(rotation_matrix, dtype=np.float64)
        
        trace:float = np.trace(R)

        S:float = 0
        qw:float = 0
        qx:float = 0
        qy:float = 0
        qz:float = 0

        
        if trace > 0:
            S = 2.0 * np.sqrt(trace + 1.0)
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        
        quaternion = np.array([qw, qx, qy, qz])

        return quaternion
    
    def quaternion_to_rotation_matrix(self, quaternion: np.ndarray) -> np.ndarray:
        """
        Convert a quaternion into a 3x3 rotation matrix.

        Args:
            quaternion (list or np.array): A 4-element list or array representing the quaternion.

        Returns:
            np.array: A 3x3 rotation matrix.
        """
        q:np.ndarray  = np.array(quaternion, dtype=np.float64)
        q = q / np.linalg.norm(q)

        w, x, y, z = q
        rotation_matrix: np.ndarray = np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
                                    [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
                                    [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]])

        return rotation_matrix
    
    def quaternion_to_euler(self, quaternion):
        """Convert quaternion to Euler angles.

        Args:
            quaternion (np.array): Quaternion in the form [w, x, y, z].

        Returns:
            np.array: Euler angles in the form [roll, pitch, yaw].
        """
        w, x, y, z = quaternion
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use +/-90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])
        
    def FK(self, angles: np.ndarray) -> tuple[np.ndarray, list[np.ndarray], list[np.ndarray]]:
        """
        Perform forward kinematics using the given joint angles and return both TCP pose and intermediate matrices.

        Args:
            angles (np.ndarray): Array of joint angles [θ1, θ2, ..., θn] in radians.

        Returns:
            tuple:
                - np.ndarray: TCP pose as [X, Y, Z, qw, qx, qy, qz]
                - list[np.ndarray]: List of intermediate transformation matrices (each A_i)
                - list[np.ndarray]: List of transformation  (each T_current)
        """
        if len(angles) != (len(self.arm_config.links) - 1):
            raise ValueError("Length of angles does not match number of robot links.")

        T_current = np.identity(4, dtype=np.float64)
        T_all: list[np.ndarray] = []
        T_tranform: list[np.ndarray] = []

        for i, link in enumerate(self.arm_config.links):
            if isinstance(link, ToolLink):
                A_i = link.to_matrix()
            else:
                theta = angles[i]
                link_params = link.DenaHar_params
                A_i = self.DH_matrix(theta, link_params)

            
            T_current = T_current @ A_i
            T_tranform.append(A_i.copy())
            T_all.append(T_current.copy())

            if self.logger:
                self.logger.debug(f"Link {i}: A_i =\n{np.round(A_i, 4)}")
                self.logger.debug(f"T (base to link {i}):\n{np.round(T_current, 4)}")

        x, y, z = T_current[0:3, 3]
        R = T_current[0:3, 0:3]
        tcp_quat = self.rotation_matrix_to_quaternion(R)

        tcp_pose = np.concatenate(([x, y, z], tcp_quat))

        return tcp_pose, T_tranform, T_all
    
    def IK (self, pose: np.ndarray[float]) -> tuple[np.ndarray, np.ndarray]:
        """
        Perform inverse kinematics given a 6-DOF TCP pose.

        Args:
            pose (np.ndarray): TCP pose as [X, Y, Z, qw, qx, qy, qz]

        Returns:
            tuple[np.ndarray, np.ndarray]: Two possible IK solutions (in radians)
        """

        # Build TCP transform matrix
        T_tcp = np.identity(4)
        T_tcp[:3, :3] = self.quaternion_to_rotation_matrix(pose[3:])
        T_tcp[:3, 3] =  pose[:3]

        if self.logger:
            self.logger.debug("TCP Transform:\n" + np.array2string(T_tcp, formatter={'float_kind': lambda x: f"{x:8.4f}"}))

        # Extract tool frame and remove it from TCP pose
        tool_frame = self.links[-1].to_matrix()
        inv_tool_frame = np.linalg.inv(tool_frame)
        T_06 = T_tcp @ inv_tool_frame

        # Remove final wrist link to get frame 5
        T_65_inv = np.linalg.inv(self.DH_matrix(0, self.links[-2].DenaHar_params))
        T_05 = T_06 @ T_65_inv

        sol1 = np.zeros(6)
        sol2 = np.zeros(6)

        # ----- Solve Joint 1 -----
        sol1[0] = sol2[0] = np.arctan2(T_05[1, 3], T_05[0, 3])

        # ----- Geometry for Joint 2 and 3 -----
        x_prime = T_05[0, 3]*np.cos(-sol1[0]) - T_05[1, 3]*np.sin(-sol1[0])
        z_prime = T_05[2, 3] - self.links[0].DenaHar_params.get("d", 0)

        a1 = self.links[0].DenaHar_params.get("a", 0)
        a2 = self.links[1].DenaHar_params.get("a", 0)
        a3 = self.links[2].DenaHar_params.get("a", 0)
        d4 = self.links[3].DenaHar_params.get("d", 0)

        l1 = abs(x_prime - a1)
        l2 = np.hypot(l1, z_prime)
        l3 = np.hypot(a3, d4)

        thetaB = np.arctan2(l1, z_prime)
        thetaC = np.arccos(np.clip((a2**2 + l2**2 - l3**2) / (2 * a2 * l2), -1.0, 1.0))
        thetaD = np.arccos(np.clip((l3**2 + a2**2 - l2**2) / (2 * l3 * a2), -1.0, 1.0))
        thetaE = np.arctan2(a3, d4)

         # ----- Solve Joint 2 -----
        if x_prime > a1:
            sol1[1] = thetaB - thetaC if z_prime > 0 else thetaB - thetaC + np.pi
        else:
            sol1[1] = -(thetaB + thetaC)

        sol2[1] = sol1[1]

        # ----- Solve Joint 3 -----
        sol1[2] = sol2[2] = -(thetaD + thetaE) + np.pi / 2

        # ----- Orientation (Joints 4, 5, 6) -----
        A_1 = self.DH_matrix(sol1[0], self.links[0].DenaHar_params)
        A_2 = self.DH_matrix(sol1[1], self.links[1].DenaHar_params)
        A_3 = self.DH_matrix(sol1[2], self.links[2].DenaHar_params)

        T_03 = A_1 @ A_2 @ A_3
        R_03_T = T_03[:3, :3].T  # Transpose to invert rotation

        R_36 = R_03_T @ T_06[:3, :3]

        # ----- Solve Joint 5 -----
        temp = np.clip(R_36[2, 2], -1.0, 1.0)
        sol1[4] = np.arctan2(np.sqrt(1 - temp**2), temp)
        sol2[4] = np.arctan2(-np.sqrt(1 - temp**2), temp)

        # ----- Solve Joint 4 -----
        sol1[3] = np.arctan2(R_36[1, 2], R_36[0, 2])
        sol2[3] = np.arctan2(-R_36[1, 2], -R_36[0, 2])

        # ----- Solve Joint 6 -----
        sol1[5] = np.arctan2(R_36[2, 1], -R_36[2, 0])
        sol2[5] = np.arctan2(-R_36[2, 1], R_36[2, 0])

        return sol1, sol2
    

    
    def verify_Kinematics(self, pose: np.ndarray, FK_IK: bool = bool) -> list[tuple[bool, Optional[int]]]:
        """
        Verify that joint angles are within mechanical limits.

        Args:
            pose (np.ndarray): Either joint angles (FK) or a TCP pose (IK).
            FK_IK (bool): True to verify IK solutions; False to verify FK pose.

        Returns:
            list[tuple[bool, Optional[int]]]: 
                For FK: [(is_valid, failing_joint_index), (is_valid, failing_joint_index)]
                For IK: same format, one for each IK solution.
                If valid, failing_joint_index is None.
        """

        def check_joint_limits(joint_angles: np.ndarray) -> tuple[bool, Optional[int]]:
            """
            Checks whether the provided joint angles are within the allowed limits
            defined for each joint in the robot arm configuration.

            Args:
                joint_angles (np.ndarray): An array of joint angles to validate, 
                                        typically the output of FK or IK.

            Returns:
                tuple[bool, Optional[int]]: 
                    - A boolean indicating whether all joint angles are within their limits.
                    - The index of the first joint that violates its limit (if any), or None if all are valid.
            """
            for i, (angle, link) in enumerate(zip(joint_angles, self.arm_config.links)):
                min_angle = link.motor_params["min_angle"]
                max_angle = link.motor_params["max_angle"]
                if not (min_angle < angle < max_angle):
                    if self.logger:
                        self.logger.error(
                            f"Joint {i} angle {angle:.2f} out of range [{min_angle}, {max_angle}]"
                        )
                    return False, i
            return True, None

        if FK_IK:
            valid, bad_index = check_joint_limits(pose)
            return [(valid, bad_index), (valid, bad_index)]
        else:
            joint_pose1, joint_pose2 = self.IK(pose)
            result1 = check_joint_limits(joint_pose1)
            result2 = check_joint_limits(joint_pose2)
            return [result1, result2]

    @staticmethod
    def print_matrices(matrices: list[np.ndarray]) -> None:
        """
        Nicely print a list of 4x4 matrices with 4 decimal places.
        """
        for i, mat in enumerate(matrices):
            print(f"A[{i + 1}]:")
            print(np.array2string(mat, formatter={'float_kind': lambda x: f"{x:8.4f}"}))
            print()