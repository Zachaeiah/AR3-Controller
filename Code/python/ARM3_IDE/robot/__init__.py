"""
robot: A Python package for robotic arm configuration and kinematics.

Modules:
- RobotArmConfig: Configuration manager for the robot arm.
- RobotKinematics: Handles FK, IK, and Jacobian calculations.
- MotorLink: Represents individual motor/link pairs.
- ToolLink: Represents the tool/end-effector link.
"""

from robot.RobotArmConfig import RobotArmConfig
from robot.RobotKinematics import RobotKinematics
from robot.MotorLink import MotorLink
from robot.ToolLink import ToolLink

__all__ = ["RobotArmConfig", "RobotKinematics", "MotorLink", "ToolLink"]
