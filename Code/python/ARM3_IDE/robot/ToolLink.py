from typing import Optional, Dict, Any, Union
import numpy as np
from scipy.spatial.transform import Rotation as R

class ToolLink:
    """
    Represents a robot tool (end effector) with motor config, position, and orientation as a quaternion.
    """

    def __init__(
        self,
        name: str,
        id: Union[str, int],
        motor_params: Optional[Dict[str, Any]],
        position: Union[list[float], tuple[float, float, float]],
        orientation: Union[list[float], tuple[float, float, float, float]]
    ) -> None:
        """
        Represents a tool link with a fixed transformation from the last joint.

        Args:
            name (str): Name of the tool link.
            id (str): ID of the tool link.
            motor_params (dict, optional): Dictionary containing motor-related parameters
                such as steps per revolution, velocity limits, acceleration limits, etc.
                Defaults to empty dict if not provided.
            position (list/tuple): [x, y, z] translation in mm.
            orientation (list/tuple): [r, i, j, j] quaternion rotation.
        """
        if len(position) != 3:
            raise ValueError("Position must have 3 elements (x, y, z).")
        if len(orientation) != 4:
            raise ValueError("Orientation must have 4 elements (r, i, j, k).")

        self.name = name
        self.id = id
        self.motor_params = motor_params
        self.position = list(position)
        self.orientation = list(orientation)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "id": self.id,
            "motor_params": self.motor_params,
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist()
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ToolLink":
        return cls(
            name=data["name"],
            id=data["id"],
            motor_params=data.get("motor_params", {}),
            position=data.get("position", [0.0, 0.0, 0.0]),
            orientation=data.get("orientation", [0.0, 0.0, 0.0, 1.0])
        )
    
    def to_matrix(self) -> np.ndarray:
        """
        Convert position and quaternion orientation to 4x4 transformation matrix.
        """
        matrix = np.identity(4)
        rotation = R.from_quat(self.orientation).as_matrix()
        matrix[:3, :3] = rotation
        matrix[:3, 3] = self.position
        return matrix

    def update(self, motor_params: dict = None, pose: dict = None) -> None:
        if motor_params:
            self.motor_params.update(motor_params)
        if pose:
            self.position = np.array(pose.get("position", self.position))
            self.orientation = np.array(pose.get("orientation", self.orientation))

    def __repr__(self) -> str:
        pos = ", ".join(f"{v:.3f}" for v in self.position)
        ori = ", ".join(f"{q:.3f}" for q in self.orientation)
        return f"ToolLink(name={self.name}, position=({pos}), orientation=({ori}))"
