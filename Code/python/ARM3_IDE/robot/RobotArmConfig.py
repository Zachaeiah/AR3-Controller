import json
import os
from typing import List, Optional, Union
from pathlib import Path
from robot.MotorLink import MotorLink
from datetime import datetime


class RobotArmConfig:
    """
    Class to manage a robot arm configuration composed of multiple MotorLink objects.
    """
    metadata: dict = {
            "type": "robot_arm_config",
            "version": "1.0",
            "author": "Unknown",
            "description": "",
            "created": datetime.now().isoformat()
        }

    def __init__(self) -> None:
        """
        Initialize an empty robot arm configuration.
        """
        self.links: List[MotorLink] = []

    def add_link(self, motor_link: MotorLink) -> None:
        """
        Add a MotorLink object to the configuration.

        Args:
            motor_link (MotorLink): The MotorLink instance representing a motor/link.

        Raises:
            TypeError: If the argument is not a MotorLink instance.
        """
        if not isinstance(motor_link, MotorLink):
            raise TypeError("Expected a MotorLink instance")
        self.links.append(motor_link)

    def to_json(self, filename: Union[str, Path]) -> None:
        """
        Save the configuration to a JSON file.

        Args:
            filename (str or Path): File path where the configuration will be saved.
        """
        path = os.fspath(filename)
        data = {
            "metadata": self.metadata,
            "links": [link.to_dict() for link in self.links]
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=4)

    @classmethod
    def from_json(cls, filename: Union[str, Path]) -> "RobotArmConfig":
        """
        Load a robot arm configuration from a JSON file.

        Args:
            filename (str or Path): Path to the JSON file.

        Returns:
            RobotArmConfig: A new RobotArmConfig instance populated from the file.

        Raises:
            FileNotFoundError: If the file does not exist.
            json.JSONDecodeError: If the file content is not valid JSON.
        """
        path = os.fspath(filename)
        with open(path, 'r') as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError("Expected a dictionary with 'metadata' and 'links'")

        metadata = data.get("metadata")
        if not isinstance(metadata, dict) or metadata.get("type") != "robot_arm_config":
            raise ValueError("Missing or incorrect metadata for robot arm configuration")

        links_data = data.get("links")
        if not isinstance(links_data, list):
            raise ValueError("'links' should be a list of motor link data")

        arm = cls()
        arm.metadata = metadata  # Overwrite default metadata
        for link_data in links_data:
            arm.add_link(MotorLink.from_dict(link_data))

        return arm

    def find_link(self, key: Union[str, int]) -> Optional[MotorLink]:
        """
        Find a MotorLink by name or ID.

        Args:
            key (str or int): The name or ID of the MotorLink.

        Returns:
            Optional[MotorLink]: The matching MotorLink, or None if not found.
        """
        for link in self.links:
            if link.name == key or str(link.id) == str(key):
                return link
        return None

    def update_link(
        self,
        key: Union[str, int],
        motor_params: Optional[dict] = None,
        link_params: Optional[dict] = None
    ) -> None:
        """
        Update a MotorLink's parameters by name or ID.

        Args:
            key (str or int): The name or ID of the MotorLink to update.
            motor_params (dict, optional): Motor parameter updates.
            link_params (dict, optional): Link parameter updates.

        Raises:
            ValueError: If the MotorLink is not found.
            TypeError: If the updates are not in dictionary format.
        """
        link = self.find_link(key)
        if not link:
            raise ValueError(f"Link with name or ID '{key}' not found.")

        if motor_params is not None and not isinstance(motor_params, dict):
            raise TypeError("motor_params must be a dictionary")
        if link_params is not None and not isinstance(link_params, dict):
            raise TypeError("link_params must be a dictionary")

        link.update(motor_params, link_params)

    def __repr__(self) -> str:
        return f"RobotArmConfig(links={self.links!r})"