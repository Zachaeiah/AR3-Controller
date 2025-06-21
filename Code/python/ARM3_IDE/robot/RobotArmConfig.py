import json
import os
from typing import List, Optional, Union
from pathlib import Path
from robot.MotorLink import MotorLink
from robot.ToolLink import ToolLink
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
        self.links: List[Union[MotorLink, ToolLink]] = []

    def add_link(self, link: Union[MotorLink, ToolLink]) -> None:
        """
        Add a MotorLink or ToolLink object to the configuration.

        Args:
            link (MotorLink | ToolLink): The link instance to add.

        Raises:
            TypeError: If the argument is not a MotorLink or ToolLink instance.
        """
        if not isinstance(link, (MotorLink, ToolLink)):
            raise TypeError("Expected a MotorLink or ToolLink instance")
        self.links.append(link)

    def to_json(self, filename: Union[str, Path]) -> None:
        """
        Save the configuration to a JSON file.

        Args:
            filename (str or Path): File path where the configuration will be saved.
        """
        path: str = os.fspath(filename)
        data: dict[str, any] = {
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
        path: str = os.fspath(filename)
        with open(path, 'r') as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError("Expected a dictionary with 'metadata' and 'links'")

        metadata: dict = data.get("metadata")
        if not isinstance(metadata, dict) or metadata.get("type") != "robot_arm_config":
            raise ValueError("Missing or incorrect metadata for robot arm configuration")

        links_data: list = data.get("links")
        if not isinstance(links_data, list):
            raise ValueError("'links' should be a list of motor/tool link data")

        arm = cls()
        arm.metadata = metadata

        for link_data in links_data:
            if "DenaHar_params" in link_data:
                arm.add_link(MotorLink.from_dict(link_data))
            elif "position" in link_data and "orientation" in link_data:
                arm.add_link(ToolLink.from_dict(link_data))
            else:
                raise ValueError("Invalid link entry: missing DenaHar_params or tool pose")

        return arm

    def find_link(self, key: Union[str, int]) -> Optional[Union[MotorLink, ToolLink]]:
        """
        Find a MotorLink or ToolLink by name or ID.

        Args:
            key (str or int): The name or ID of the link.

        Returns:
            Optional[MotorLink | ToolLink]: The matching link, or None if not found.
        """
        for link in self.links:
            if link.name == key or str(link.id) == str(key):
                return link
        return None

    def update_link(
        self,
        key: Union[str, int],
        motor_params: Optional[dict] = None,
        params: Optional[dict] = None
    ) -> None:
        """
        Update a MotorLink or ToolLink's parameters by name or ID.

        Args:
            key (str or int): The name or ID of the link to update.
            motor_params (dict, optional): Motor parameter updates.
            params (dict, optional): For MotorLink, this is DH parameters.
                                             For ToolLink, it must contain "position" and/or "orientation".

        Raises:
            ValueError: If the link is not found.
            TypeError: If the updates are not in dictionary format.
        """
        link = self.find_link(key)
        if not link:
            raise ValueError(f"Link with name or ID '{key}' not found.")

        if motor_params is not None and not isinstance(motor_params, dict):
            raise TypeError("motor_params must be a dictionary")
        if params is not None and not isinstance(params, dict):
            raise TypeError("DenaHar_params must be a dictionary")

        if isinstance(link, MotorLink):
            link.update(motor_params, params)
        elif isinstance(link, ToolLink):
            pose = {}
            if "position" in params:
                pose["position"] = params["position"]
            if "orientation" in params:
                pose["orientation"] = params["orientation"]
            link.update(motor_params, pose)

    def __repr__(self) -> str:
        return f"RobotArmConfig(links={self.links!r})"