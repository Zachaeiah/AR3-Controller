from typing import Optional, Dict, Any, Union
import copy

class MotorLink:
    def __init__(
        self,
        name: str,
        id: Union[str, int],
        motor_params: Optional[Dict[str, Any]] = None,
        link_params: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize a MotorLink instance representing a robot arm joint.

        Args:
            name (str): The human-readable name of the motor/link (e.g. "shoulder_joint").
            id (str or int): A unique identifier for the motor/link.
            motor_params (dict, optional): Dictionary containing motor-related parameters
                such as steps per revolution, velocity limits, acceleration limits, etc.
                Defaults to empty dict if not provided.
            link_params (dict, optional): Dictionary containing link-related parameters,
                typically the Denavit-Hartenberg parameters like 'a', 'alpha', 'd'.
                Defaults to empty dict if not provided.

        Raises:
            TypeError: If `motor_params` or `link_params` are provided but are not dicts.
            TypeError: If `name` is not a string or `id` is not str or int.
        """
        if not isinstance(name, str):
            raise TypeError(f"name must be a string, got {type(name)}")
        if not isinstance(id, (str, int)):
            raise TypeError(f"id must be a string or int, got {type(id)}")

        if motor_params is not None and not isinstance(motor_params, dict):
            raise TypeError("motor_params must be a dict if provided")
        if link_params is not None and not isinstance(link_params, dict):
            raise TypeError("link_params must be a dict if provided")

        # Use deepcopy to avoid shared mutable default issues
        self.name: str = name
        self.id: Union[str, int] = id
        self.motor_params: Dict[str, Any] = copy.deepcopy(motor_params) if motor_params else {}
        self.link_params: Dict[str, Any] = copy.deepcopy(link_params) if link_params else {}

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the MotorLink instance into a dictionary representation
        suitable for serialization (e.g. JSON).

        Returns:
            dict: A dictionary containing the motor link's data.
        """
        return {
            "name": self.name,
            "id": self.id,
            "motor_params": copy.deepcopy(self.motor_params),
            "link_params": copy.deepcopy(self.link_params),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotorLink":
        """
        Create a MotorLink instance from a dictionary representation.

        Args:
            data (dict): Dictionary containing the motor link data, typically
                loaded from a JSON file or similar.

        Returns:
            MotorLink: A new MotorLink instance populated with the data.

        Raises:
            KeyError: If required keys ('name', 'id') are missing.
            TypeError: If the data provided is not a dict.
        """
        if not isinstance(data, dict):
            raise TypeError("Input data must be a dictionary")

        name = data["name"]  # KeyError if missing, which is reasonable
        id_ = data["id"]

        motor_params = data.get("motor_params", {})
        link_params = data.get("link_params", {})

        return cls(name=name, id=id_, motor_params=motor_params, link_params=link_params)

    def update(
        self,
        motor_params: Optional[Dict[str, Any]] = None,
        link_params: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Update the motor and/or link parameters of this MotorLink.
        Only parameters passed in the dictionaries will be updated; others remain unchanged.

        Args:
            motor_params (dict, optional): Motor parameters to update (e.g. {"max_velocity": 150}).
            link_params (dict, optional): Link parameters to update (e.g. {"d": 0.25}).

        Raises:
            TypeError: If provided parameters are not dictionaries.
        """
        if motor_params is not None:
            if not isinstance(motor_params, dict):
                raise TypeError("motor_params must be a dict")
            self.motor_params.update(motor_params)

        if link_params is not None:
            if not isinstance(link_params, dict):
                raise TypeError("link_params must be a dict")
            self.link_params.update(link_params)

    def __repr__(self) -> str:
        return (
            f"MotorLink(\n"
            f"  name={self.name!r},\n"
            f"  id={self.id!r},\n"
            f"  motor_params={{\n"
            + "".join(f"    {k}: {v},\n" for k, v in self.motor_params.items()) +
            f"  }},\n"
            f"  link_params={{\n"
            + "".join(f"    {k}: {v},\n" for k, v in self.link_params.items()) +
            f"  }}\n"
            f")"
        )