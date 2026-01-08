import json
import os
from typing import Any
import datetime

class SeqParameterError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class SeqParameterManagerError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class SeqParameter:
    def __init__(self, name: str, val_type: str, value: Any):
        """
        Represents a single parameter, which can be a simple type (int, float, str)
        or a complex type represented as a dictionary (e.g., Pose).
        """
        self._name = name
        self._type = val_type
        self._value = self._validate_value(value)

    def _validate_value(self, value: Any) -> Any:
        """
        Ensure the value is serializable — allows primitives, lists, and dicts.
        """
        if isinstance(value, (int, float, str, bool, type(None))):
            return value
        elif isinstance(value, (list, tuple)):
            return [self._validate_value(v) for v in value]
        elif isinstance(value, dict):
            return {k: self._validate_value(v) for k, v in value.items()}
        else:
            raise SeqParameterError(
                f"Unsupported value type '{type(value).__name__}' for parameter '{self._name}'. "
                "Use dicts for complex objects like Pose."
            )

    def get_name(self) -> str:
        return self._name

    def get_value(self) -> Any:
        return self._value

    def get_type(self) -> str:
        return self._type

    def set_value(self, value: Any) -> None:
        self._value = self._validate_value(value)

    def get_as_dict(self) -> dict:
        """Return parameter data as a serializable dictionary."""
        return {
            "name": self._name,
            "type": self._type,
            "value": self._value,
        }


class SeqParameterManager:

    FILE_ENDING = ".rsapp.json"

    def __init__(self):
        self._seq_parameter_list:list[SeqParameter] = []
        self._file_name = None
        self._file_dir = None
        self._metadata = {}

    def add_parameter(self, parameter: SeqParameter) -> None:
        """Add a new parameter to the manager."""
        if not self.check_parameter_name_exists(parameter.get_name()):
            self._seq_parameter_list.append(parameter)
        else:
            raise SeqParameterError(f"Parameter with name '{parameter.get_name()}' already exists.")

    def check_parameter_name_exists(self, name: str) -> bool:
        return any(param.get_name() == name for param in self._seq_parameter_list)

    def get_parameter_by_name(self, name: str) -> SeqParameter:
        for param in self._seq_parameter_list:
            if param.get_name() == name:
                return param
        raise SeqParameterError(f"Parameter with name '{name}' not found.")

    def get_parameter_list(self) -> list:
        return self._seq_parameter_list

    def save_to_file(self) -> None:
        """Save all parameters to a JSON file with '.rsapp.json' extension."""
        if not self._file_dir:
            raise SeqParameterManagerError("File path is not set.")
        if not self._file_name:
            raise SeqParameterManagerError("File name is not set.")

        # Enforce the .rsapp.json extension
        file_name = self._file_name
        if not file_name.endswith(".rsapp.json"):
            file_name += ".rsapp.json"

        os.makedirs(self._file_dir, exist_ok=True)
        file_path = os.path.join(self._file_dir, file_name)

        # Update timestamp in metadata
        datatimestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._metadata["file_timestamp"] = datatimestamp

        # Build list: metadata first, then parameters
        parameter_data = []

        # then parameters
        parameter_data.extend(param.get_as_dict() for param in self._seq_parameter_list)

        full_dict = {"metadata": self._metadata, "parameters": parameter_data}

        with open(file_path, "w") as f:
            json.dump(full_dict, f, indent=4)

    def load_file(self, file_path: str) -> None:
        """
        Load parameters from a given .rsapp.json file.
        Automatically sets _file_dir and _file_name based on the input path.
        Supports optional top-level metadata dict(s) before parameter entries.
        """
        try:
            # Enforce extension
            if not file_path.endswith(".rsapp.json"):
                file_path += ".rsapp.json"

            if not os.path.exists(file_path):
                raise SeqParameterManagerError(f"File '{file_path}' does not exist.")

            # Read JSON
            with open(file_path, "r") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError as e:
                    raise SeqParameterManagerError(f"Failed to parse JSON: {e}")

            if not isinstance(data, dict):
                raise SeqParameterManagerError(f"Invalid file format: expected top-level list, got {type(data).__name__}")

            # Reset current state
            self.clear_parameters()

            parameter_data = data.get("parameters", [])
            # Iterate entries: first dicts that are metadata (do NOT contain name/type/value)
            # and then parameter dicts that do contain those keys.
            for entry in parameter_data:
                if not isinstance(entry, dict):
                    # ignore non-dict entries
                    continue

                # parameter entry check
                if all(k in entry for k in ("name", "type", "value")):
                    param = SeqParameter(entry["name"], entry["type"], entry["value"])
                    self.add_parameter(param)
                else:
                    # treat as metadata (merge)
                    # e.g. {"file_timestamp": "..."}
                    # merge keys into self._metadata (later metadata keys override earlier)
                    self._metadata.update(entry)

            # Update manager state (path & name)
            self._file_dir = os.path.dirname(file_path) or "."
            self._file_name = os.path.basename(file_path)
        except Exception as e:
            raise SeqParameterManagerError(f"Error loading file '{file_path}': {e}")

    def set_file_name(self, 
                      file_name: str) -> None:
        self._file_name = file_name

    def set_file_dir(self, file_path: str) -> None:
        self._file_dir = file_path

    def clear_parameters(self) -> None:
        """
        Clear all parameters and reset state.
        """
        self._seq_parameter_list = []
        self._file_name = None
        self._file_dir = None
        self._metadata = {}

    def get_file_name(self, strip_extension: bool = False) -> str:
        if strip_extension:
            # remove both .rsap.json or .json extensions safely
            file_name = self._file_name
            if file_name.endswith(self.FILE_ENDING):
                return file_name[:-len(self.FILE_ENDING)]
            elif file_name.endswith(".json"):
                return file_name[:-len(".json")]
            return file_name
        else:
            return self._file_name

    def get_file_dir(self) -> str:
        return self._file_dir

    def get_file_path(self) -> str:
        return os.path.join(self._file_dir, self._file_name) if self._file_dir and self._file_name else None

    def print_content(self) -> None:
        # print metadata first (if any)
        if self._metadata:
            print("Metadata:")
            for k, v in self._metadata.items():
                print(f"  {k}: {v}")
        for param in self._seq_parameter_list:
            print(f"Name: {param.get_name()}, Type: {param.get_type()}, Value: {param.get_value()}")

    def get_is_initialized(self) -> bool:
        return self._file_name is not None and self._file_dir is not None
    
    def get_number_of_parameters(self) -> int:
        return len(self._seq_parameter_list)

    def get_parameters_for_type(self, val_type: str) -> list[SeqParameter]:
        return [param for param in self._seq_parameter_list if param.get_type() == val_type]
    
    
if __name__ == "__main__":
    path = "/home/mll"
    file_name = "test"  # You can use "test" or "test.rsapp.json"

    manager = SeqParameterManager()
    manager.set_file_dir(path)
    manager.set_file_name(file_name)

    pose_dict = {
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }

    #param1 = SeqParameter("pose_param", "Pose", pose_dict)
    #param2 = SeqParameter("greeting", "string", "hello world")

    #manager.add_parameter(param1)
    #manager.add_parameter(param2)
    #manager.save_to_file()

    manager.load_file(os.path.join(path, file_name + ".rsapp.json"))
    manager.print_content()
    manager.save_to_file()