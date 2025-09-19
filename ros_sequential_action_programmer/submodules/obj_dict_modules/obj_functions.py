from typing import Union
import re

def get_obj_value_from_key(obj: Union[dict, list, any], path_key: str) -> any:
    """
    This function iterates through an object (any object, list, dict) and returns the value given in the path.
    E.g. path_key = 'Foo.Fuu.Faa'
    """
    if path_key is None or obj is None:
        return None

    keys = path_key.split(".")
    current_value = obj

    try:
        for key in keys:
            if isinstance(current_value, dict) and key in current_value:
                current_value = current_value[key]
            elif isinstance(current_value, list):
                try:
                    key = int(key)
                    current_value = current_value[key]
                except (ValueError, IndexError):
                    return None
            elif hasattr(current_value, key):
                current_value = getattr(current_value, key)
            else:
                return None
    except (KeyError, AttributeError):
        return None

    return current_value


def set_obj_value_from_key(obj: any, path_key: str, new_value: any) -> bool:
    """
    This function sets the new_value at the value to which the path_key leads to. The obj can be any object, dict, list
    Returns False if value cant be set
    """
    keys = path_key.split(".")
    current_value = obj

    try:
        for key in keys[:-1]:
            if isinstance(current_value, dict) and key in current_value:
                current_value = current_value[key]
            elif isinstance(current_value, list):
                key = int(key)
                current_value = current_value[key]
            elif hasattr(current_value, key):
                current_value = getattr(current_value, key)
            else:
                return False

        last_key = keys[-1]
        if isinstance(current_value, dict) and last_key in current_value:
            current_value[last_key] = new_value
        elif isinstance(current_value, list):
            last_key = int(last_key)
            current_value[last_key] = new_value
        elif hasattr(current_value, last_key):
            setattr(current_value, last_key, new_value)
        else:
            return False
        # Finally
        return True
    except (KeyError, IndexError, AttributeError, ValueError):
        return False
    
def get_last_index_value(self, input_string:str)->int:
    # Use regular expression to find all occurrences of '[x]' and extract 'x' from the last one
    matches = re.findall(r'\[(\d+)\]', input_string)
    
    if matches:
        return int(matches[-1])
    else:
        return None
    