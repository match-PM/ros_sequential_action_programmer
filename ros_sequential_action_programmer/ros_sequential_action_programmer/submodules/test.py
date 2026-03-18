from collections import OrderedDict
from typing import Union
import json

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



test = OrderedDict([('ref_plane', OrderedDict([('ref_plane_name', ''), ('point_names', ['', '', ''])]))])
#test = convert_ordered_dict_to_ordered_dict(test)
print(test)
test_key = 'ref_plane.point_names'

test2 = get_obj_value_from_key((test), test_key)
test2[0] = 'test'
print(test2)