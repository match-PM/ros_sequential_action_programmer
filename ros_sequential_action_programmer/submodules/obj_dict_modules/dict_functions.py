
from collections import OrderedDict

def convert_to_ordered_dict(dictionary):
    if isinstance(dictionary, dict):
        return OrderedDict(
            (key, convert_to_ordered_dict(value)) for key, value in dictionary.items()
        )
    elif isinstance(dictionary, list):
        return [convert_to_ordered_dict(item) for item in dictionary]
    else:
        return dictionary
        

def get_key_value_pairs_from_dict_OLD(
        dictionary: OrderedDict,
    ) -> list[dict]:
        """
        Returns a list of all keys and values of a given dictionary.
        """
        def get_key_values(dic, k_v_list: list, parent_key=None):
            # iterate through the dict
            for key, value in dic.items():
                if parent_key is not None:
                    full_key = parent_key + "." + key
                else:
                    full_key = key

                if isinstance(value, OrderedDict):
                    get_key_values(value, k_v_list, full_key)
                if isinstance(value, list):
                    get_key_values(value, k_v_list, full_key)
                else:
                    k_v_list.append({full_key: value})

        key_value_list = []
        get_key_values(dictionary, key_value_list)
        return key_value_list

def get_key_value_pairs_from_dict(dictionary: [OrderedDict,dict]) -> list[dict]:
    """
    Returns a list of all keys and values of a given dictionary.
    """
    def get_key_values(dic, k_v_list: list, parent_key=None):
        # iterate through the dict
        for key, value in dic.items():
            if parent_key is not None:
                full_key = f"{parent_key}.{key}"
            else:
                full_key = key

            if isinstance(value, (OrderedDict, dict)):
                get_key_values(value, k_v_list, full_key)
            elif isinstance(value, list):
                for index, item in enumerate(value):
                    list_key = f"{full_key}[{index}]"
                    if isinstance(item, (OrderedDict, dict)):
                        get_key_values(item, k_v_list, list_key)
                    else:
                        k_v_list.append({list_key: item})
            else:
                k_v_list.append({full_key: value})

    key_value_list = []
    get_key_values(dictionary, key_value_list)
    return key_value_list

def get_dict_value_by_key(dictionary: [dict, OrderedDict], key:str)-> any:
    """
    Retrieves the value from a nested dictionary based on the key.

    Args:
    - dictionary: The nested dictionary (or OrderedDict) to search for the value.
    - key: The key in the format 'key2.subkey2[1].nested_key1'.

    Returns:
    - The value corresponding to the provided key, or None if the key is not found.
    """

    keys = key.split('.')
    current_obj = dictionary

    for key_part in keys:
        if '[' in key_part:
            key_name, index_str = key_part.split('[')
            index = int(index_str[:-1])  # Remove the trailing ']'
            try:
                current_obj = current_obj[key_name][index]
            except (KeyError, IndexError, TypeError):
                return None
        else:
            try:
                current_obj = current_obj[key_part]
            except (KeyError, TypeError):
                return None
    return current_obj

def flatten_dict_to_list(input_dict):
    result_list = []
    for key, values in input_dict.items():
        for value in values:
            result_list.append(f"{key}/{value}")
    return result_list

def is_string_in_dict(my_dict, target_string):
    for key, value in my_dict.items():
        if isinstance(value, list):
            # Check if the target string is in the list
            if target_string in value:
                return True
        elif isinstance(value, dict):
            # Recursively check in nested dictionaries
            if is_string_in_dict(value, target_string):
                return True
    return False