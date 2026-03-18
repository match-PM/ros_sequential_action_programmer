
from rosidl_runtime_py import message_to_ordereddict

from rosidl_parser.definition import (
    BasicType,
    NamespacedType,
    AbstractSequence,
    AbstractNestedType,
    BoundedString,
    UnboundedString
)
import importlib

def field_type_map_recursive( msg_cls):
    """
    Return a dict mapping field_name -> type or nested dict for ROS 2 message class.
    Recursively breaks down nested messages into their base types.
    """

    # create a dummy instance of the message
    instance = msg_cls()

    result = {}
    for name in instance.__slots__:
        field_val = getattr(instance, name)
        field_type = type(field_val)

        if hasattr(field_val, '__slots__'):
            # nested message: recurse
            result[name.lstrip('_')] = field_type_map_recursive(field_type)
        elif isinstance(field_val, list):
            # list of primitives or nested messages
            if len(field_val) > 0 and hasattr(field_val[0], '__slots__'):
                result[name.lstrip('_')] = [field_type_map_recursive(type(field_val[0]))]
            else:
                # primitive list
                result[name.lstrip('_')] = [type(field_val[0]).__name__] if field_val else []
        else:
            # primitive type
            result[name.lstrip('_')] = field_type.__name__

    return result
    
    
# def field_type_map_recursive_with_msg_type( msg_cls):
#     """
#     Return a dict mapping field_name -> type info for ROS 2 message class.
#     Nested messages are represented as:
#         { "type": "<package/msg/Type>", "fields": { ... } }
#     Primitives are returned as their ROS type strings.
#     """
#     # create a dummy instance
#     instance = msg_cls()

#     result = {}
#     for name in instance.__slots__:
#         field_val = getattr(instance, name)
#         field_name = name.lstrip('_')
#         field_type = type(field_val)

#         if hasattr(field_val, '__slots__'):
#             # nested message: include its type and recursively its fields
#             type_str = f"{field_val.__class__.__module__.split('.')[0]}/" \
#                     f"{field_val.__class__.__name__}"
#             result[field_name] = {
#                 "type": type_str,
#                 "fields": field_type_map_recursive_with_msg_type(field_type)
#             }
#         elif isinstance(field_val, list):
#             # list of primitives or nested messages
#             if len(field_val) > 0 and hasattr(field_val[0], '__slots__'):
#                 type_str = f"{field_val[0].__class__.__module__.split('.')[0]}/" \
#                         f"{field_val[0].__class__.__name__}"
#                 result[field_name] = {
#                     "type": type_str,
#                     "fields": field_type_map_recursive_with_msg_type(type(field_val[0])),
#                     "is_array": True
#                 }
#             else:
#                 # primitive list
#                 result[field_name] = {
#                     "type": type(field_val[0]).__name__ if field_val else "unknown",
#                     "is_array": True
#                 }
#         else:
#             # primitive type
#             result[field_name] = {"type": field_type.__name__}

#     return result

def resolve_ros_type(val_type):
    """Convert ROSIDL type object to a string usable in the GUI."""
    if isinstance(val_type, BasicType):
        return val_type.typename          # e.g., "float32", "int32"
    elif isinstance(val_type, (BoundedString, UnboundedString)):
        return "str"
    elif isinstance(val_type, NamespacedType):
        return f"{val_type.namespaced_name()[0]}/{val_type.namespaced_name()[1]}"
    elif hasattr(val_type, 'type'):
        return str(val_type.type)
    else:
        return str(val_type)

import importlib
from rosidl_parser.definition import BasicType, NamespacedType, UnboundedString, BoundedString

def field_type_map_recursive_with_msg_type(msg_cls):
    """
    Return a dict mapping field_name -> type info for a ROS 2 message class.

    Output format examples:
      'foo': {'type': 'int64'}
      'poses': {'type': 'geometry_msgs/msg/Pose', 'is_array': True,
                'fields': { ... Pose fields ... }}
      'stamp': {'type': 'builtin_interfaces/Time', 'fields': {...}}
    """
    def rosidl_type_to_info(t):
        """
        Convert a rosidl_parser.definition type object `t` into either:
         - a primitive type string (e.g. 'int64'), or
         - a dict { 'type': '<pkg/msg/Type>' [, 'fields': {...}] [, 'is_array': True, ...] }
        """

        # 1) Sequence / Array (both expose .value_type)
        if getattr(t, 'value_type', None) is not None:
            elem = t.value_type
            elem_info = rosidl_type_to_info(elem)

            info = {}
            if isinstance(elem_info, str):
                info['type'] = elem_info
            else:
                info.update(elem_info)

            info['is_array'] = True
            if getattr(t, 'size', None) is not None:
                info['array_size'] = t.size
            if getattr(t, 'maxlen', None) is not None:
                info['maxlen'] = t.maxlen
            return info

        # 2) Basic primitive type
        if getattr(t, 'typename', None) is not None:
            return t.typename

        # 2b) Handle ROS 2 string types explicitly
        if isinstance(t, (UnboundedString, BoundedString)):
            return 'string'

        # 3) Namespaced / nested message type
        names = None
        if getattr(t, 'namespaced_name', None) is not None:
            try:
                names = t.namespaced_name()
            except TypeError:
                names = t.namespaced_name

        if names:
            type_str = "/".join(names)  # e.g. 'geometry_msgs/msg/Pose'
            pkg = names[0]
            sub = names[1]
            type_name = names[-1]
            
            # Try to import the generated Python class to recurse into fields
            module_name = f"{pkg}.{sub}"
            try:
                module = importlib.import_module(module_name)
                cls = getattr(module, type_name)
                fields = field_type_map_recursive_with_msg_type(cls)
                return {'type': type_str, 'fields': fields}
            except Exception:
                # Cannot import/resolve — just return the type string
                return {'type': type_str}

        # fallback: string representation
        return {'type': str(t)}

    # Iterate over message slots and slot types
    result = {}
    for slot_name, slot_type in zip(msg_cls.__slots__, msg_cls.SLOT_TYPES):
        field_name = slot_name.lstrip('_')
        info = rosidl_type_to_info(slot_type)

        if isinstance(info, str):
            result[field_name] = {'type': info}
        else:
            result[field_name] = info

    return result

