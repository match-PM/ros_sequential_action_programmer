# COMPATIBLE_TYPES = {
#     "int8": ["int8", "int16", "int32", "int64"],
#     "uint8": ["uint8", "uint16", "uint32", "uint64"],
#     "int16": ["int16", "int32", "int64"],
#     "uint16": ["uint16", "uint32", "uint64"],
#     "int32": ["int32", "int64"],
#     "uint32": ["uint32", "uint64"],
#     "int64": ["int64"],
#     "uint64": ["uint64"],
#     "float32": ["float32", "float64", "float", "double"],
#     "float64": ["float64", "double"],
#     "float": ["float", "double"],
#     "double": ["double"],
#     "string": ["string", "str"],
#     "str": ["string", "str"],
#     "bool": ["bool", "boolean"],
#     "boolean": ["bool", "boolean"]
# }


# COMPATIBLE_TYPES = {
#     # --- Integer family ---
#     "int8":   ["int8", "uint8"],
#     "uint8":  ["uint8"],
#     "int16":  ["int8", "int16", "uint8","uint16"],
#     "uint16": ["uint8", "uint16"],
#     "int32":  ["int8", "int16", "int32", "uint8", "uint16", "uint32"],
#     "uint32": ["uint8", "uint16", "uint32"],
#     "int64":  ["int8", "int16", "int32", "int64","uint8", "uint16", "uint32", "uint64"],
#     "uint64": ["uint8", "uint16", "uint32", "uint64"],

#     # --- Floating-point family ---
#     "float32": ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32","double", "float"],
#     "float64": ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64","double", "float"],
#     "float":   ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float", "float64", "double"],
#     "double":  ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "float", "double"],

#     # --- String family ---
#     "string": ["string", "str", "std_msgs/msg/String"],
#     "str":    ["string", "str", "std_msgs/msg/String"],
#     "std_msgs/msg/String": ["string", "str", "std_msgs/msg/String"],
    
#     # --- Boolean family ---
#     "bool": ["bool", "boolean", "std_msgs/msg/Bool"],
#     "boolean": ["bool", "boolean", "std_msgs/msg/Bool"],
#     "std_msgs/msg/Bool": ["bool", "boolean", "std_msgs/msg/Bool"],
# }


COMPATIBLE_TYPES = {
    # --- Integer family ---
    "int8":   ["int8", "uint8", "std_msgs/msg/Int8", "std_msgs/msg/UInt8"],
    "uint8":  ["uint8", "std_msgs/msg/UInt8"],
    "int16":  ["int8", "int16", "uint8", "uint16", "std_msgs/msg/Int8", "std_msgs/msg/Int16", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16"],
    "uint16": ["uint8", "uint16", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16"],
    "int32":  ["int8", "int16", "int32", "uint8", "uint16", "uint32", "std_msgs/msg/Int8", "std_msgs/msg/Int16", "std_msgs/msg/Int32", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32"],
    "uint32": ["uint8", "uint16", "uint32", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32"],
    "int64":  ["int8", "int16", "int32", "int64", "uint8", "uint16", "uint32", "uint64", "std_msgs/msg/Int8", "std_msgs/msg/Int16", "std_msgs/msg/Int32", "std_msgs/msg/Int64", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32", "std_msgs/msg/UInt64"],
    "uint64": ["uint8", "uint16", "uint32", "uint64", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32", "std_msgs/msg/UInt64"],


    "std_msgs/msg/Int8":   ["int8", "uint8", "std_msgs/msg/Int8", "std_msgs/msg/UInt8"],
    "std_msgs/msg/UInt8":  ["uint8", "std_msgs/msg/UInt8"],
    "std_msgs/msg/Int16":  ["int8", "int16", "uint8", "uint16", "std_msgs/msg/Int8", "std_msgs/msg/Int16", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16"],
    "std_msgs/msg/UInt16": ["uint8", "uint16", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16"],
    "std_msgs/msg/Int32":  ["int8", "int16", "int32", "uint8", "uint16", "uint32", "std_msgs/msg/Int8", "std_msgs/msg/Int16", "std_msgs/msg/Int32", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32"],
    "std_msgs/msg/UInt32": ["uint8", "uint16", "uint32", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32"],
    "std_msgs/msg/Int64":  ["int8", "int16", "int32", "int64", "uint8", "uint16", "uint32", "uint64", "std_msgs/msg/Int8", "std_msgs/msg/Int16", "std_msgs/msg/Int32", "std_msgs/msg/Int64", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32", "std_msgs/msg/UInt64"],
    "std_msgs/msg/UInt64": ["uint8", "uint16", "uint32", "uint64", "std_msgs/msg/UInt8", "std_msgs/msg/UInt16", "std_msgs/msg/UInt32", "std_msgs/msg/UInt64"],


    # --- Floating-point family ---
    "float32": ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "double", "float",
                "std_msgs/msg/Int8", "std_msgs/msg/UInt8", "std_msgs/msg/Int16", "std_msgs/msg/UInt16", "std_msgs/msg/Int32", "std_msgs/msg/UInt32", "std_msgs/msg/Int64", "std_msgs/msg/UInt64",
                "std_msgs/msg/Float32", "std_msgs/msg/Float64", "std_msgs/msg/Float"],
    "float64": ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "double", "float",
                "std_msgs/msg/Int8", "std_msgs/msg/UInt8", "std_msgs/msg/Int16", "std_msgs/msg/UInt16", "std_msgs/msg/Int32", "std_msgs/msg/UInt32", "std_msgs/msg/Int64", "std_msgs/msg/UInt64",
                "std_msgs/msg/Float32", "std_msgs/msg/Float64", "std_msgs/msg/Float"],
    "float":   ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float", "float64", "double",
                "std_msgs/msg/Int8", "std_msgs/msg/UInt8", "std_msgs/msg/Int16", "std_msgs/msg/UInt16", "std_msgs/msg/Int32", "std_msgs/msg/UInt32", "std_msgs/msg/Int64", "std_msgs/msg/UInt64",
                "std_msgs/msg/Float32", "std_msgs/msg/Float64", "std_msgs/msg/Float"],
    "double":  ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "float", "double",
                "std_msgs/msg/Int8", "std_msgs/msg/UInt8", "std_msgs/msg/Int16", "std_msgs/msg/UInt16", "std_msgs/msg/Int32", "std_msgs/msg/UInt32", "std_msgs/msg/Int64", "std_msgs/msg/UInt64",
                "std_msgs/msg/Float32", "std_msgs/msg/Float64", "std_msgs/msg/Float"],

    "std_msgs/msg/Float": ["float32", "float64", "float", "double",
                "std_msgs/msg/Int8", "std_msgs/msg/UInt8", "std_msgs/msg/Int16", "std_msgs/msg/UInt16", "std_msgs/msg/Int32", "std_msgs/msg/UInt32", "std_msgs/msg/Int64", "std_msgs/msg/UInt64",
                "std_msgs/msg/Float32", "std_msgs/msg/Float64", "std_msgs/msg/Float"],
    
    "std_msgs/msg/Float64": ["float32", "float64", "float", "double",
                "std_msgs/msg/Int8", "std_msgs/msg/UInt8", "std_msgs/msg/Int16", "std_msgs/msg/UInt16", "std_msgs/msg/Int32", "std_msgs/msg/UInt32", "std_msgs/msg/Int64", "std_msgs/msg/UInt64",
                "std_msgs/msg/Float32", "std_msgs/msg/Float64", "std_msgs/msg/Float"],


    # --- String family ---
    "string": ["string", "str", "std_msgs/msg/String"],
    "str":    ["string", "str", "std_msgs/msg/String"],
    "std_msgs/msg/String": ["string", "str", "std_msgs/msg/String"],

    # --- Boolean family ---
    "bool": ["bool", "boolean", "std_msgs/msg/Bool"],
    "boolean": ["bool", "boolean", "std_msgs/msg/Bool"],
    "std_msgs/msg/Bool": ["bool", "boolean", "std_msgs/msg/Bool"],
}