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


COMPATIBLE_TYPES = {
    # --- Integer family ---
    "int8":   ["int8", "uint8"],
    "uint8":  ["uint8"],
    "int16":  ["int8", "int16", "uint8","uint16"],
    "uint16": ["uint8", "uint16"],
    "int32":  ["int8", "int16", "int32", "uint8", "uint16", "uint32"],
    "uint32": ["uint8", "uint16", "uint32"],
    "int64":  ["int8", "int16", "int32", "int64","uint8", "uint16", "uint32", "uint64"],
    "uint64": ["uint8", "uint16", "uint32", "uint64"],

    # --- Floating-point family ---
    "float32": ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32","double", "float"],
    "float64": ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64","double", "float"],
    "float":   ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float", "float64", "double"],
    "double":  ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "float", "double"],

    # --- String family ---
    "string": ["string", "str"],
    "str":    ["string", "str"],

    # --- Boolean family ---
    "bool": ["bool", "boolean"],
    "boolean": ["bool", "boolean"],
}
