tool = [
    {
      "type": "retrieval"
    },
    {
      "type": "function",
      "function": {
          "name": "SaveSequence",
          "description": "This function saves the assembly sequence as a json-file.",
          "parameters": {
              "type": "object",
              "properties": {
                  "assembly_sequence": {
                      "type": "string",
                      "description": "Assembly sequence formatted as a json.",
                  },
              },
              "required": ["assembly_sequence"],
          },
      },
    },
] 
   # {
    #   "type": "function",
    #   "function": {
    #       "name": "ServiceCall",
    #       "description": "This function calls a service.",
    #       "parameters": {
    #           "type": "object",
    #           "properties": {
    #               "service_name": {
    #                   "type": "string",
    #                   "description": "Name of the service.",
    #               },
    #               "service_type": {
    #                   "type": "string",
    #                   "description": "Type of the service.",
    #               },
    #               "request": {
    #                   "type": "string",
    #                   "description": "Service request formatted as a json",
    #               },
    #               "response": {
    #                   "type": "string",
    #                   "description": "Service response formatted as a json",
    #               },
    #           },
    #           "required": ["service_name","service_type","request","response"],
    #       },
    #   },
    # },