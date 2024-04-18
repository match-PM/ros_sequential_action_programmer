tool = [
    {
      "type": "retrieval"
    },
] 


    # {
    #   "type": "function",
    #   "function": {
    #       "name": "SaveSequence",
    #       "description": "This function saves a json-string. Remove all \ and \n and input the complete string!",
    #       "parameters": {
    #           "type": "object",
    #           "properties": {
    #               "json": {
    #                   "type": "string",
    #                   "description": "json-string",
    #               },
    #           },
    #           "required": ["json"],
    #       },
    #   },
    # },
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