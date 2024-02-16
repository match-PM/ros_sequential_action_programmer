import os
from rclpy.node import Node
from openai import OpenAI
import json
import yaml
import time
from ros_sequential_action_programmer.submodules.co_pilot.tools import tool
from ament_index_python.packages import get_package_share_directory
from ros_sequential_action_programmer.submodules.co_pilot.ServiceCalls import ServiceCalls
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer

class AssistantAPI:
    def __init__(self, service_node:Node): 
        
        self.service_node = service_node
        self.action_sequence_node = RosSequentialActionProgrammer(service_node)
        self.service_builder = ServiceCalls(service_node)
        
        self.path = get_package_share_directory("ros_sequential_action_programmer")
        
        # Init client
        self.client = OpenAI()
        self.client.api_key = os.environ["OPENAI_API_KEY"]
        

        config_path = self.path + '/OpenAI_config.yaml'        
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
            gpt_model = config_data['gpt_model'][0]
            instruction_prompt = config_data['instruction_prompt']

            self.service_node.get_logger().info(f"model {gpt_model}")
            self.service_node.get_logger().info(f"instr {instruction_prompt}")

        #to retrieve all defined assistants
        self.my_assistants = self.client.beta.assistants.list(
            order="desc",
            limit = "20"
        )
                
        if self.my_assistants.data != []:
            self.assistant_id = self.my_assistants.data[0].id
            self.service_node.get_logger().info(f"Using Assistant {self.assistant_id}")

            # get the current assistant and add the current file
            self.my_assistant = self.client.beta.assistants.retrieve(self.assistant_id)
            # self.client.beta.assistants.delete(self.file_id)
            self.assistant = self.client.beta.assistants.update(
                assistant_id=self.assistant_id
            )

        else:
            self.service_node.get_logger().warn(f"No Assistant defined! A new one is created!")

            # Upload file
            self.update_and_upload_file()

            self.file_id = self.get_file_id()

            #call this once to create the assistant
            self.assistant = self.client.beta.assistants.create(
                name="PM Co Pilot",
                instructions=instruction_prompt,
                model = gpt_model,
                tools = tool,
                file_ids=[self.file_id]
            )
            #to retrieve all defined assistants
            self.my_assistants = self.client.beta.assistants.list(
                order="desc",
                limit = "20"
            )
            self.assistant_id = self.my_assistants.data[0].id


        self.thread = self.client.beta.threads.create()

        self.service_node.get_logger().info("Co-Pilot AssistantAPI initialized!")


    def get_file_id(self):
        #Call this to get the list of all uploaded files
        file_list = self.client.files.list()

        #Grab the ID of the uploaded earlier
        file_id = file_list.data[0].id
        return file_id
    
    
    def update_and_upload_file(self):

        file_name = "services_and_frames.json"
        file_path = f"{self.path}/{file_name}"

        # # Get current tf_static frames
        # tf_data = []
        # try:
        #     tf_data = self.action_sequence_node.recommendations.get_recommendations()

        #     tf_data = tf_data[1]['TF_static']
        # except Exception:
        #     self.service_node.get_logger().warn("tf_static topic not active!")


        # # Get all services that are included in the whitelist.yaml
        # data = self.action_sequence_node.get_all_service_req_res_dict(self.action_sequence_node.get_active_client_whtlist())
        
        # if data == {}:
        #     self.service_node.get_logger().warn("No active services!")

        # data['Frames'] = tf_data

        # # Save json-file
        # try:
        #     with open(file_path, 'w') as file:
        #         json.dump(data, file)

        #     self.service_node.get_logger().info(f"List saved to {file_path}")

        # except IOError as e:
        #     self.service_node.get_logger().error(f"An error occurred while writing the file: {e}")

        # except Exception as e:
        #     self.service_node.get_logger().error(f"An unexpected error occurred: {e}")

        # upload file to openAI API
        file = self.client.files.create(
            file=open(file_path, "rb"),
            purpose='assistants'
        )

    
    def add_message(self,message:str):
        #add a user massage to the thread
        assistant_message = self.client.beta.threads.messages.create(
            thread_id = self.thread.id,
            role = 'user',
            content=message
        )
        print(assistant_message)
        
    def run_assistant(self):
        #run the assistant to get back a response
        self.run = self.client.beta.threads.runs.create(
            thread_id = self.thread.id,
            assistant_id = self.assistant.id,
        )
        print(self.run.id)
    
    def retrieve_status(self):
        #retrieve the run status
        try:
            self.run = self.client.beta.threads.runs.retrieve(
                thread_id = self.thread.id,
                run_id = self.run.id
            )
            return self.run.status
        except:
            return 'failed'
    
    def execute_function(self):
        #figure out which function should be called
        
        tools_to_call = self.run.required_action.submit_tool_outputs.tool_calls

        self.service_node.get_logger().info(tools_to_call[0].function.name)
        self.service_node.get_logger().info(tools_to_call[0].function.arguments)

        #call the function
        self.tools_output_array = []
        for each_tool in tools_to_call:
            tool_call_id = each_tool.id
            function_name = each_tool.function.name
            function_arg = each_tool.function.arguments
            print("Tool ID:" + tool_call_id)
            print("Function to Call:" + function_name )
            print("Parameters to use:" + function_arg)

            function_arg = json.loads(function_arg)

            # function_call_args = self.extract_json_from_string(function_arg)

            # if 'error' in function_call_args:
            #     print(f"Error parsing JSON for tool {tool_call_id}: {function_call_args['error']}")
            #     output = "Function parameters not formatted as a json!"
            if (function_name == 'ServiceCall'):
                success, response=self.service_builder.execute_service_call(srv_values=function_arg)

            if success:
                output = json.dumps(response)
                self.service_node.get_logger().info(f"output {output}")
            else:
                output = False

            self.tools_output_array.append({"tool_call_id": tool_call_id, "output": output})

        print(self.tools_output_array)


    def extract_json_from_string(self, s):
        try:
            start = s.index('{')
            end = start
            brace_count = 0
            for i in range(start, len(s)):
                if s[i] == '{':
                    brace_count += 1
                elif s[i] == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        end = i
                        break
            json_str = s[start:end+1]
            return json.loads(json_str)
        except ValueError as e:
            # Handle case where '{' or '}' is not found or JSON is malformed
            return {"error": f"ValueError: {str(e)}"}
        except json.JSONDecodeError as e:
            # Handle case where the substring is not a valid JSON
            return {"error": f"JSONDecodeError: {str(e)}"}
        except Exception as e:
            # Handle any other unexpected errors
            return {"error": f"Unexpected error: {str(e)}"}
    

    def output_function_to_assistant(self):
        #give the the ouput from the function back to the assistant
        self.run = self.client.beta.threads.runs.submit_tool_outputs(
            thread_id = self.thread.id,
            run_id = self.run.id,
            tool_outputs=self.tools_output_array
        )

    def retrieve_response(self):
        #retrieve the response message from the assistant
        messages = self.client.beta.threads.messages.list(
            thread_id = self.thread.id,
        )
        # assistant_msg = ""
        # for each in messages:
        #     # print(each.role + ":" + each.content[0].text.value)
        #     if each.role == 'assistant':
        #         assistant_msg = each.content[0].text.value
        assistant_msg = ""
        latest_timestamp = 0

        for message in messages.data:
            if message.role == 'assistant' and message.created_at > latest_timestamp:
                # Update the latest timestamp and assistant message
                latest_timestamp = message.created_at
                assistant_msg = message.content[0].text.value

        print("Assistant: " + assistant_msg)
        return assistant_msg

    # deletes all files uploaded to the API
    def delete_files(self):
        #Call this to get the list of all uploaded files
        file_list = self.client.files.list()
        for file in file_list:
            self.client.files.delete(file.id)


    # Deletes the file attached to the assistant
    def delete_assistant_file(self):
        delete_file = self.client.beta.assistants.files.delete(
            assistant_id=self.assistant_id,
            file_id=self.file_id
        )
        return delete_file
    
    def delete_assistant(self) -> bool:
        return self.client.beta.assistants.delete(self.assistant.id).deleted

if __name__ == "__main__":
    pass


