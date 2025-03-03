#!/usr/bin/env python3
import openai
import py_trees
import rospy
from std_msgs.msg import *  
from sensor_msgs.msg import *
from stonefish_ros import *
from std_msgs.msg import *
import re
import time
import os
from pcl_geometric_primitives_detector.srv import QueryFullGraph,GetTaxonomy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import json
import threading
import importlib
import roslib
# Initialize OpenAI API (make sure you have your API key set up)
openai.api_key = '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'

def get_custom_msg_class(msg_type_str):
    try:
        pkg, msg = msg_type_str.split("/")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg, None)
    except Exception as e:
        print(f"Error loading message class {msg_type_str}: {e}")
        return None

#Variable to store system context
SYSTEM_MESSAGE = """You are the intelligent brain of an underwater robot. Your role is to process mission requests, validate them against provided knowledge, and generate appropriate behavior trees in XML format. Just answer to what was give to you only taking into account the data sent to you. But don't add to the actual answer, answer to previous queries

#### Input Data:
You will receive the following regularly:
- **Knowledge Graph (JSON)**: Contains information about the robot’s capabilities and environment.
- **Taxonomy (JSON)**: Defines classifications and relationships relevant to the robot's operations.
- **Transformations (TFs) (JSON)**: Provides positional and orientation data of objects. Remember them, and update them if receive them
- If you receive them return that you have received them and specify what you received

#### Handling Input Requests:
1. **Mission Validation:**
   - If the input starts with **Mission:**, validate it using the knowledge graph and taxonomy.
   - If the mission is possible, return:
     ```
     "mission validated"
     Reason: <why the mission is possible>
     ```
   - If the mission is not possible, return:
     ```
     "cannot perform the mission"
     Reason: <why the mission is not possible>
     ```
   - Remember the validation result but don't talk about that until asked
   
   - Give a behavior tree for the full mission in separetes trees, using the information below

   
   -If you don't have received yet the knowledge graph and taxonomy, tell that you don't know yet

2. **Location Queries:**
   - If the input asks for the location of an object:
     - Return its **position and orientation** using the TF data, only that, no other text
     - If no TF is available, return:
       ```
       "Unknown location: No transformation data available."
       ```

3. **Battery Check & Docking:**
   - If the input mentions **"battery", "low battery", or "charge"**, return:
     ```
     "yes it should dock"
     ```
     **OR**
     ```
     "no it can continue"
     ```
   - Additionally, provide an XML behavior tree for docking at a specified position:
     ```xml
     <root BTCPP_format="4">
         <BehaviorTree ID="BehaviorTreeNode">
             <PrioritySelector>
                 <ConditionNode name="CheckBattery" condition="battery_low"/>
                 <Sequence>
                     <ActionNode name="MoveToDock" action="move_to_position" x="0" y="0" z="1"/>
                     <ActionNode name="Dock" action="dock"/>
                 </Sequence>
             </PrioritySelector>
         </BehaviorTree>
     </root>
     ```

4. **Survey, Area Scanning Missions:**
   - If the mission mentions **"survey", "area", or "scan"**, return:
     ```
     "yes, can do the mission"
     Reason: <why the mission is possible>
     ```
   - Provide an XML behavior tree for a **lawnmower trajectory**, ensuring correct indentation. Example:
     ```xml
     <root BTCPP_format="4">
         <BehaviorTree ID="main_bt">
             <Sequence name="iauv_girona1000_survey_scan">
                 <LogSequence name="start" message="mission start"/>
                 <PathRequest type="scan" start="-2.5;-2.0;4.0" goal="0.0;3.0;5.5" width="10.0" length="5.0" survey_type="{survey_type}"/>
                 <FollowPath path_follow_is_completed="false" survey_type="{survey_type}"/>
             </Sequence>
         </BehaviorTree>
     </root>
     ```

5. **Movement Requests (Non-Docking):**
   - If the input asks to move to a location (but **not docking-related**), return an XML behavior tree for a **go-to action**:
     ```xml
     <root BTCPP_format="4">
         <BehaviorTree ID="main_bt">
             <Sequence name="iauv_girona1000_survey_scan">
                 <LogSequence name="start" message="[GOTO] mission start"/>
                 <PathRequest name="main_path" type="simple" start="{start}" w_start="0.707" z_start="-0.707" goal="{goal}" w_end="0.707" z_end="-0.707" is_to_object="false" survey_type="{survey_type}"/>
                 <FollowPath path_follow_is_completed="false" survey_type="{survey_type}"/>
             </Sequence>
         </BehaviorTree>
     </root>
     ```

6. **Touch Requests:**
   - If the input mentions **"touch"**, return:
     ```
     "touch <object>"
     ```
   - Provide an XML behavior tree for the touch action:
     ```xml
     <root BTCPP_format="4">
         <BehaviorTree ID="main_bt">
             <Sequence name="iauv_girona1000_survey_scan">
                 <LogSequence name="start" message="[TOUCH] mission start"/>
                 <LogSequence name="end" message="[TOUCH] mission end"/>
             </Sequence>
         </BehaviorTree>
     </root>
     ```

7. **Circular Inspection Missions:**
   - If the mission mentions **"circular", "surround", "object on our path", "look at", or "inspect"**, return:
     ```
     "circular"
     ```
   - Provide an XML behavior tree for circular inspection:
     ```xml
     <root BTCPP_format="4">
         <BehaviorTree ID="main_bt">
             <Sequence name="iauv_girona1000_survey">
                 <PathRequest type="circular" survey_type="{survey_type}"/>
                 <ReactiveSequence>
                     <isPathClear/>
                     <Inspect inspection_is_completed="true" survey_type="{survey_type}"/>
                 </ReactiveSequence>
             </Sequence>
         </BehaviorTree>
     </root>
     ```

#### Formatting Guidelines:
- Always ensure correct XML indentation.
- Keep behavior tree structures valid and readable.
- Use placeholders `{survey_type}`, `{start}`, `{goal}` where necessary.
"""


# A list to keep track of the conversation history
conversation_history = [{"role": "system", "content": SYSTEM_MESSAGE}]

def llm_decision(input_text):
    try:
        #print(input_text)
        # Append the user's message to the conversation history
        conversation_history.append({"role": "user", "content": input_text})

        # Send the conversation history to the LLM
        response = openai.ChatCompletion.create(
            model="gpt-4o",  # You can adjust to GPT-4 or other models
            messages=conversation_history,
            max_tokens=1000
        )

        # Extract the response content
        decision = response['choices'][0]['message']['content'].strip()

        # Clean up the decision (removes backticks, 'xml' keyword)
        decision = re.sub(r'(?i)[`]|\bxml\b', '', decision)
        
        print("DECISION:",decision)

        # Check if the response contains XML
        if "<xml" in decision or "<root" in decision:
            # Extract an action keyword for the filename
            action_match = re.search(r"(dock|go(?: to)?|inspect|exploration|scan|touch|circular)", decision, re.IGNORECASE | re.DOTALL)
            action = action_match.group(1) if action_match else "action"
            if (("go" in action) or ("GO" in action) or ("GOTO" in action) or ("go to" in action)) :
                action = "go_to"
            if action == "circular":
                #print(action)
                action = "circular_survey"
            
            # Save the XML content to a file
            # Carlo cambia il path del file se serve
            path=roslib.packages.get_pkg_dir("bt_cpp") + "/llm_bt_xml_a/";
            filename = path+f"{action}.xml"
            with open(filename, "w", encoding="utf-8") as file:
                file.write(decision)
            
            print(f"XML response saved as {filename}")
            return filename  # Return the filename for the XML

        # If no XML, return the action decision
        #print("Filtered Decision: " + decision)
        return decision  # Return the decision if it's not XML

    except Exception as e:
        print(f"Error calling LLM: {e}")
        return "Failure"


def convert_to_json(subject, target, action):
    """
    Converts subject, target, and action lists into JSON format.
    """
    # Ensure the lengths of subject, target, and action are the same
    if len(subject) == len(target) == len(action):
        # Prepare the list of relations as dictionaries
        relations = [{"subject": s, "target": t, "action": a} 
                     for s, t, a in zip(subject, target, action)]
        
        # Create the final structure
        graph_data = {
            "relations": relations
        }

        # Convert the structure to a JSON string
        return json.dumps(graph_data, indent=4)

class ROS1TopicExplorer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ros1_topic_explorer')
        
        # Subscribers for dynamic and static transforms
        self.tf_data = {}
        self.subscription_tf = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=100)
        self.subscription_tf_static = rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(30.0), self.timer_callback) 

        rospy.loginfo("TF Listener initialized.")
        
        
        # Wait for service availability
        rospy.wait_for_service('/query_full_graph')
        rospy.wait_for_service('/get_taxonomy')
        self.cli = rospy.ServiceProxy('/query_full_graph', QueryFullGraph)
        self.cli_taxonomy = rospy.ServiceProxy('/get_taxonomy', GetTaxonomy)

    def timer_callback(self, event):
        """Called every 2 seconds to print all transforms"""
        all_transforms = self.get_all_transforms()
        rospy.loginfo(f"Current stored transforms: {all_transforms}")
    
    def wait_for_service(self, service, service_name):
        """Helper function to wait for service availability."""
        rospy.loginfo(f"Waiting for service {service_name}...")
        rospy.wait_for_service(service_name)
        rospy.loginfo(f"Service {service_name} available.")

    def tf_callback(self, msg):
        """Callback for /tf (dynamic transforms)"""
        for transform in msg.transforms:
            self.store_transform(transform, static=False)

    def tf_static_callback(self, msg):
        """Callback for /tf_static (static transforms)"""
        for transform in msg.transforms:
            self.store_transform(transform, static=True)

    def store_transform(self, transform, static=False):
        """Stores TF in a dictionary with its position and orientation"""
        frame_name = transform.child_frame_id
        if(("camera" not in frame_name) and("thruster" not in frame_name) and ("pressure" not in frame_name) and ("bravo" not in frame_name) ):
            self.tf_data[frame_name] = {
               "parent": transform.header.frame_id,
               "translation": {
                   "x": transform.transform.translation.x,
                   "y": transform.transform.translation.y,
                   "z": transform.transform.translation.z,
               },
               "rotation": {
                   "x": transform.transform.rotation.x,
                   "y": transform.transform.rotation.y,
                   "z": transform.transform.rotation.z,
                   "w": transform.transform.rotation.w,
               },
               "static": static  # True if from /tf_static
           }



    def get_all_transforms(self):
        """Returns all stored transforms as a dictionary"""
        info = "These are the transformations \n" + json.dumps(self.tf_data)
        conversation_history.append({"role": "system", "content": info})
        llm_decision(info)

        
    def list_topics(self, max_retries=5, delay=0.5):
        """Ensure all topics are discovered by retrying."""
        all_topics = set()
        
        # Try multiple times to discover topics
        for _ in range(max_retries):
            # Get all currently published topics
            topics = rospy.get_published_topics()
            
            # Format the topics into a set of tuples (name, types)
            formatted_topics = {(name, tuple(types)) for name, types in topics} 
            all_topics.update(formatted_topics)
            
            if len(topics) > 0:  # Ensure at least some topics are found
                break
            
            time.sleep(delay)  # Wait before retrying
            
        return list(all_topics)

        
    def find_battery_topic(self):
        """Finds and subscribes to the battery topic."""
        topics = self.list_topics()
        for topic, types in topics:
            if "robotB/battery_level" in topic.lower():
                self.battery_topic = topic
                self.subscribe_to_topic(topic, types[0])  # Subscribe to the topic with the correct message type
                rospy.loginfo(f"Monitoring battery topic: {topic}")
                
                # Start periodic checking using rospy.Timer
                rospy.Timer(rospy.Duration(1.0), self.check_battery_with_llm, oneshot=False)
                break  # Assume only one battery topic exists

    def subscribe_to_topic(self, topic, msg_type):
        """Subscribe to the topic."""
        if msg_type == 'std_msgs/Float32':  # Assuming battery level is a Float32 message
            rospy.Subscriber(topic, Float32, self.battery_callback)
        else:
            rospy.logwarn(f"Unknown message type {msg_type} for topic {topic}")

    def battery_callback(self, msg):
        """Callback for the battery topic."""
        self.battery_level = msg.data

    def check_battery_with_llm(self, event=None):
        """Checks the battery level and decides whether to dock."""
        rospy.loginfo(f"BATTERY LEVEL: {self.battery_level}")
        if self.battery_level is not None:
            input_text = f"The robot's battery is at {self.battery_level}%. Should it dock?"
            rospy.loginfo(input_text)
            
            # Assuming llm_decision is a function that gets a decision from a language model
            decision = llm_decision(input_text)  # Implement llm_decision elsewhere
            rospy.loginfo(f"LLM Decision: {decision}")

            if "<root" in decision:  # LLM returned XML (docking action)
                with open("dock.xml", "w", encoding="utf-8") as file:
                    file.write(decision)
                rospy.loginfo("Docking behavior tree saved as dock.xml")
            else:
                rospy.loginfo(f"LLM Decision: {decision}")

    def create_behavior_tree(self):
        root = py_trees.composites.Sequence(name="Root Sequence", memory=False)
        root.add_children([
            CheckEnvironment("Check Environment"),
            TakeAction("Take Action"),
            GoAction("Go Action"),
            TouchAction("Touch Action"),
            HandleFailure("Handle Failure"),
            CircularSurveyAction("Circular Survey")
        ])
        return root

    # Main function to run the behavior tree
    def run_behavior_tree(self):
        self.root = self.create_behavior_tree()
        behavior_tree = py_trees.trees.BehaviourTree(self.root)

        # Run the tree for a few cycles
        for i in range(6):
            behavior_tree.tick()


    def get_full_graph(self):
        """
        This function calls the 'get_full_graph' service and handles the response synchronously.
        """
        #print("Calling service")
        try:
            result = self.cli()  # Synchronous service call in ROS1
            #print("Service call completed.")
            
            if result is not None:
                #print("Graph data received")
                #print(result.subject, result.target, result.action)
                graph_data_json = convert_to_json(result.subject, result.target, result.action)
                # Print or save the JSON as needed
                #print("Graph data in JSON format:")
                return graph_data_json  # Assuming graph_data is in YAML or JSON format
            else:
                rospy.logerr('Failed to retrieve affordance graph.')
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def get_taxonomy(self):
        """
        Calls the 'get_taxonomy' service to retrieve the taxonomy data.
        Converts the response into a structured JSON format.
        """
        try:
            result = self.cli_taxonomy()  # Synchronous service call in ROS1

            if result is not None:
                # Convert response into structured JSON
                taxonomy_data = []
                for i, obj_name in enumerate(result.object_names):
                    start_idx = result.action_offsets[i]
                    end_idx = result.action_offsets[i + 1] if i + 1 < len(result.action_offsets) else len(result.actions)
                    actions = result.actions[start_idx:end_idx]
                    taxonomy_data.append({"name": obj_name, "actions": actions})

                taxonomy_json = json.dumps({"objects": taxonomy_data}, indent=2)

                # Print or return JSON as needed
                #print("Taxonomy data in JSON format:")
                return taxonomy_json
            else:
                rospy.logerr('Failed to retrieve taxonomy data.')
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

# Define the behavior tree tasks

class CheckEnvironment(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckEnvironment, self).__init__(name)
        self.checked = False  # To keep track if environment has already been checked

    def update(self):
        if not self.checked:  # Only ask once
            input_text = "Do the survey of the area"
            decision = llm_decision(input_text)
            
            if "safe" in decision.lower():
                self.checked = True
                #print("Environment check passed.")
                return py_trees.common.Status.SUCCESS
            else:
                #print("Environment check failed.")
                self.checked = True
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS  # Skip check if already done


class TakeAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(TakeAction, self).__init__(name)
        self.checked = False

    def update(self):
        if(not self.checked):
            input_text = "What should the robot do if its battery is low? Docking pos x=0 y=0 z=1"
            decision = llm_decision(input_text)
            if "action" in decision.lower():
               #print(f"Action taken: {decision}")
               self.checked = True
               return py_trees.common.Status.SUCCESS
            else:
               self.checked = True
               return py_trees.common.Status.FAILURE
        else:
           return py_trees.common.Status.SUCCESS
           
class TouchAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(TouchAction, self).__init__(name)
        self.checked = False

    def update(self):
        if(not self.checked):
            input_text = "We need to touch a cube, cube pos x=0 y=0 z=1"
            decision = llm_decision(input_text)
            if "touch" in decision.lower():
               #print(f"Action taken: {decision}")
               self.checked = True
               return py_trees.common.Status.SUCCESS
            else:
               self.checked = True
               return py_trees.common.Status.FAILURE
        else:
           return py_trees.common.Status.SUCCESS
           
class CircularSurveyAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CircularSurveyAction, self).__init__(name)
        self.checked = False

    def update(self):
        if(not self.checked):
            input_text = "There is an object on our path, we need to understand what it is"
            decision = llm_decision(input_text)
            if "circular_survey" in decision.lower():
               #print(f"Action taken: {decision}")
               self.checked = True
               return py_trees.common.Status.SUCCESS
            else:
               self.checked = True
               return py_trees.common.Status.FAILURE
        else:
           return py_trees.common.Status.SUCCESS
           
class GoAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GoAction, self).__init__(name)
        self.checked = False

    def update(self):
        if(not self.checked):
            input_text = "We need to go at x=0 y=0 z=1"
            decision = llm_decision(input_text)
            if "action" in decision.lower():
               #print(f"Action taken: {decision}")
               self.checked = True
               return py_trees.common.Status.SUCCESS
            else:
               self.checked = True
               return py_trees.common.Status.FAILURE
        else:
           return py_trees.common.Status.SUCCESS
           
class HandleFailure(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(HandleFailure, self).__init__(name)
        self.checked = False

    def update(self):
        if(not self.checked):
           self.checked = True
           print("Handling failure... Switching strategy.")
           return py_trees.common.Status.SUCCESS
        else:
           return py_trees.common.Status.SUCCESS


def update_graph_periodically(ros_topic_exp, conversation_history):
    while not rospy.is_shutdown():
        graph = ros_topic_exp.get_full_graph()
        taxonomy = ros_topic_exp.get_taxonomy()
        #print("GRAFO RICEVUTO", graph) 
        info = (
            f"This is the actual knowledge graph of the robot: {graph}\n"
            f"Here is the taxonomy: {taxonomy}\n"
        )
        conversation_history.append({"role": "system", "content": info})
        llm_decision(info)
        rospy.sleep(20)  # Update every 10 seconds, adjust as needed

def read_mission_file(file_path):
    """Reads the mission file as a string."""
    with open(file_path, "r") as file:
        mission_text = file.read().strip()
    return mission_text



# Main function
def main():
    #rospy.init_node('ros1_topic_explorer', anonymous=True) 
    ros_topic_exp = ROS1TopicExplorer()
    conversation_history = []

    # Start the graph update in a separate thread
    graph_update_thread = threading.Thread(target=update_graph_periodically, args=(ros_topic_exp, conversation_history), daemon=True)
    graph_update_thread.start()

    # Fetch and print graph and taxonomy data
    graph = ros_topic_exp.get_full_graph()
    taxonomy = ros_topic_exp.get_taxonomy()
    info = (
        f"This is the actual knowledge graph of the robot: {graph}\n"
        f"Here is the taxonomy: {taxonomy}\n"
    )
    conversation_history.append({"role": "system", "content": info})
    llm_decision(info)
    if((len(graph)>0) and (len(taxonomy) > 0)):
          mission_file_path = roslib.packages.get_pkg_dir('pcl_geometric_primitives_detector')+'/ace/mission_Beta.txt'  # Path to the text file
          mission_text = read_mission_file(mission_file_path)
          print("Mission Content:\n", mission_text)
          decision = llm_decision(mission_text)


    while not rospy.is_shutdown():
        # Fetch TF data and append to conversation history
        rospy.sleep(1)  

    rospy.spin() 


if __name__ == "__main__":
    main()

