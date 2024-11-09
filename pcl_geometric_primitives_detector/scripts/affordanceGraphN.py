#!/usr/bin/env python


import rospy
from std_srvs.srv import Empty
import tkinter as tk
from tkinter import simpledialog
import graphviz
import yaml
from pcl_geometric_primitives_detector.srv import AddObject, AddAffordance, ExportGraph, CreateGUI, DisplayGraph, DisplayGraphResponse, CheckRelation, GetActions, GetActionsResponse
from std_srvs.srv import Trigger, TriggerResponse

class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = []

    def add_node(self, node):
        self.nodes.add(node)

    def add_edge(self, subject, target, action):
        self.edges.append((subject, target, action))
        self.add_node(subject)
        self.add_node(target)

    def has_relation(self, subject, target, action):
        for edge in self.edges:
            if subject == edge[0] and target == edge[1] and action == edge[2]:
                return True
        return False
 
 
    def has_recursive_relation(self, subject, target, action, visited=None):
        """
        Recursively check if a relation exists between subject and target for a given action.
        
        Parameters:
        - subject: The initial subject in the graph (starting from AUV).
        - target: The target object to check the action against.
        - action: The action we want to verify.
        - visited: Set to keep track of visited nodes and prevent cycles.
        
        Returns:
        - True if the action is allowed (directly or indirectly), False otherwise.
        """
        if visited is None:
            visited = set()
        
        # Direct check
        if self.has_relation(subject, target, action):
            return True
        
        # Mark this subject as visited to avoid cycles
        visited.add(subject)
        
        # Get all outgoing relations from the current subject
        for relation in self.get_outgoing_relations(subject):
            intermediate_target = relation['target']
            intermediate_action = relation['action']
            
            # If an intermediate action matches or leads to the desired action, continue recursively
            if intermediate_action == action or (intermediate_target not in visited and self.has_recursive_relation(intermediate_target, target, action, visited)):
                return True

        # If no valid path was found
        return False
 
 
    def get_outgoing_relations(self, subject):
        """
        Retrieve all outgoing relations from a given subject node.

        Parameters:
        - subject: The node for which outgoing relations are requested.

        Returns:
        - List of dictionaries, each representing an outgoing relation with keys:
          'target' (the destination node) and 'action' (the action taken).
        """
        outgoing_relations = []
        for edge in self.edges:
            edge_subject, edge_target, edge_action = edge
            if edge_subject == subject:
                outgoing_relations.append({'target': edge_target, 'action': edge_action})
        return outgoing_relations
 
       
    def validate_mission(self, mission_file):
        """
        Validates a mission file by checking each action against the affordance graph.
        """
        try:
            # Open and read the mission file
            with open(mission_file, 'r') as file:
                mission_lines = file.readlines()

            for line in mission_lines:
                line = line.strip()
                if not line:
                    continue

                # Parse each mission line in the format "Subject do Action" or "Subject look at Target"
                parts = line.split()
                
                if parts[1] == "do":  # Example: "AUV do Survey"
                    subject = parts[0]
                    action = "allows"
                    target = parts[2]

                elif parts[1] == "look_at":  # Example: "AUV look_at Sphere"
                    subject = parts[0]
                    action = "observe"  # Map "look at" to "observe" as per affordance graph terminology
                    target = parts[2]
                    rospy.set_param('/target', target)

                else:
                    rospy.logerr(f"Invalid mission format: {line}")
                    return False

                # Check the required relation in the graph
                if not self.has_relation(subject, target, action):
                    rospy.logwarn(f"Mission validation failed: '{subject} {action} {target}' is not possible.")
                    return False

            rospy.loginfo("Mission validated successfully!")
            return True

        except FileNotFoundError:
            rospy.logerr("Mission file not found: {}".format(mission_file))
            return False
        except Exception as e:
            rospy.logerr("Error validating mission file: {}".format(e))
            return False
    
    @classmethod
    def read_graph_from_yaml(cls, file_path):
        with open(file_path, 'r') as file:
             data = yaml.safe_load(file)
        graph_data = data['graph']
        graph = cls()
        for entry in graph_data:
            graph.add_edge(entry['subject'], entry['target'], entry['action'])
        return graph

    def export_to_yaml(self, filename):
        try:
            with open(filename, 'w') as file:
                data = {'graph': []}
                for edge in self.edges:
                    data['graph'].append({'subject': edge[0], 'target': edge[1], 'action': edge[2]})
                yaml.dump(data, file)
            return True
        except Exception as e:
            rospy.logerr("Failed to export graph to YAML: {}".format(e))
            return False
        
class GUIWindow:
    def __init__(self, graph, taxonomy, taxonomy_file):
        self.graph = graph
        self.taxonomy = taxonomy  # Pass the taxonomy as a dictionary
        self.taxonomy_file = taxonomy_file  # File path to the taxonomy YAML file
        self.root = tk.Tk()
        self.root.title("ROS GUI")

        self.add_object_button = tk.Button(self.root, text="Add Object", command=self.add_object)
        self.add_object_button.pack()

        self.add_affordance_button = tk.Button(self.root, text="Add Affordance", command=self.add_affordance)
        self.add_affordance_button.pack()

        self.add_taxonomy_button = tk.Button(self.root, text="Add Object to Taxonomy", command=self.add_object_to_taxonomy)
        self.add_taxonomy_button.pack()
        
        self.save_button = tk.Button(self.root, text="Save and Finish", command=self.save_and_finish)
        self.save_button.pack()

    def add_object(self):
        object_name = simpledialog.askstring("Add Object", "Enter object name:")
        if object_name:
            rospy.loginfo("Adding object: {}".format(object_name))
            self.graph.add_node(object_name)
            rospy.loginfo("Object added successfully!")

    def add_affordance(self):
        subject = simpledialog.askstring("Add Affordance", "Enter subject object:")
        if subject:
            target = simpledialog.askstring("Add Affordance", "Enter target object:")
            action = simpledialog.askstring("Add Affordance", "Enter action:")
            if target and action:
                rospy.loginfo("Adding affordance: Subject={}, Target={}, Action={}".format(subject, target, action))
                self.graph.add_edge(subject, target, action)
                    
    def add_object_to_taxonomy(self):
        """
        Prompts the user to add a new object with associated actions to the taxonomy.
        If the object already exists, it allows adding new actions to that object.
        The updated taxonomy is then saved to the YAML file.
        """
        object_name = simpledialog.askstring("Add Object to Taxonomy", "Enter object name:")
        if not object_name:
            rospy.logwarn("No object name entered.")
            return
        
        # Check if the object already exists in the taxonomy
        if object_name in self.taxonomy:
            rospy.loginfo("Object '{}' already exists in the taxonomy.".format(object_name))
            # Prompt for new actions to add to the existing object
            actions_input = simpledialog.askstring("Add Actions to Existing Object", 
                                                     "Enter actions to add (comma-separated):")
            if actions_input:
                new_actions = [action.strip() for action in actions_input.split(',') if action.strip()]
                if new_actions:
                    # Append new actions to the existing object's actions
                    self.taxonomy[object_name].extend(new_actions)
                    rospy.loginfo("Added actions {} to object '{}'.".format(new_actions, object_name))
                else:
                    rospy.logwarn("No valid actions entered.")
            else:
                rospy.logwarn("No actions entered.")
        
        else:
            # The object does not exist; proceed to add it
            actions_input = simpledialog.askstring("Add Object to Taxonomy", "Enter actions (comma-separated):")
            if actions_input:
                actions = [action.strip() for action in actions_input.split(',') if action.strip()]
                if actions:
                    rospy.loginfo("Adding object to taxonomy: {} with actions {}".format(object_name, actions))
                    
                    # Add the new object to the taxonomy dictionary
                    self.taxonomy[object_name] = actions
                    
                    # Save the updated taxonomy to the YAML file
                    self.save_taxonomy()
                else:
                    rospy.logwarn("No valid actions entered.")
            else:
                rospy.logwarn("No actions entered.")

    def save_taxonomy(self):
        """
        Saves the current state of the taxonomy to the YAML file.
        """
        try:
            with open(self.taxonomy_file, 'w') as file:
                yaml.dump({'objects': [{'name': name, 'actions': act} for name, act in self.taxonomy.items()]}, file)
            rospy.loginfo("Object and actions saved to taxonomy successfully!")
        except Exception as e:
            rospy.logerr("Failed to save taxonomy to YAML: {}".format(e))
            

    def save_and_finish(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def handle_gui_request(req, graph):
    rospy.loginfo("Received GUI request, displaying GUI...")
    gui_window = GUIWindow(graph, taxonomy, taxonomy_filename)
    gui_window.run()
    return []

def handle_display_graph(req, graph):
    # Example usage
    g = graphviz.Digraph('G')
    
    for edge in graph.edges:
        subject, target, action = edge
        if not target:
            g.node(subject)
        else:
            g.edge(subject, target, label=action)
    
    g.view()
    return True  

def handle_check_relation(req, graph):
    return graph.has_relation(req.subject, req.target, req.action)
    
def handle_export_graph(req, graph):
    try:
        success = graph.export_to_yaml(req.filename)
        return ExportGraphResponse(success=success)
    except Exception as e:
        rospy.logerr("Failed to handle export graph service request: {}".format(e))
        return ExportGraphResponse(success=False)

def handle_validate_mission(req, graph):
    """
    Service handler to validate the mission file.
    """
    # Assume mission file path is set as a parameter or hard-coded here
    mission_file = rospy.get_param('/mission_file')
    
    rospy.loginfo("Validating mission from file: {}".format(mission_file))
    
    if graph.validate_mission(mission_file):
        return TriggerResponse(success=True, message="Mission validated successfully. All requirements met.")
    else:
        return TriggerResponse(success=False, message="Mission validation failed. Requirements not met.")
   
   
def load_taxonomy(file_path):
    """
    Load the taxonomy from a YAML file.
    Returns a dictionary with object names as keys and their actions as lists of strings.
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    taxonomy_data = {obj['name']: obj['actions'] for obj in data['objects']}
    return taxonomy_data



def handle_get_actions(req, graph, taxonomy):
    """
    Service handler for retrieving possible actions for an object.
    
    Parameters:
    - req: The request object containing the label (object name).
    - graph: The affordance graph instance.
    - taxonomy: Dictionary with object actions from taxonomy.
    
    Returns:
    - GetActionsResponse with a list of possible actions for the object.
    """

    label = req.label
    possible_actions = []

    # Check if the object exists in the taxonomy
    if label in taxonomy:
        # Get the actions allowed by the taxonomy for this object
        object_actions = taxonomy[label]
        
        # Verify each action in the context of the affordance graph, allowing recursive checks
        for action in object_actions:
            if graph.has_recursive_relation('AUV', label, action):
                possible_actions.append(action)
    
    # Return the list of possible actions as response
    return GetActionsResponse(actions=possible_actions)

   
    
def gui_service(graph):
    rospy.init_node("affordance_graph") 
    rospy.Service('display_gui', Empty, lambda req: handle_gui_request(req, graph))
    rospy.Service("display_graph", DisplayGraph, lambda req: handle_display_graph(req, graph))
    rospy.Service("query", CheckRelation, lambda req: handle_check_relation(req, graph))
    rospy.Service("validate_mission", Trigger, lambda req: handle_validate_mission(req, graph))
    rospy.Service("get_actions", GetActions, lambda req: handle_get_actions(req, graph, taxonomy))
    rospy.loginfo("GUI service is ready.")
    rospy.spin()

if __name__ == "__main__":
    filename = rospy.get_param('/affordance_graph')
    taxonomy_filename=rospy.get_param('/taxonomy')
    rospy.loginfo("Using filename: %s", filename)
    print("Filename parameter:", filename)
    graph = Graph.read_graph_from_yaml(filename)
    taxonomy = load_taxonomy(taxonomy_filename)
    gui_service(graph)

