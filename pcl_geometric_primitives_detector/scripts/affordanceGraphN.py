#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import tkinter as tk
from tkinter import simpledialog
import graphviz
import yaml
from pcl_geometric_primitives_detector.srv import AddObject, AddAffordance, ExportGraph, CreateGUI, DisplayGraph, CheckRelation, QueryFullGraph, GetTaxonomy, QueryFullGraphRequest, QueryFullGraphResponse, GetTaxonomyRequest, GetTaxonomyResponse
from std_srvs.srv import Trigger, TriggerResponse
import roslib

taxonomy_data=[]

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

    # New handler for querying the full graph
def handle_query_graph(req, graph):
        response=QueryFullGraphResponse()

        # Collect the nodes
        response.nodes = list(graph.nodes)
        # Collect the edges as three separate lists (subject, target, action)
        subject_list = []
        target_list = []
        action_list = []
        
        for edge in graph.edges:
            subject, target, action = edge
            subject_list.append(subject)
            target_list.append(target)
            action_list.append(action)
        
        response.subject = subject_list
        response.target = target_list
        response.action = action_list
        
        return QueryFullGraphResponse()

def handle_get_taxonomy(request, taxonomy):
    response=GetTaxonomyResponse()

    response.object_names=[]
    response.actions=[]
    response.action_offsets=[]

    offset=0
    for object_names, actions in taxonomy.items():
        response.object_names.append(object_names)
        response.actions.extend(actions)
        response.action_offsets.append(offset)
        offset=len(response.actions)

    print(response.object_names)

    return response
   
def load_taxonomy(file_path):
        """
        Load the taxonomy from a YAML file.
        Returns a dictionary with object names as keys and their actions as lists of strings.
        """
        with open(file_path, 'r') as file:
             data = yaml.safe_load(file)
             taxonomy_data = {obj['name']: obj['actions'] for obj in data['objects']}
             print(taxonomy_data)
        return taxonomy_data
   
    
def gui_service(graph):
    rospy.init_node("affordance_graph") 
    rospy.Service('display_gui', Empty, lambda req: handle_gui_request(req, graph))
    rospy.Service("display_graph", DisplayGraph, lambda req: handle_display_graph(req, graph))
    rospy.Service("query", CheckRelation, lambda req: handle_check_relation(req, graph))
    rospy.Service("query_full_graph", QueryFullGraph, lambda req: handle_query_graph(req, graph))
    rospy.Service("get_taxonomy", GetTaxonomy, lambda req: handle_get_taxonomy(req, taxonomy))
    rospy.loginfo("GUI service is ready.")
    rospy.spin()

if __name__ == "__main__":
    filename = roslib.packages.get_pkg_dir('pcl_geometric_primitives_detector')+'/ace/affordance_graph.yaml'
    taxonomy_filename=roslib.packages.get_pkg_dir('pcl_geometric_primitives_detector')+'/ace/primitive_taxonomy.yaml'
    
    rospy.loginfo("Using filename: %s", filename)
    print("Filename parameter:", filename)
    graph = Graph.read_graph_from_yaml(filename)
    taxonomy = load_taxonomy(taxonomy_filename)
    gui_service(graph)

