import rospy
from std_srvs.srv import Empty
import tkinter as tk
from tkinter import simpledialog
import graphviz
import yaml
from cola2_stonefish.srv import AddObject, AddAffordance, ExportGraph, CreateGUI, DisplayGraph, DisplayGraphResponse, CheckRelation

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
    def __init__(self, graph):
        self.graph = graph
        self.root = tk.Tk()
        self.root.title("ROS GUI")

        self.add_object_button = tk.Button(self.root, text="Add Object", command=self.add_object)
        self.add_object_button.pack()

        self.add_affordance_button = tk.Button(self.root, text="Add Affordance", command=self.add_affordance)
        self.add_affordance_button.pack()

        self.save_button = tk.Button(self.root, text="Save and Finish", command=self.save_and_finish)
        self.save_button.pack()

    def add_object(self):
        object_name = simpledialog.askstring("Add Object", "Enter object name:")
        if object_name:
            rospy.loginfo("Adding object: {}".format(object_name))
            try:
                add_object = rospy.ServiceProxy('add_object_service_name', AddObject)
                response = add_object(object_name)
                if response.success:
                    self.graph.add_node(object_name)
                    rospy.loginfo("Object added successfully!")
                else:
                    rospy.logwarn("Failed to add object!")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def add_affordance(self):
        subject = simpledialog.askstring("Add Affordance", "Enter subject object:")
        if subject:
            target = simpledialog.askstring("Add Affordance", "Enter target object:")
            action = simpledialog.askstring("Add Affordance", "Enter action:")
            if target and action:
                rospy.loginfo("Adding affordance: Subject={}, Target={}, Action={}".format(subject, target, action))
                try:
                    add_affordance = rospy.ServiceProxy('add_affordance_service_name', AddAffordance)
                    response = add_affordance(subject, target, action)
                    if response.success:
                        self.graph.add_edge(subject, target, action)
                        rospy.loginfo("Affordance added successfully!")
                    else:
                        rospy.logwarn("Failed to add affordance!")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: {}".format(e))


    def save_and_finish(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def handle_gui_request(req, graph):
    rospy.loginfo("Received GUI request, displaying GUI...")
    gui_window = GUIWindow(graph)
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
    
def gui_service(graph):
    rospy.init_node("affordance_graph") 
    rospy.Service('display_gui', Empty, lambda req: handle_gui_request(req, graph))
    rospy.Service("display_graph", DisplayGraph, lambda req: handle_display_graph(req, graph))
    rospy.Service("query", CheckRelation, lambda req: handle_check_relation(req, graph))
    rospy.loginfo("GUI service is ready.")
    rospy.spin()

if __name__ == "__main__":
    filename = rospy.get_param('/affordance_graph/affordance_graph_filename', './../../ace/affordance_graph.yaml')
    rospy.loginfo("Using filename: %s", filename)
    print("Filename parameter:", filename)
    graph = Graph.read_graph_from_yaml(filename)
    gui_service(graph)

