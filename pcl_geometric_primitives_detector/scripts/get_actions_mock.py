#!/usr/bin/env python

import rospy
from pcl_geometric_primitives_detector.srv import GetActions, GetActionsResponse

def handle_get_actions(req):
    """
    [MOCK] Service handler for retrieving possible actions for an object.

    Parameters:
    - req: The request object containing the label (object name).

    Returns:
    - GetActionsResponse with a list of possible actions for the object.
    """
    label = req.label
    actions = []

    # Determine actions based on the label
    if label == "cylinder":
        actions = ["observe", "touch"]
    elif label == "sphere":
        actions = ["observe", "manipulate"]
    elif label == "box":
        actions = ["observe", "touch", "manipulate"]
    elif label == "cone":
        actions = ["observe"]

    # Return the list of possible actions as response
    return GetActionsResponse(actions=actions)

def get_actions_server():
    rospy.init_node('get_actions_server')
    # Define the service /get_actions_mock with handler handle_get_actions
    service = rospy.Service("get_actions_mock", GetActions, handle_get_actions)
    rospy.loginfo("Service /get_actions_mock is ready.")
    rospy.spin()

if __name__ == "__main__":
    get_actions_server()
