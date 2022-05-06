# ur3_moveit Package

The ur3_moveit package was written by Unity and we will use it in this project mostly unmodified during the simulation portion. In this part, we will go over the contents of the package, the nodes that this package launches and what they do.

The package has three scripts that launch nodes that are located in the /scripts/ folder. They are: `pose_estimation_script.py`, `mover.py`, and `server_endpoint.py`.  These scripts are written in python and are launched with a file with a .launch extension within the command console. The `server_endpoint.py` script is very simple:

```py
#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from ur3_moveit.msg import *
from ur3_moveit.srv import *

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        'UR3Trajectory': RosSubscriber('UR3Trajectory', UR3Trajectory, tcp_server),
        'ur3_moveit': RosService('ur3_moveit', MoverService),
        'pose_estimation_srv': RosService('pose_estimation_service', PoseEstimationService)
    })

    rospy.spin()


if __name__ == "__main__":
    main()
```

It creates a way for Unity to connect to ROS for routing messages to the correct nodes.

The `pose_estimation_script.py` script takes an image from Unity and runs the model to extract position and orientation of the object.

```py
#!/usr/bin/env python3

from ur3_moveit.setup_and_run_model import *

import rospy
import io
import os
import math

from ur3_moveit.srv import PoseEstimationService, PoseEstimationServiceResponse
from PIL import Image, ImageOps
from geometry_msgs.msg import Point, Quaternion, Pose
from scipy.spatial.transform import Rotation as R

NODE_NAME = "PoseEstimationNode"
PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/scripts"))] # remove "/scripts"
MODEL_PATH = PACKAGE_LOCATION + "/models/UR3_single_cube_model.tar"

count = 0


def _save_image(req):
    """  convert raw image data to a png and save it
    Args:
        req (PoseEstimationService msg): service request that contains the image data
     Returns:
        image_path (str): location of saved png file
    """
    global count
    count += 1

    image_height = req.image.width
    image_width = req.image.height
    image = Image.frombytes('RGBA', (image_width,image_height), req.image.data)
    image = ImageOps.flip(image)
    image_name = "Input" + str(count) + ".png"
    if not os.path.exists(PACKAGE_LOCATION + "/images/"):
        os.makedirs(PACKAGE_LOCATION + "/images/")
    image_path = PACKAGE_LOCATION + "/images/" + image_name
    image.save(image_path)
    return image_path


def _run_model(image_path):
    """ run the model and return the estimated posiiton/quaternion
    Args:
        image_path (str): location of saved png file
     Returns:
        position (float[]): object estmated x,y,z
        quaternion (float[]): object estimated w,x,y,z
    """
    output = run_model_main(image_path, MODEL_PATH)
    position = output[1].flatten()
    quaternion = output[0].flatten()
    return position, quaternion


def _format_response(est_position, est_rotation):
    """ format the computed estimated position/rotation as a service response
       Args:
           est_position (float[]): object estmated x,y,z
           est_rotation (float[]): object estimated Euler angles
        Returns:
           response (PoseEstimationServiceResponse): service response object
       """
    position = Point()
    position.x = est_position[0]
    position.y = est_position[1]
    position.z = est_position[2]

    rotation = Quaternion()
    rotation.x = est_rotation[0]
    rotation.y = est_rotation[1]
    rotation.z = est_rotation[2]
    rotation.w = est_rotation[3]
    
    pose = Pose()
    pose.position = position
    pose.orientation = rotation

    response = PoseEstimationServiceResponse()
    response.estimated_pose = pose
    return response


def pose_estimation_main(req):
    """  main callback for pose est. service.
    Args:
        req (PoseEstimationService msg): service request that contains the image data
     Returns:
       response (PoseEstimationServiceResponse): service response object
    """
    print("Started estimation pipeline")
    image_path = _save_image(req)
    print("Predicting from screenshot " + image_path)
    est_position, est_rotation = _run_model(image_path)
    response = _format_response(est_position, est_rotation)
    print("Finished estimation pipeline\n")
    return response


def main():
    """
     The function to run the pose estimation service
     """
    rospy.init_node(NODE_NAME)
    s = rospy.Service('pose_estimation_service', PoseEstimationService, pose_estimation_main)
    print("Ready to estimate pose!")
    rospy.spin()


if __name__ == "__main__":
    main()

```
It converts the raw data image into a png, and runs the model.  Then it outputs the position and orientation as a response and sends it back to Unity as a Vector3 and Quaternion.  This node can be modified if the project is expanded to include the identification and/or classification of multiple objects.  Details on other uses can be found [here](https://github.com/Unity-Technologies/com.unity.perception).

The `mover.py` script takes current joint state of the robot and start and end positions of the object and outputs an array of joint positions.  This array are small slices of the robot's path between all of the important points of travel.  

The robot moves between 6 points:
1)  At rest, where it is above the object and the image is taken with the wrist camera.
2)  The pre-grasp position, directly above the object.
3)  The grasp position, at the object.
4)  The pre-grasp position again, to lift the object away from the table surface.
5)  The place position, where the object will be released by the gripper.
6)  the post-place position, where the robot moves up from its place position to clear the gripper from the object.

Excluding the rest position, there are only 4 unique positions.  The `mover` node engages the trajectory planner built in to ROS to output a path of travel for the robot, appends them to an array and sends that array as a response back to Unity.  Unity then iterates the robot thorough each set of joint positions, stopping only to open or close the gripper at pre-defined points.

It is possible to modify this script in order to add/remove additional points or to adjust the drop-off height (which is a bit high).

```py
from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur3_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)

    return planCompat(move_group.plan())


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position
    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved
    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    setup_scene()

    response = MoverServiceResponse()

    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_pose)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.075  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_pose)

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pick_up_pose)

    #add pre place pose for box with walls

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(place_pose)

    # Post place - lift gripper after placement position
    place_pose = copy.deepcopy(req.place_pose)
    place_pose.position.z += 0.075 # Static value offset to lift up gripper
    post_place_pose = plan_trajectory(move_group, place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = post_place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(post_place_pose)

    move_group.clear_pose_targets()
    return response

def setup_scene():
    global move_group
    
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Add table collider to MoveIt scene
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = move_group.get_planning_frame()
    p = set_pose(p, [0, 0, -0.02+0.77])
    scene.add_box("table", p, (1.2, 1.8, 0.01))

def set_pose(poseStamped, pose):
    '''
    pose is an array: [x, y, z]
    '''
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    return poseStamped

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_moveit_server')

    s = rospy.Service('ur3_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")

    rospy.spin()


if __name__ == "__main__":
    moveit_server()
```

### Proceed to [Part 9](9_data_collection_model_training.md).
