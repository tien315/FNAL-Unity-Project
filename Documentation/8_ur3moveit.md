# ur3_moveit Package

The ur3_moveit package was written by Unity and we will use it in this project mostly unmodified during the simulation portion. In this part, we will go over the contents of the package, the nodes that this package launches and what they do.

The package has three scripts that launch nodes that are located in the /scripts/ folder. They are: `pose_estimation.py`, `mover.py`, and `server_endpoint.py`.  These scripts are written in python and are launched with a file with a .launch extension within the command console. The `server_endpoint.py` script is very simple:

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
