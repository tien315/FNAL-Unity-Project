# FNAL Unity Project Tutorial: Part 1

In this first part of the tutorial, we will start by learning how to create a URDF. URDF stands for Unified Robot Description Format. The file is written in XML and is a way to describe the links, joints, shape, and movement limits.  Most robot description files you will encounter online are written not as URDFs but as XACROs or XML macros. XACROs are an easy way to separate a URDF into parts so that the URDF of a complex robot isn't extremely long and difficult to parse.  For example, there may be a XACRO for links and joints, and another for the transmission controlling the rate of joint rotation for all joints. The XACROs of different components are referenced by a 'main' XACRO that arranges them in their proper order.

Another way to use XACROs is to have a XACRO of a robot arm, and XACROs of each accessory. This way, a robot arm can be attached to many different kinds of accessories such as a gripper, a camera, or sensors, all without having to rewrite a URDF from scratch for each combination.

Unfortunately for us, Unity cant import XACROs so we will have to assemble our robot and convert our collection of XACROs into a single URDF. This tutorial will show the step-by-step process of creating a Universal-Robots model UR3e robot arm equipped with a Robotiq 2F-85 2-fingered gripper. At the time of writing this tutorial, a XACRO of the Robotiq Wrist Camera is not available.  It is possible to use the STL [provided by Robotiq](https://assets.robotiq.com/website-assets/support_documents/document/WRIST_CAMERA_20171116.STEP) to [create one](http://wiki.ros.org/sw_urdf_exporter), but it would be purely for aesthetics as its only real function in the simulation is as a spacer.

**Table of Contents**
  - [Requirements](#reqs)
  - [Putting the files in the correct place](#step-1)
  - [Edit the XACRO](#step-2)
  - [Converting our XACROs into a URDF](#step-3)

---

### <a name="reqs">Requirements</a>

To follow this tutorial you need to **clone** the following repositories. They contain the XACROs as well as the visual and collision meshes we will need to assemble our robot.

1. Open a terminal and navigate to the folder where you want to host the repository.

This repository contains the files for all the Universal-Robots products.
```bash
git clone --recurse-submodules https://github.com/ros-industrial/universal_robot
```
This repository contains the files for all the Robotiq products.
```bash
git clone --recurse-submodules https://github.com/ros-industrial/robotiq
```
This repository contains the files for all a custom XACRO to combine UR and Robotiq products.
```bash
git clone --recurse-submodules https://github.com/cambel/ur3
```




### <a name="step-1">Putting the files in the correct place</a>
This next step will require you to have ROS installed and a workspace created.  You can go [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) if you havent done it yet.

1. Navigate to the /catkin_ws/src/ directory. You will need 3 things moved here.
2. Move or copy the universal_robot repository you cloned previously.
3. Move or copy the robotiq repository you cloned previously.
4. Move or copy the ur3_description/urdf/ur3_robotiq85_gripper.urdf.xacro file located in the cambel/ur3 repository you cloned previously.

<p align="center">
<img src="linux_screencap.png" align="center" width=950/>
</p>

5. Open a terminal and navigate to the catkin_ws folder. Copy the following command to rebuild your catkin workspace with the new packages you have copied over so that ROS can search for files in the repositories you have copied over.

```bash
cd catkin_ws
catkin_make
```



### <a name="step-2">Edit the XACRO</a>

Once the files have been copied over, open the ur3_with_gripper.xacro file with a text editor. The XACRO written here is for a UR3 robot with a 2F-140 gripper so we will make some small modifications. First we will change the comments to reflect the changes we are making. Any references to ur3 or ur5 should be changed to ur3e. Next, on line 5, there is a reference to ur_description. We need to change it to ur_e_decription to access the files for the e-Series of robots. Similarly, the ur3.urdf.xacro filename at the end of line 5 should be changed to ur3e.urdf.xacro. Continue to change all references to match our equipment including references to the gripper and you should have something that looks like this:

<p align="center">
<img src="xacro_screencap.png"/>
</p>




### <a name="step-3">Converting our XACROs into a URDF</a>

Now that we have finished editing the ur3_with_gripper.xacro file, save it and then rename the file to ur3e_with_gripper.xacro to reflect the changes we made. Open a Terminal and navigate to the catkin_ws folder, then source your setup file by copying the following:

```bash
source devel/setup.bash
```
Then copy the following command to use the XACRO conversion function within ROS to convert our XACRO into a URDF:

```bash
rosrun xacro xacro --inorder -o ur3e_with_gripper.urdf ur3e_with_gripper.xacro
```

This command follows the format: <!--rosrun xacro xacro --inorder -o [filename of new URDF] [filename of xacro]-->

You can name the file whatever you want, but keep it simple and descriptive because we will need to keep track of it across both ROS and Unity. After running the command, you should have a URDF in the same location as the XACRO. Make a copy of this file and keep track of it because we will be editing it and the edited version will be copied over into two places. It is important that they be identical or there will be problems later.




### Proceed to [Part 2](2_set_up_the_scene.md).
