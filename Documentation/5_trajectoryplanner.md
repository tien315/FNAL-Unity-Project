#TrajectoryPlanner.cs and how it works

The `TrajectoryPlanner.cs`script is the main script that handles the tasks of the scenario.  It captures the image, sends and receives responses to the pose estimation and mover nodes in ROS and moves the robot model by making changes to the joint positions.  We will go over some important parts of the code below.

```C#
// Hardcoded variables 
private int numRobotJoints = 6;
private readonly float jointAssignmentWait = 0.06f;
private readonly float poseAssignmentWait = 0.5f;
private readonly float gripperAngle = 37.5f;//original 35f

// Offsets to ensure gripper is above grasp points
// Adjust this to change the height of the TCP if it is hitting the table or missing the object
private readonly Vector3 pickPoseOffset = new Vector3(0, 0.240f, 0);//original (0, 0.255f, 0)  
private readonly Vector3 placePoseOffset = new Vector3(0, 0.276f, 0);//.276f
```

Near the top of the script, some of the variables are hardcoded for ease of use with Unity.  Because the gripper controls are handled in Unity and not considered as part of the robot kinematics in ROS, the gripper angle when closed has to be manually adjusted.  This is important if the with of the object changes. Too wide and the gripper will fail to grasp.  Too tight and the finger joints will clip through each other.

The TCP or tool center point offset are also set here.  Again, because the gripper is not considered in the kinematics of the robot in ROS, the end effector is the flange at the end of wrist 3.  So we need to offset the end effector position upward to give space for the gripper.  For very small objects, it may be necessary to fine tune this variable for consistent results during simulation.

```C#
public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle : 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            gripperJoints[i].xDrive = curXDrive;
            //Debug.Log(gripperJoints[i].name);
        }
        //yield return new WaitForSeconds(jointAssignmentWait);
        yield return new WaitForSeconds(3);
    }

/// <summary>
///     For manual control of the gripper position via button click within the simulation
/// </summary>
public void squeeze()
{
    StartCoroutine(IterateToGrip(true));
}
public void release()
{
    StartCoroutine(IterateToGrip(false));
}
```

This portion of code handles the gripper control.  A simple for loop iterates through all of the joints in the gripper and moves them to the correct position.  Note that the multipliers are defined because orientation of some joints are flipped.  When the URDF was created, left side and right side joints have their z-axis pointed in opposite directions and so the direction of rotation must be reversed for some joints. 

```C#
public void PoseEstimation() 
{
    InitializeButton.interactable = false;
    RandomizeButton.interactable = false;
    ServiceButton.interactable = false;
    ActualPos.text = target.transform.position.ToString("F3");
    ActualRot.text = target.transform.eulerAngles.ToString();
    EstimatedPos.text = "-";
    EstimatedRot.text = "-";

    // moves the robot arm into position so that the wrist camera can capture the screenshot
    var tempXDrive = jointArticulationBodies[2].xDrive;
    tempXDrive.target = -58f;
    jointArticulationBodies[2].xDrive = tempXDrive;
    tempXDrive = jointArticulationBodies[4].xDrive;
    tempXDrive.target = 90f;
    jointArticulationBodies[4].xDrive = tempXDrive;
    tempXDrive = jointArticulationBodies[5].xDrive;
    tempXDrive.target = 90f;
    jointArticulationBodies[5].xDrive = tempXDrive;

    StartCoroutine(CallPoseEstSvc());
}
```
This is the button callback for the Pose Estimation button.  When the button is pressed in the simulation, the robot moves from rest into a position where the wrist camera is above the object. It then starts the `CallPoseEstSvc` coroutine:

```C#
private IEnumerator CallPoseEstSvc()
    {
        yield return new WaitForSecondsRealtime(2);
        // Capture the screenshot and pass it to the pose estimation service
        Debug.Log("Capturing screenshot...");
        byte[] rawImageData = CaptureScreenshot();
        InvokePoseEstimationService(rawImageData);
    }
```

which captures the image from the camera and passes it to the `InvokePoseEstimationService`.

```C#
/// <summary>
///     Create a new PoseEstimationServiceRequest with the captured screenshot as bytes and instantiates 
///     a sensor_msgs/image.
///
///     Call the PoseEstimationService using the ROSConnection and calls PoseEstimationCallback on the 
///     PoseEstimationServiceResponse.
/// </summary>
/// <param name="imageData"></param>
private void InvokePoseEstimationService(byte[] imageData)
{
uint imageHeight = (uint)renderTexture.height;
uint imageWidth = (uint)renderTexture.width;

RosMessageTypes.Sensor.Image rosImage = new RosMessageTypes.Sensor.Image(new RosMessageTypes.Std.Header(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
PoseEstimationServiceRequest poseServiceRequest = new PoseEstimationServiceRequest(rosImage);
ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);
}
```
This is where Unity begins to interact with ROS. It formats the image into a datatype that is ROS compatible, then sends it to the `pose estimation` node for position extraction.

```C#
ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);
```
This line shows how the request is made: `pose_estimation_srv` is a service that handles sending requests directly to a node. The `poseServiceRequest` is the image after being formated into a ROS message type. The `PoseEstimationCallback` lets ROS know where to send its response back. The code for the `PoseEstimationCallback` function is below.

```C#
void PoseEstimationCallback(PoseEstimationServiceResponse response)
{
if (response != null)
{
    // The position output by the model is the position of the cube relative to the camera so we need to extract its global position 
    var estimatedPosition = Camera.main.transform.TransformPoint(response.estimated_pose.position.From<RUF>());
    var estimatedRotation = Camera.main.transform.rotation * response.estimated_pose.orientation.From<RUF>();
    //Debug.Log(estimatedPosition);
    //Debug.Log(Camera.main.transform.rotation);


    PublishJoints(estimatedPosition, estimatedRotation);

    EstimatedPos.text = estimatedPosition.ToString("F3");
    EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
}
InitializeButton.interactable = true;
RandomizeButton.interactable = true;
}
```

After receiving the position and orientation of the object, the current joint state of the robot is collected and sent to the `mover` node in the same way as in `InvokePoseEstimationService`.  We can see below in the last line that the method is the same and follows the same format, but for a different ROS message datatype.

```C#
public void PublishJoints(Vector3 targetPos, Quaternion targetRot)
{
    MoverServiceRequest request = new MoverServiceRequest();
    request.joints_input = CurrentJointConfig();
    targetPos.y = (0.774f);//override target estimated height and set it to match the table surface height
    // Pick Pose
    request.pick_pose = new RosMessageTypes.Geometry.Pose
    {
        position = (targetPos + pickPoseOffset).To<FLU>(),
        orientation = Quaternion.Euler(90, targetRot.eulerAngles.y, 0).To<FLU>()
    };

    // Place Pose
    request.place_pose = new RosMessageTypes.Geometry.Pose
    {
        position = (goal.position + placePoseOffset).To<FLU>(),
        orientation = pickOrientation.To<FLU>()
    };

    ros.SendServiceMessage<MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
}
```

```C#
void TrajectoryResponse(MoverServiceResponse response)
{
    if (response.trajectories != null && response.trajectories.Length > 0)
    {
        Debug.Log("Trajectory returned.");
        StartCoroutine(ExecuteTrajectories(response));
    }
    else
    {
        Debug.LogError("No trajectory returned from MoverService.");
        InitializeButton.interactable = true;
        RandomizeButton.interactable = true;
        ServiceButton.interactable = true;
    }
}
```

The trajectories are sent back to Unity in the for of arrays of joint positions.  These joint positions are small movements from one pose to another. The arm will move in small increments as the Unity iterates through them so that the arm will follow a very specific path.

```C#
/// <summary>
///     Execute the returned trajectories from the MoverService.
///
///     The expectation is that the MoverService will return four trajectory plans,
///         PreGrasp, Grasp, PickUp, and Place,
///     where each plan is an array of robot poses. A robot pose is the joint angle values
///     of the six robot joints.
///
///     Executing a single trajectory will iterate through every robot pose in the array while updating the
///     joint values on the robot.
/// 
/// </summary>
/// <param name="response"> MoverServiceResponse received from ur3_moveit mover service running in ROS</param>
/// <returns></returns>
private IEnumerator ExecuteTrajectories(MoverServiceResponse response)
{
    if (response.trajectories != null)
    {
        // For every trajectory plan returned
        for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
        {
            // For every robot pose in trajectory plan
            for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();

                // Set the joint values for every joint
                for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive  = jointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    jointArticulationBodies[joint].xDrive = joint1XDrive;
                }
                // Wait for robot to achieve pose for all joint assignments
                yield return new WaitForSeconds(jointAssignmentWait);
            }

            // Close the gripper if completed executing the trajectory for the Grasp pose
            if (poseIndex == (int)Poses.Grasp){
                StartCoroutine(IterateToGrip(true));
                yield return new WaitForSeconds(jointAssignmentWait);
            }
            else if (poseIndex == (int)Poses.Place){
                yield return new WaitForSeconds(poseAssignmentWait);
                // Open the gripper to place the target cube
                StartCoroutine(IterateToGrip(false));
            }
            // Wait for the robot to achieve the final pose from joint assignment
            yield return new WaitForSeconds(poseAssignmentWait);
        }

        // Re-enable buttons
        InitializeButton.interactable = true;
        RandomizeButton.interactable = true;
        yield return new WaitForSeconds(jointAssignmentWait);
    }
}
```

