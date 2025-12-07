# Chapter 12: Unity Visualization & Interaction

Unity provides high-fidelity visualization and interaction capabilities for robotics development. This chapter covers integrating Unity with ROS 2 for creating immersive digital twin environments.

---

## Why Unity for Robotics?

| Capability | Benefit |
|------------|---------|
| **High-quality rendering** | Realistic visualization for stakeholders |
| **Cross-platform** | Windows, Linux, VR/AR devices |
| **Asset ecosystem** | Pre-made environments, characters, objects |
| **C# scripting** | Rapid UI and interaction development |
| **ROS integration** | Unity Robotics Hub packages |

### Unity vs Gazebo

| Aspect | Unity | Gazebo |
|--------|-------|--------|
| **Rendering** | Excellent | Good |
| **Physics accuracy** | Good | Excellent |
| **ROS integration** | Via bridge | Native |
| **Learning curve** | Moderate | Moderate |
| **Best for** | Visualization, VR/AR | Physics simulation |

---

## Setting Up Unity for ROS 2

### Installation Requirements

1. Unity Hub and Unity Editor (2021.3 LTS or newer)
2. ROS 2 Humble on Ubuntu or Windows
3. Unity Robotics packages

### Installing Unity Robotics Packages

In Unity Package Manager, add packages by git URL:

```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

### ROS 2 Endpoint Setup

```bash
# Install ROS TCP Endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Launch the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

---

## Importing Robot Models

### URDF Import

1. In Unity: **Assets → Import Robot from URDF**
2. Select your `.urdf` file
3. Configure import settings:

```csharp
// URDFImporter settings
UrdfRobotExtensions.Create(
    urdfPath,
    new ImportSettings {
        chosenAxis = ImportSettings.axisType.yAxis,
        convexMethod = ImportSettings.convexDecomposer.vHACD
    }
);
```

### Fixing Common Import Issues

| Issue | Solution |
|-------|----------|
| Meshes not found | Update mesh paths in URDF |
| Wrong orientation | Adjust `chosenAxis` setting |
| Missing materials | Create Unity materials manually |
| Collision issues | Use convex decomposition |

---

## ROS 2 Communication

### Publishing from Unity

```csharp
// JointStatePublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity/joint_states";
    public float publishRate = 50f;

    private ArticulationBody[] joints;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);

        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= 1f / publishRate)
        {
            PublishJointStates();
            timeElapsed = 0;
        }
    }

    void PublishJointStates()
    {
        JointStateMsg msg = new JointStateMsg();
        msg.header.stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (int)Time.time,
            nanosec = (uint)((Time.time % 1) * 1e9)
        };

        var names = new string[joints.Length];
        var positions = new double[joints.Length];
        var velocities = new double[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            names[i] = joints[i].name;
            positions[i] = joints[i].jointPosition[0];
            velocities[i] = joints[i].jointVelocity[0];
        }

        msg.name = names;
        msg.position = positions;
        msg.velocity = velocities;

        ros.Publish(topicName, msg);
    }
}
```

### Subscribing in Unity

```csharp
// JointCommandSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;

public class JointCommandSubscriber : MonoBehaviour
{
    public string topicName = "/joint_trajectory";
    private ArticulationBody[] joints;
    private Dictionary<string, ArticulationBody> jointMap;

    void Start()
    {
        joints = GetComponentsInChildren<ArticulationBody>();
        jointMap = new Dictionary<string, ArticulationBody>();

        foreach (var joint in joints)
        {
            jointMap[joint.name] = joint;
        }

        ROSConnection.GetOrCreateInstance()
            .Subscribe<JointTrajectoryMsg>(topicName, OnJointCommand);
    }

    void OnJointCommand(JointTrajectoryMsg msg)
    {
        if (msg.points.Length == 0) return;

        var point = msg.points[0];

        for (int i = 0; i < msg.joint_names.Length; i++)
        {
            string jointName = msg.joint_names[i];
            if (jointMap.ContainsKey(jointName))
            {
                var joint = jointMap[jointName];
                var drive = joint.xDrive;
                drive.target = (float)point.positions[i] * Mathf.Rad2Deg;
                joint.xDrive = drive;
            }
        }
    }
}
```

---

## Camera Streaming

### Streaming Unity Camera to ROS

```csharp
// CameraStreamer.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraStreamer : MonoBehaviour
{
    public Camera targetCamera;
    public string topicName = "/unity/camera/image";
    public int width = 640;
    public int height = 480;
    public float publishRate = 30f;

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        renderTexture = new RenderTexture(width, height, 24);
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
        targetCamera.targetTexture = renderTexture;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= 1f / publishRate)
        {
            PublishImage();
            timeElapsed = 0;
        }
    }

    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        byte[] imageData = texture2D.GetRawTextureData();

        // Flip vertically (Unity vs ROS coordinate convention)
        byte[] flippedData = FlipVertically(imageData, width, height, 3);

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "unity_camera"
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(width * 3),
            data = flippedData
        };

        ros.Publish(topicName, msg);
    }

    byte[] FlipVertically(byte[] data, int width, int height, int channels)
    {
        byte[] flipped = new byte[data.Length];
        int rowSize = width * channels;

        for (int y = 0; y < height; y++)
        {
            int srcRow = y * rowSize;
            int dstRow = (height - 1 - y) * rowSize;
            System.Array.Copy(data, srcRow, flipped, dstRow, rowSize);
        }

        return flipped;
    }
}
```

---

## Interactive UI for Robot Control

### Control Panel

```csharp
// RobotControlUI.cs
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotControlUI : MonoBehaviour
{
    public Slider[] jointSliders;
    public Text[] jointLabels;
    public Button emergencyStopButton;
    public Toggle motorsToggle;

    private ROSConnection ros;
    private string commandTopic = "/unity/joint_commands";
    private string estopTopic = "/emergency_stop";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64MultiArrayMsg>(commandTopic);
        ros.RegisterPublisher<BoolMsg>(estopTopic);

        emergencyStopButton.onClick.AddListener(OnEmergencyStop);
        motorsToggle.onValueChanged.AddListener(OnMotorsToggle);

        foreach (var slider in jointSliders)
        {
            slider.onValueChanged.AddListener((val) => OnJointSliderChanged());
        }
    }

    void OnJointSliderChanged()
    {
        Float64MultiArrayMsg msg = new Float64MultiArrayMsg();
        msg.data = new double[jointSliders.Length];

        for (int i = 0; i < jointSliders.Length; i++)
        {
            msg.data[i] = jointSliders[i].value * Mathf.Deg2Rad;
        }

        ros.Publish(commandTopic, msg);
    }

    void OnEmergencyStop()
    {
        BoolMsg msg = new BoolMsg { data = true };
        ros.Publish(estopTopic, msg);
        Debug.LogWarning("EMERGENCY STOP TRIGGERED");
    }

    void OnMotorsToggle(bool enabled)
    {
        // Call enable motors service
    }
}
```

### Status Display

```csharp
// StatusDisplay.cs
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class StatusDisplay : MonoBehaviour
{
    public Text balanceStatus;
    public Text batteryLevel;
    public Text connectionStatus;
    public Image balanceIndicator;

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<StringMsg>("/robot/status", OnStatusReceived);
    }

    void OnStatusReceived(StringMsg msg)
    {
        // Parse JSON status
        var status = JsonUtility.FromJson<RobotStatus>(msg.data);

        balanceStatus.text = status.is_balanced ? "Balanced" : "UNSTABLE";
        balanceIndicator.color = status.is_balanced ? Color.green : Color.red;
        batteryLevel.text = $"Battery: {status.battery_percent}%";
    }

    void Update()
    {
        connectionStatus.text = ROSConnection.GetOrCreateInstance().HasConnectionThread
            ? "Connected" : "Disconnected";
    }
}

[System.Serializable]
public class RobotStatus
{
    public bool is_balanced;
    public float battery_percent;
    public string current_state;
}
```

---

## VR/AR Integration

### VR Robot Teleoperation

```csharp
// VRTeleoperation.cs
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VRTeleoperation : MonoBehaviour
{
    public Transform leftController;
    public Transform rightController;
    public string leftArmTopic = "/left_arm/target_pose";
    public string rightArmTopic = "/right_arm/target_pose";

    private ROSConnection ros;
    private InputDevice leftHand;
    private InputDevice rightHand;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(leftArmTopic);
        ros.RegisterPublisher<PoseStampedMsg>(rightArmTopic);

        // Get VR controllers
        var leftHandDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, leftHandDevices);
        if (leftHandDevices.Count > 0) leftHand = leftHandDevices[0];

        var rightHandDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, rightHandDevices);
        if (rightHandDevices.Count > 0) rightHand = rightHandDevices[0];
    }

    void Update()
    {
        // Check grip buttons for activation
        bool leftGrip, rightGrip;
        leftHand.TryGetFeatureValue(CommonUsages.gripButton, out leftGrip);
        rightHand.TryGetFeatureValue(CommonUsages.gripButton, out rightGrip);

        if (leftGrip)
        {
            PublishPose(leftController, leftArmTopic);
        }

        if (rightGrip)
        {
            PublishPose(rightController, rightArmTopic);
        }
    }

    void PublishPose(Transform controller, string topic)
    {
        PoseStampedMsg msg = new PoseStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "base_link"
            },
            pose = new PoseMsg
            {
                position = new PointMsg
                {
                    x = controller.position.z,  // Unity Z → ROS X
                    y = -controller.position.x, // Unity X → ROS -Y
                    z = controller.position.y   // Unity Y → ROS Z
                },
                orientation = new QuaternionMsg
                {
                    x = controller.rotation.z,
                    y = -controller.rotation.x,
                    z = controller.rotation.y,
                    w = controller.rotation.w
                }
            }
        };

        ros.Publish(topic, msg);
    }
}
```

---

## Environment Creation

### Procedural Room Generation

```csharp
// RoomGenerator.cs
using UnityEngine;

public class RoomGenerator : MonoBehaviour
{
    public float roomWidth = 10f;
    public float roomLength = 10f;
    public float roomHeight = 3f;
    public Material floorMaterial;
    public Material wallMaterial;

    void Start()
    {
        GenerateRoom();
    }

    void GenerateRoom()
    {
        // Floor
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.transform.localScale = new Vector3(roomWidth/10, 1, roomLength/10);
        floor.GetComponent<Renderer>().material = floorMaterial;

        // Walls
        CreateWall(new Vector3(0, roomHeight/2, roomLength/2),
                   new Vector3(roomWidth, roomHeight, 0.1f));
        CreateWall(new Vector3(0, roomHeight/2, -roomLength/2),
                   new Vector3(roomWidth, roomHeight, 0.1f));
        CreateWall(new Vector3(roomWidth/2, roomHeight/2, 0),
                   new Vector3(0.1f, roomHeight, roomLength));
        CreateWall(new Vector3(-roomWidth/2, roomHeight/2, 0),
                   new Vector3(0.1f, roomHeight, roomLength));
    }

    void CreateWall(Vector3 position, Vector3 scale)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.transform.position = position;
        wall.transform.localScale = scale;
        wall.GetComponent<Renderer>().material = wallMaterial;
        wall.isStatic = true;
    }
}
```

---

## Summary

This chapter covered Unity integration for robotics:

*   **Setup**: Unity Robotics Hub packages and ROS TCP Endpoint
*   **Robot Import**: URDF to Unity workflow
*   **ROS Communication**: Publishing and subscribing from Unity
*   **Camera Streaming**: Unity cameras as ROS image topics
*   **Interactive UI**: Control panels and status displays
*   **VR/AR**: Immersive teleoperation interfaces

Unity complements Gazebo by providing superior visualization and interaction capabilities. In the next chapter, we will explore URDF and SDF formats in depth.
