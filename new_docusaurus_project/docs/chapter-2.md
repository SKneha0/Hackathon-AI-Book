---
sidebar_position: 2
---

# Chapter 2: Describing Your Robot with URDF

## The Anatomy of a Robot: Links and Joints

Before a robot can be controlled, simulated, or even visualized, its physical structure must be precisely defined. A robot's body is fundamentally a collection of rigid bodies, called **links**, connected by **joints** that allow relative motion. This chain of links and joints forms the robot's kinematic structure, essentially its skeleton.

For a humanoid robot, these links and joints are organized to form legs, arms, a torso, and a head, mimicking human anatomy. The complexity of a robot is often measured by its **Degrees of Freedom (DoF)**, which is the total number of independent movements it can make. Each joint typically contributes one or more DoF. For example, a human-like leg might have 3 DoF at the hip, 1 DoF at the knee, and 2 DoF at the ankle, totaling 6 DoF per leg. This high dimensionality makes controlling humanoids particularly challenging.

## Unified Robot Description Format (URDF)

The **Unified Robot Description Format (URDF)** is an XML-based standard used extensively in the ROS ecosystem to represent a robot's physical structure. An URDF file defines the robot's links, joints, their visual appearance, collision properties, and inertial characteristics. This comprehensive description allows ROS tools, simulators like Gazebo, and visualization tools like RViz to understand and interact with the robot model.

Let's model a very simple two-link arm to understand the core components of URDF. The arm will have a "base" link (fixed to the world), a "humerus" (upper arm) link, and an "ulna" (lower arm) link.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Link: base_link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint: shoulder_joint connects base_link and humerus_link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="humerus_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- Link: humerus_link -->
  <link name="humerus_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Joint: elbow_joint connects humerus_link and ulna_link -->
  <joint name="elbow_joint" type="revolute">
    <parent link="humerus_link"/>
    <child link="ulna_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.8" effort="100" velocity="10"/>
  </joint>

  <!-- Link: ulna_link -->
  <link name="ulna_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
    </inertial>
  </link>

</robot>
```

### Understanding the URDF Tags

*   `<robot>`: The root element, defining the robot's name.
*   `<link>`: Defines a rigid body. Key sub-elements are:
    *   `<visual>`: Describes what the link looks like (geometry, color, origin). Used for visualization in tools like RViz.
    *   `<collision>`: Defines the collision geometry, used by physics engines (e.g., in Gazebo) to detect contacts. This is often a simplified shape to improve performance.
    *   `<inertial>`: Crucial for physics simulation, defining the link's mass, center of mass, and inertia tensor. These properties dictate how the link responds to forces and torques.
*   `<joint>`: Defines the connection between two links.
    *   `name` and `type`: The joint's name and its type (e.g., `revolute` for a rotating joint, `prismatic` for a sliding joint, `fixed` for a rigid connection).
    *   `<parent>` and `<child>`: Define the kinematic hierarchy. The `child` link moves relative to the `parent` link.
    *   `<origin>`: Specifies the 3D position (xyz) and orientation (rpy - roll, pitch, yaw) of the joint relative to the parent link's origin.
    *   `<axis>`: For revolute/prismatic joints, defines the axis of rotation or translation.
    *   `<limit>`: Specifies the joint's operational range (lower/upper limits), and maximum effort and velocity.

## ROS 2 Integration with URDF

A standalone URDF file describes the robot, but to bring it to life in a ROS 2 system, we need additional components:

1.  **`robot_state_publisher`:** This ROS 2 node reads the URDF file and the current joint states (typically published by motor encoders or a simulation) and broadcasts the robot's kinematic state (the position and orientation of all its links) as TF2 transformations. This allows other ROS nodes to know where every part of the robot is in 3D space.

2.  **`joint_state_publisher` (or equivalent in simulation):** In a real robot, joint states come from hardware. In simulation or for visualization, a `joint_state_publisher` node can be used to publish mock joint states, allowing you to manually move the robot's joints in RViz.

Here's a simplified view of how URDF integrates into a ROS 2 system for visualization:

```mermaid
graph TD
    A[URDF File] --> B(robot_state_publisher Node);
    C[Joint States<br/>(e.g., from encoders or simulation)] --> B;
    B --> D((/tf Topic));
    D --> E[RViz Visualization Tool];

    subgraph ROS 2 System
        B; C; D; E;
    end
```
*Diagram 2.1: URDF integration with ROS 2 for robot state broadcasting and visualization in RViz.*

To launch a URDF model and its publishers in ROS 2, you would typically use a launch file (as seen in Chapter 1). Here's a conceptual snippet for launching the `simple_arm` for visualization:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to your URDF file
    # This assumes your URDF is in 'description/urdf/simple_arm.urdf' within a package named 'my_robot_description'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_arm.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui', # For manual control in RViz
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('my_robot_description'), 'rviz', 'urdf_config.rviz')],
            output='screen'
        )
    ])
```
In this launch file:
*   `robot_state_publisher` reads the `simple_arm.urdf` and publishes the robot's link transforms.
*   `joint_state_publisher_gui` provides a GUI slider to manually control the joint angles, which then feed into the `robot_state_publisher`.
*   `rviz2` (ROS Visualization) is launched to display the robot model in 3D space.

## Exercises

1.  **URDF Modification:** Take the provided `simple_arm.urdf` and modify it to add a third link, the `hand_link`, connected to the `ulna_link` by a `wrist_joint`. This joint should be a `revolute` joint allowing rotation around the `ulna_link`'s main axis. Define appropriate `visual`, `collision`, and `inertial` tags for the `hand_link`.

2.  **Conceptual Question:** Explain the difference in purpose between the `<visual>` and `<collision>` tags in a URDF file. Why is it common for them to define different geometries for the same link?

3.  **Kinematic Chains:** A mobile robot has a base, two driven wheels, and a simple one-DoF gripper arm mounted on its top. Draw a simple kinematic chain diagram and identify all the links and joints. How many total Degrees of Freedom (DoF) does this robot have?

4.  **ROS 2 and URDF Roles:** Describe the distinct roles of the `URDF file`, the `robot_state_publisher` node, and `RViz` in visualizing a robot within the ROS 2 ecosystem. How do they work together?

5.  **Debugging URDF:** You have created a complex URDF for a new humanoid leg, but when you launch it in RViz, parts of the leg are floating disconnected from others, or moving incorrectly. List three common mistakes in URDF files that could lead to such visualization problems.
