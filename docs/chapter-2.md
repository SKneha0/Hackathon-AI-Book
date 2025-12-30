---
sidebar_position: 2
---

# Chapter 2: Describing Your Robot with URDF

## The Anatomy of a Robot: Links and Joints

For any robotic system to be effectively controlled, simulated, or even visualized, its physical structure must be precisely defined. A robot is fundamentally composed of **links**—rigid bodies representing its segments (e.g., a forearm, a wheel, a torso)—interconnected by **joints**, which permit relative motion between these links. This arrangement of links and joints forms the robot's kinematic chain, often referred to as its "skeleton."

In humanoid robotics, links and joints are configured to emulate human anatomy, forming limbs, a torso, and a head. The complexity of a robot is often quantified by its **Degrees of Freedom (DoF)**, representing the total number of independent movements it can execute. Each joint typically contributes one or more DoF. For instance, a human-like hip might offer three DoF, a knee one DoF, and an ankle two DoF, culminating in six DoF per leg. The high dimensionality in humanoid robots significantly complicates their control.

## Unified Robot Description Format (URDF)

The **Unified Robot Description Format (URDF)** is an XML-based specification widely adopted within the ROS ecosystem for defining a robot's physical and kinematic properties. An URDF file encapsulates the robot's links, joints, visual characteristics, collision geometries, and inertial parameters. This comprehensive description is instrumental for various ROS tools, physics simulators such as Gazebo, and visualization applications like RViz to accurately interpret and interact with the robot model.

Let's construct a simple two-link manipulator arm to illustrate the fundamental components of URDF. This arm will consist of a `base_link` (assumed fixed to the world), a `humerus_link` (upper arm), and an `ulna_link` (lower arm).

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Define a common material for visualization -->
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>

  <!-- Link: base_link - The root of our arm -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint: shoulder_joint - Connects base_link and humerus_link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="humerus_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/> <!-- Joint origin relative to parent link -->
    <axis xyz="0 1 0"/> <!-- Rotation around Y-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- Link: humerus_link - The upper arm segment -->
  <link name="humerus_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/> <!-- Visual origin relative to its link's origin -->
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint: elbow_joint - Connects humerus_link and ulna_link -->
  <joint name="elbow_joint" type="revolute">
    <parent link="humerus_link"/>
    <child link="ulna_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/> <!-- Joint origin relative to parent link -->
    <axis xyz="0 1 0"/> <!-- Rotation around Y-axis -->
    <limit lower="0" upper="2.8" effort="100" velocity="10"/>
  </joint>

  <!-- Link: ulna_link - The lower arm segment -->
  <link name="ulna_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/> <!-- Visual origin relative to its link's origin -->
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

### Understanding Key URDF Tags

The primary elements within a URDF file are `<link>` and `<joint>`, nested within the root `<robot>` tag.

*   **`<link>`:** Represents a rigid segment of the robot.
    *   **`<visual>`:** Defines the graphical representation of the link, used by visualization tools. It includes `<geometry>` (e.g., `box`, `cylinder`, `mesh`), `<origin>` (position and orientation relative to the link's frame), and `<material>` (color).
    *   **`<collision>`:** Describes the link's physical boundary for collision detection. Often a simplified geometric primitive to optimize physics engine performance.
    *   **`<inertial>`:** Specifies the link's mass properties, including `<mass>`, `<origin>` (center of mass), and `<inertia>` (inertia tensor), critical for accurate physical simulation.
*   **`<joint>`:** Connects two links, defining their relative motion.
    *   `name` and `type`: Unique identifier and kinematic type (e.g., `revolute` for rotation, `prismatic` for translation, `fixed` for rigid connection).
    *   **`<parent>`** and **`<child>`:** Specify the `parent` and `child` links in the kinematic hierarchy, indicating that the child link moves relative to the parent.
    *   **`<origin>`:** Defines the 3D pose (position `xyz` and orientation `rpy` - roll, pitch, yaw) of the joint's frame relative to the parent link's frame.
    *   **`<axis>`:** For revolute and prismatic joints, specifies the axis of rotation or translation.
    *   **`<limit>`:** For actuated joints, defines the operational range (`lower`, `upper`), maximum `effort`, and `velocity`.

## ROS 2 Integration with URDF

A URDF file provides a static description of a robot. To dynamically integrate this model into a ROS 2 system, allowing for real-time visualization and simulation, additional ROS 2 nodes are employed:

1.  **`robot_state_publisher`:** This node is fundamental. It reads the URDF model and the current joint states (typically sourced from robot sensors or a simulator) and subsequently broadcasts the robot's complete kinematic state as TF2 transformations. This enables any other ROS 2 node to query the precise 3D position and orientation of every link on the robot.

2.  **`joint_state_publisher` (or `joint_state_publisher_gui`):** In a real robotic system, actual joint positions are provided by hardware encoders. For development, simulation, or simple visualization purposes, `joint_state_publisher` or its GUI variant can publish synthetic or manually controlled joint states, which are then consumed by the `robot_state_publisher`.

The following diagram illustrates how URDF integrates into a ROS 2 system for visualization:

```mermaid
graph TD
    A[URDF File] --> B(robot_state_publisher Node);
    C[Joint States Source<br/>(e.g., Hardware, Simulator, joint_state_publisher_gui)] --> B;
    B --> D((/tf and /tf_static Topics));
    D --> E[RViz2 Visualization Tool];

    subgraph ROS 2 Ecosystem
        B; C; D; E;
    end
```
*Figure 2.1: Illustration of URDF integration within a ROS 2 ecosystem, detailing the flow from URDF definition and joint states to the `robot_state_publisher` for TF2 broadcasting, culminating in 3D visualization within RViz2.*

To instantiate and visualize a URDF model in ROS 2, a launch file (as introduced in Chapter 1) is typically used. Here’s a conceptual Python launch file snippet to visualize our `simple_arm`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments for robot_description (URDF content) and use_sim_time
    urdf_robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=Command(['xacro ', os.path.join(
            get_package_share_directory('my_robot_description'),
            'urdf',
            'simple_arm.urdf.xacro')]) # Using xacro for more advanced URDFs
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Joint State Publisher GUI Node (for manual control in RViz2)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2 Node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('my_robot_description'),
            'rviz',
            'urdf_config.rviz'
        )],
        output='screen'
    )

    return LaunchDescription([
        urdf_robot_description_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
```
This Python launch file:
*   Utilizes `xacro` (a macro language for XML) for a more modular URDF definition, referencing `simple_arm.urdf.xacro`.
*   Launches `robot_state_publisher` to broadcast the robot's kinematics based on the URDF.
*   Starts `joint_state_publisher_gui` to allow interactive manipulation of joint angles, which feed into the `robot_state_publisher`.
*   Initializes `rviz2` to render the 3D robot model, often configured with a specific RViz configuration file.

---

### Exercises

1.  **URDF Extension:** Extend the `simple_arm` URDF to include a `gripper_link` attached to the `ulna_link` via a `fixed` joint. The gripper should have a simple box geometry. Explain why a `fixed` joint is appropriate here if the gripper is non-articulated.
2.  **URDF Parameterization (Conceptual):** Research the `xacro` format. Explain its advantages over plain URDF, particularly for complex robots with many similar components (e.g., a humanoid with two symmetric legs). How would you use `xacro` to define a single leg and then instantiate it for both the left and right sides of a humanoid?
3.  **Kinematic vs. Visual vs. Collision:** You are designing a mobile robot with a large, transparent dome for its sensor housing.
    a. How would you represent this dome in the `<visual>` tag?
    b. How might you represent it differently in the `<collision>` tag for optimal physics simulation, and why?
    c. What implications do your choices have for the robot's `inertial` properties?
4.  **ROS 2 Launch File Modification:** Modify the provided Python launch file to:
    a. Remove the `joint_state_publisher_gui` and instead launch a simple `joint_state_publisher` (non-GUI version) that publishes a fixed set of joint states (e.g., all joints at 0 radians).
    b. Explain how `use_sim_time` argument is passed and its significance when integrating with simulators like Gazebo.
5.  **Debugging Challenge:** You've created a URDF and launched it, but in RViz2, your robot appears as a single point, or only one link is visible. What are the most likely causes for this behavior, and what debugging steps would you take in ROS 2?