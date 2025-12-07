# Chapter 7: Simulation with Gazebo

## 7.1 Introduction to Gazebo: Physics-Based Robot Simulator

Robot development, especially for complex systems like humanoids, often involves extensive testing, which can be time-consuming, expensive, and potentially dangerous on physical hardware. This is where high-fidelity robot simulators become indispensable. **Gazebo** is one such powerful 3D robot simulator, widely adopted in the robotics community and particularly well-integrated with the Robot Operating System (ROS) ecosystem.

Gazebo provides the ability to accurately simulate populations of robots, complex environments, and a variety of sensors. Its core strength lies in its **physics engine**, which handles gravity, rigid body dynamics, fluid dynamics (to some extent), and accurate collision detection. This allows researchers and developers to:

*   **Test algorithms safely**: Experiment with control systems, navigation algorithms, and perception pipelines without risking damage to expensive physical hardware or endangering humans.
*   **Rapid prototyping**: Quickly iterate on robot designs and software implementations.
*   **Reproducible experiments**: Conduct experiments in identical virtual conditions.
*   **Train AI models**: Generate large datasets for training machine learning models for perception and control.
*   **Visualize robot behavior**: Gain insights into complex robot-environment interactions.

Gazebo supports various robot models, typically described in SDF (Simulation Description Format) or URDF (Unified Robot Description Format), and can simulate a wide array of sensors (cameras, LiDAR, IMUs, contact sensors) and actuators (motors, joints).

## 7.2 Installing and Launching Gazebo with ROS 2

Integrating Gazebo with ROS 2 allows you to simulate your robot within a realistic environment and control it using ROS 2 nodes, topics, and services, just as you would a real robot.

### 7.2.1 Installation

Gazebo is often installed alongside ROS 2 (e.g., the `ros-humble-desktop` metapackage includes Gazebo Classic). If you need to install it separately, or a specific version of Gazebo, you can do so:

```bash
# Install Gazebo Garden (default for ROS 2 Humble onwards)
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs # Installs Gazebo and ROS 2 bridge packages

# Or, if you prefer Gazebo Classic (older, but still widely used)
# sudo apt install ros-humble-gazebo-ros-pkgs # This will usually pull in Gazebo Classic
# If you need to install Gazebo Classic directly:
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
# wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# sudo apt update
# sudo apt install gazebo11 # For Gazebo Classic 11
```
*Note: Ensure your ROS 2 environment is sourced before using Gazebo with ROS 2.*

### 7.2.2 Launching Gazebo

You can launch Gazebo in various ways:
*   **Stand-alone**: `gazebo` (launches an empty world)
*   **With a specific world file**: `gazebo <path_to_world_file.world>`
*   **Via ROS 2 launch files**: This is the most common method when working with ROS 2, as it allows you to launch Gazebo alongside your robot model and ROS 2 control nodes.

*Example of a simple ROS 2 launch file (`my_robot_gazebo.launch.py`):*

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your robot's URDF file
    robot_description_path = os.path.join(
        get_package_share_directory('my_robot_description'), # Replace with your package
        'urdf',
        'my_robot.urdf'
    )

    # Launch Gazebo world
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')

    # Load robot description into a parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_path}]
    )

    # Spawn your robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', robot_description_path],
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': 'empty.world'}.items() # Or your custom world
        ),
        robot_state_publisher_node,
        spawn_entity
    ])
```
*To launch: `ros2 launch my_robot_package my_robot_gazebo.launch.py`*

## 7.3 Creating and Importing Robot Models in Gazebo

Gazebo uses SDF as its native format for describing models. However, it can also import and convert URDF models, making it compatible with the ROS ecosystem.

### 7.3.1 URDF to SDF Conversion

When you spawn a URDF model in Gazebo using ROS 2 tools (like `spawn_entity.py` in the example above), Gazebo often performs an internal conversion to SDF. For more complex interactions or Gazebo-specific features, you might want to create a full SDF model.

### 7.3.2 SDF Model Structure

An SDF model typically includes:
*   **`<model>`**: The root element for a model.
*   **`<link>`**: Similar to URDF links, defining rigid body parts with visual, collision, and inertial properties.
*   **`<joint>`**: Connects links, defining motion. SDF supports more joint types than URDF.
*   **`<sensor>`**: Native definition of various sensor types (camera, LiDAR, IMU) with their properties.
*   **`<plugin>`**: Allows extending Gazebo's functionality (e.g., for ROS 2 control interfaces).

Models are typically stored in the `models` directory of your Gazebo installation or in directories specified in the `GAZEBO_MODEL_PATH` environment variable.

### 7.3.3 Importing Humanoid Models

To simulate a humanoid, you'll need its URDF or SDF description. Many open-source humanoid models are available, such as `Atlas` or `Nao`. You can typically place these model files within a ROS 2 package (e.g., `my_humanoid_description/urdf/humanoid.urdf`) and then use a launch file to load and spawn it into Gazebo.

## 7.4 Simulating Sensors and Actuators in Gazebo

Gazebo provides extensive capabilities for simulating various robot sensors and actuators, making it a valuable tool for developing and testing control algorithms.

### 7.4.1 Simulating Sensors

Gazebo allows you to add virtual sensors to your robot models, whose data can be published as ROS 2 topics. This includes:

*   **Cameras**: Monocular, stereo, depth cameras, publishing `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`.
*   **LiDAR/Range Sensors**: Simulating laser scans, publishing `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`.
*   **IMUs**: Publishing `sensor_msgs/msg/Imu`.
*   **Contact Sensors**: Detecting collisions, publishing `gazebo_msgs/msg/ContactsState`.
*   **GPS**: Publishing `sensor_msgs/msg/NavSatFix`.

These sensors are typically configured within the SDF model file or through Gazebo plugins that bridge sensor data to ROS 2 topics.

### 7.4.2 Simulating Actuators and Control

To control your robot in Gazebo via ROS 2, you typically use `ros2_control` (previously `ros_control` in ROS 1). `ros2_control` is a set of packages that provide a generic control architecture for robots, allowing you to use the same high-level control code for both simulated and real robots.

Gazebo plugins (e.g., `libgazebo_ros2_control.so`) are used to:

*   **Bridge Joints**: Connect the simulated joints in Gazebo to the `ros2_control` hardware interface.
*   **Implement Controllers**: Load and run various controllers (position, velocity, effort controllers) that subscribe to ROS 2 command topics and publish joint states.

*Example of a conceptual `ros2_control` snippet within a URDF (or xacro, its macro-based extension):*

```xml
<!-- Example: Conceptual ros2_control definition for a joint -->
<ros2_control name="robot_controller" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">0</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```
*This snippet, combined with appropriate Gazebo plugins and controller configurations, allows you to send velocity commands to `left_wheel_joint` via ROS 2 topics.*

## 7.5 Basic World Building and Environment Interaction

Gazebo is not just for robots; it's also a powerful tool for creating and interacting with virtual environments.

### 7.5.1 Creating Custom Worlds

Gazebo worlds are defined in `.world` SDF files. These files can include:

*   **Static Models**: Buildings, furniture, obstacles from Gazebo's model database or custom models.
*   **Lights**: Sunlight, spotlights, point lights with configurable properties.
*   **Ground Plane**: A simple flat surface.
*   **Physics Properties**: Gravity, friction coefficients, global step size.

### 7.5.2 Environment Interaction

*   **Manipulation**: Robots can interact with objects in the world, picking them up, pushing them, or placing them.
*   **Collision Detection**: The physics engine automatically handles collisions between objects and the robot, providing contact information.
*   **Sensory Feedback**: Simulated sensors provide data influenced by the environment's properties.
*   **User Interface**: Gazebo's GUI allows users to directly manipulate objects, add new models, and inspect properties of the simulation.

By building rich and interactive worlds, developers can thoroughly test robot behaviors in diverse scenarios, from simple pick-and-place tasks to complex navigation in cluttered urban environments.

## 7.6 Summary & Exercises

Chapter 7 introduced Gazebo as an indispensable physics-based robot simulator, highlighting its role in safe, rapid, and reproducible robot development. We covered the practical steps for installing and launching Gazebo within a ROS 2 environment, emphasizing the use of ROS 2 launch files. The chapter detailed how robot models, primarily URDF and SDF, are integrated into Gazebo and how various sensors and actuators are accurately simulated. Finally, we explored the capabilities for basic world building and environment interaction, underscoring Gazebo's power in creating diverse testing scenarios for complex robotic systems.

### Exercises

1.  Explain the primary advantages of using a physics-based simulator like Gazebo for humanoid robot development.
2.  Outline the key components of a ROS 2 launch file used to spawn a robot in Gazebo.
3.  Describe the role of URDF/SDF in defining robot models for Gazebo simulation.
4.  How does `ros2_control` facilitate the control of simulated robot actuators in Gazebo?
5.  Design a simple `.world` file for Gazebo that includes a ground plane, a light source, and two static box obstacles.
