# Chapter 6: Robot Description: URDF, SDF & Kinematics

## 6.1 Describing Your Robot: Unified Robot Description Format (URDF)

To accurately simulate, visualize, and control a robot, its physical characteristics and capabilities must be precisely defined. The **Unified Robot Description Format (URDF)** is an XML-based file format used in ROS (both ROS 1 and ROS 2) to describe a robot's kinematic and dynamic properties. It's designed to be a human-readable and machine-interpretable way to model your robot.

URDF primarily focuses on describing the robot's structure:
*   **Links**: Represent the rigid segments of the robot (e.g., torso, upper arm, wheel). They have associated inertial properties (mass, inertia matrix) and visual/collision geometries.
*   **Joints**: Connect two links and define their relative motion. Each joint specifies its type (e.g., `revolute`, `prismatic`, `fixed`), axis of rotation/translation, origin (position and orientation relative to the parent link), and limits.

### Key Elements of a URDF File:

*   **`<robot>`**: The root element, containing all links and joints.
*   **`<link>`**: Defines a rigid body segment.
    *   **`<visual>`**: Describes the graphical model of the link (e.g., `mesh` for 3D models, `box`, `cylinder`, `sphere` for primitives).
    *   **`<collision>`**: Describes the collision model of the link, used by physics engines for collision detection. Often a simplified version of the visual geometry.
    *   **`<inertial>`**: Defines the mass, center of mass, and inertia matrix of the link. Essential for realistic physics simulation.
*   **`<joint>`**: Defines the connection between two links.
    *   **`name`**: Unique identifier for the joint.
    *   **`type`**: `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`.
    *   **`<parent link="..." />`**: Specifies the parent link.
    *   **`<child link="..." />`**: Specifies the child link.
    *   **`<origin xyz="..." rpy="..." />`**: Relative position and orientation of the joint to the parent link.
    *   **`<axis xyz="..." />`**: Axis of rotation or translation for revolute/prismatic joints.
    *   **`<limit lower="..." upper="..." velocity="..." effort="..." />`**: Joint limits for movement, velocity, and effort.

```xml
<!-- Example: Simplified URDF for a differential drive mobile robot -->
<robot name="my_mobile_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Wheel Link -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.1 0" rpy="1.5707 0 0"/> <!-- Rotated 90 deg around X-axis -->
    <axis xyz="0 0 1"/>
  </joint>

  <!-- (Similar structure for right wheel) -->

</robot>
```
*Note: This is a simplified example. A complete robot description often includes more links, joints, and advanced properties.*

## 6.2 Advanced Robot Description: Simulation Description Format (SDF)

While URDF is excellent for describing a single robot's kinematic and dynamic properties, it has limitations, especially for complex simulation environments. The **Simulation Description Format (SDF)** is another XML-based format primarily used by Gazebo for describing robots, static environments, and sensors in a more comprehensive way.

### Key Differences and Advantages of SDF over URDF:

*   **Environmental Description**: SDF can describe not just robots, but also entire worlds, including static objects (buildings, furniture), lights, terrain, and atmospheric properties. URDF is limited to a single robot.
*   **Multiple Robots**: An SDF file can contain descriptions of multiple robots and other entities in a single world file.
*   **Joint Types**: SDF supports a richer set of joint types and properties.
*   **Sensors**: SDF has native support for describing various sensor types (cameras, LiDAR, IMUs) directly within the model, including their noise characteristics and update rates. URDF often requires external plugins for sensor simulation.
*   **Plugins**: SDF provides a robust plugin architecture for extending functionality within Gazebo simulation.
*   **Nested Models**: SDF supports nested models, allowing for hierarchical organization of complex systems.

When developing for Gazebo, it's common to convert a URDF model to SDF, or to write models directly in SDF for more detailed simulation capabilities. ROS 2 tools often handle the conversion transparently.

## 6.3 Forward Kinematics: Robot Pose from Joint Angles

**Kinematics** is the branch of mechanics that describes the motion of objects without considering the forces that cause the motion. In robotics, it primarily deals with the spatial configuration of a robot's links and joints.

**Forward Kinematics (FK)** is the process of calculating the position and orientation (pose) of a robot's end-effector (or any point on the robot) in Cartesian space, given the values of all its joint variables (e.g., joint angles for revolute joints, linear displacements for prismatic joints).

### How it Works:

FK typically involves a series of transformations. Each joint and link segment is represented by a homogeneous transformation matrix (4x4 matrix combining rotation and translation). By multiplying these matrices sequentially from the robot's base to its end-effector, the final pose of the end-effector relative to the base frame can be determined.

*   **Denavit-Hartenberg (DH) Parameters**: A widely used convention for systematically assigning coordinate frames to the links of a kinematic chain and deriving the transformation matrices.

### Importance:

*   **Visualization**: Crucial for rendering robots in simulation and visualization tools (like RViz in ROS 2).
*   **Robot State Monitoring**: Knowing the current pose of all parts of the robot.
*   **Collision Detection**: Used to calculate the current positions of all links for checking against obstacles.

```python
# Conceptual Python snippet for Forward Kinematics (using a simplified 2R planar arm)
import numpy as np

def forward_kinematics_2R_planar(l1, l2, theta1, theta2):
    """
    Calculates the end-effector position (x, y) for a 2R planar robot arm.
    l1: length of first link
    l2: length of second link
    theta1: angle of first joint (radians)
    theta2: angle of second joint (radians)
    """
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

# Example usage:
l1 = 1.0  # meters
l2 = 1.0  # meters
theta1 = np.pi / 4  # 45 degrees
theta2 = np.pi / 2  # 90 degrees

end_effector_x, end_effector_y = forward_kinematics_2R_planar(l1, l2, theta1, theta2)
print(f"End-effector position: ({end_effector_x:.2f}, {end_effector_y:.2f})")
```
*Note: Real-world FK for complex robots (especially humanoids) involves many more links, joints, and often specialized robotics libraries.*

## 6.4 Inverse Kinematics: Joint Angles from Desired Pose

**Inverse Kinematics (IK)** is the process of calculating the required joint variables (angles or displacements) for a robot's end-effector to reach a desired position and orientation in Cartesian space. This is often a more challenging problem than forward kinematics, as it can have multiple solutions, no solutions, or singularities.

### How it Works:

IK problems can be solved using various methods:

*   **Analytical Solutions**: Closed-form mathematical equations that directly compute joint angles. Only possible for robots with simpler kinematic structures (typically with a limited number of DOFs and specific joint configurations).
*   **Numerical Solutions**: Iterative optimization algorithms that search for joint angles that minimize the error between the current end-effector pose and the desired pose. More general but computationally intensive and can get stuck in local minima.
*   **Geometric Solutions**: Using geometric relationships for simpler planar or spherical wrists.

### Importance:

*   **Path Planning**: Crucial for making robots move to specific locations in their environment.
*   **Trajectory Generation**: Generating smooth movements for the end-effector.
*   **Humanoid Control**: For making humanoid hands reach for objects or enabling specific body postures.

```python
# Conceptual Python snippet for Inverse Kinematics (simplified)
# This is a highly conceptual example and does not represent a full IK solver.
# Real IK solvers are complex and often rely on numerical methods or specialized libraries.
import numpy as np

def inverse_kinematics_2R_planar(l1, l2, x, y):
    """
    Calculates joint angles (theta1, theta2) for a 2R planar robot arm
    to reach a desired end-effector position (x, y).
    This is a highly simplified analytical solution with potential ambiguities.
    """
    # Calculate theta2 (using law of cosines)
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if cos_theta2 > 1 or cos_theta2 < -1:
        # Cannot reach target
        return None, None
    theta2 = np.arctan2(np.sqrt(1 - cos_theta2**2), cos_theta2) # Elbow up solution (one of two possible)

    # Calculate theta1
    alpha = np.arctan2(y, x)
    beta = np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    theta1 = alpha - beta

    return theta1, theta2

# Example usage:
l1 = 1.0
l2 = 1.0
target_x = 0.5
target_y = 1.5

theta1_rad, theta2_rad = inverse_kinematics_2R_planar(l1, l2, target_x, target_y)

if theta1_rad is not None:
    print(f"Joint angles: theta1={np.degrees(theta1_rad):.2f} deg, theta2={np.degrees(theta2_rad):.2f} deg")
else:
    print("Target unreachable.")
```
*Note: This is a highly simplified and specific analytical IK example. Numerical IK solvers are generally required for humanoids.*

## 6.5 Introduction to Robot Control with Inverse Kinematics

Inverse kinematics is not an end in itself but a critical tool for robot control. Once the desired joint angles are computed from an IK solution, these angles become the setpoints for a robot's joint controllers.

### Control Loop with IK:

1.  **Desired End-Effector Trajectory**: Define the path the end-effector should follow in Cartesian space (e.g., move hand from A to B).
2.  **IK Solver**: At each step of the trajectory, an IK solver computes the corresponding joint angles.
3.  **Joint Controllers**: These joint angles are fed to low-level joint position (or velocity/effort) controllers, which command the robot's motors to move to those positions.
4.  **Feedback**: Proprioceptive sensors (encoders) provide feedback on the actual joint positions, allowing the controller to correct for errors.

For humanoids, IK is essential for tasks like:
*   **Grasping**: Positioning the hand correctly relative to an object.
*   **Balance**: Adjusting leg and torso joint angles to maintain center of mass over the support polygon.
*   **Walking**: Generating leg trajectories that enable stable bipedal locomotion.

The integration of IK into a real-time control loop is a complex task, requiring careful consideration of dynamics, singularities, and computational efficiency. Robotics middleware like ROS 2 often provides specialized packages (e.g., MoveIt 2 for motion planning) that abstract away much of this complexity, allowing developers to focus on higher-level task planning.

## 6.6 Summary & Exercises

Chapter 6 provided a deep dive into how robots are described and how their motion is understood and controlled. We started with URDF (Unified Robot Description Format) for specifying a robot's physical structure, including links and joints, and contrasted it with SDF (Simulation Description Format) for more complex simulation environments. The foundational concepts of kinematics were explored, distinguishing between forward kinematics (calculating end-effector pose from joint angles) and inverse kinematics (calculating joint angles from a desired end-effector pose). Finally, we saw how inverse kinematics serves as a vital bridge for higher-level robot control, translating task-space goals into executable joint-space commands, particularly crucial for the intricate movements of humanoids.

### Exercises

1.  Explain the primary purpose of URDF and identify its key components. Provide a scenario where SDF would be preferred over URDF.
2.  Given a 3R planar manipulator (three links, three revolute joints) with known link lengths and joint angles, describe the general approach to calculate the end-effector position using forward kinematics.
3.  Why is inverse kinematics generally more computationally challenging than forward kinematics? What are some common issues encountered in IK solutions for complex robots?
4.  Describe the role of inverse kinematics in enabling a humanoid robot to grasp an object at a specific location in its environment.
5.  What are Denavit-Hartenberg (DH) parameters, and why are they useful in the context of robot kinematics?
