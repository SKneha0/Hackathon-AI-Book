# Chapter 10: VSLAM, Navigation & Path Planning (Nav2)

## 10.1 Visual SLAM (VSLAM): Mapping and Localization

For autonomous robots to navigate effectively in unknown environments, they must solve two fundamental problems simultaneously: **mapping** (building a map of the environment) and **localization** (determining their own position within that map). This coupled problem is known as **Simultaneous Localization and Mapping (SLAM)**. When vision sensors (cameras) are the primary data source, it's referred to as **Visual SLAM (VSLAM)**.

### How VSLAM Works:

VSLAM algorithms typically follow a pipeline:
1.  **Feature Extraction**: Identifying distinctive points or patterns (features) in camera images (e.g., SIFT, ORB, deep learning features).
2.  **Feature Matching**: Tracking the movement of these features across successive image frames.
3.  **Pose Estimation**: Estimating the camera's (robot's) 6-DOF pose (position and orientation) by analyzing the displacement of matched features.
4.  **Map Building**: Using the estimated poses and features to incrementally build or update a representation of the environment. This can be a sparse map (just features), a dense map (3D point cloud or mesh), or a semi-dense map.
5.  **Loop Closure**: Recognizing previously visited locations to correct accumulated errors in the map and robot's trajectory, crucial for long-term consistency.

### Types of VSLAM:

*   **Monocular VSLAM**: Uses a single camera. While simple, it struggles with scale ambiguity (cannot determine absolute distances without additional information) and is less robust to textureless environments.
*   **Stereo VSLAM**: Uses two cameras to provide depth perception, resolving scale ambiguity. More computationally intensive but more robust.
*   **RGB-D VSLAM**: Uses depth cameras (like Intel RealSense) to provide direct depth measurements, simplifying map building and improving accuracy.

VSLAM is a highly active research area, with algorithms like ORB-SLAM and LSD-SLAM pushing the boundaries of real-time, robust localization and mapping. For humanoids, VSLAM is vital for understanding their surrounding space and planning movements within it.

## 10.2 Introduction to ROS 2 Navigation Stack (Nav2)

Once a robot has a map of its environment and can localize itself within it, the next challenge is to autonomously navigate from one point to another while avoiding obstacles. The **ROS 2 Navigation Stack (Nav2)** is a powerful and flexible framework for enabling mobile robots to do exactly this. Nav2 is the successor to ROS 1's `navigation` stack, rebuilt for the DDS-centric architecture of ROS 2.

Nav2 is not a single node but a collection of interconnected ROS 2 nodes that together provide a complete navigation solution. Its modular design allows developers to swap out different algorithms for mapping, localization, path planning, and control based on their robot's capabilities and environment.

### Key Components of Nav2:

*   **Planner Server**: Responsible for generating a global path from the robot's current location to a desired goal, considering the map and static obstacles.
*   **Controller Server**: Responsible for generating local trajectories and sending velocity commands to the robot's base to follow the global path and avoid dynamic obstacles.
*   **Behavior Tree**: A flexible task orchestration engine that manages the sequence of navigation behaviors (e.g., `spin`, `backup`, `wait`, `follow_path`).
*   **Costmap 2D**: A crucial component that maintains two grids representing the environment: a static map (from SLAM) and a dynamic map (for current obstacles detected by sensors). Cells in the costmap are assigned values representing their traversability.

Nav2 is highly configurable and can be adapted to various robot platforms, from wheeled robots to humanoid-like mobile manipulators that need to navigate complex indoor environments.

## 10.3 Mapping: Occupancy Grid Generation

**Mapping** in the context of Nav2 (and SLAM generally) often involves creating an **occupancy grid map**. An occupancy grid is a 2D or 3D grid-based representation of an environment, where each cell stores a probability of being occupied by an obstacle.

### How Occupancy Grids are Built:

1.  **Sensor Data Input**: Data from range sensors (LiDAR, depth cameras) provides information about obstacles.
2.  **Inverse Sensor Model**: For each sensor reading, an inverse sensor model updates the occupancy probability of cells. For example, if a laser ray hits an object, the cell where it hit is likely occupied, while cells along the ray are likely free.
3.  **Bayesian Update**: Probabilities are updated using Bayes' theorem, combining prior knowledge with new sensor measurements.
4.  **Grid Representation**: The map is typically stored as a grid of discrete cells, each with an occupancy probability (e.g., 0 for free, 1 for occupied, 0.5 for unknown).

The `slam_toolbox` ROS 2 package is commonly used for generating occupancy grid maps in real-time. It can perform both mapping and localization simultaneously, providing the necessary `map` topic for Nav2.

## 10.4 Localization: AMCL and Particle Filters

Once a map exists, the robot needs to determine its precise position and orientation within that map. This is the problem of **localization**. Nav2 primarily uses **Adaptive Monte Carlo Localization (AMCL)** for 2D localization.

### 10.4.1 Adaptive Monte Carlo Localization (AMCL)

AMCL is a probabilistic localization algorithm that uses a **particle filter** to estimate the robot's pose.
*   **Particles**: A particle filter represents the robot's pose by a set of weighted samples (particles) spread across the map. Each particle represents a possible pose of the robot.
*   **Prediction Step**: When the robot moves (based on odometry data from wheel encoders or IMU), the particles are moved according to the robot's motion model, and noise is added.
*   **Update Step**: When new sensor data (e.g., LiDAR scan) is received, each particle is evaluated based on how well the sensor data matches the map from that particle's pose. Particles that match the map well are given higher weights.
*   **Resampling**: Particles are resampled based on their weights, effectively "killing off" low-likelihood particles and duplicating high-likelihood ones. This causes the particles to converge around the true robot pose.
*   **Adaptive**: AMCL's "adaptive" nature means it can dynamically adjust the number of particles based on the uncertainty, making it more computationally efficient.

AMCL is robust to kidnapped robot problems (where the robot suddenly finds itself at a completely different location on the map) and partial sensor occlusions.

## 10.5 Path Planning and Motion Control for Humanoids

While Nav2 is typically optimized for ground-based mobile robots, its principles of global and local planning can be adapted for the unique challenges of humanoid navigation. Path planning for humanoids involves not just obstacle avoidance but also maintaining balance, considering dynamic stability, and generating complex whole-body motions.

### 10.5.1 Global Path Planning

The **global planner** (e.g., `NavFn`, `SmacPlanner` in Nav2) finds a collision-free path from the robot's start to goal on the static map. For humanoids, this global path might be an abstract sequence of footsteps or body postures rather than a continuous trajectory.

### 10.5.2 Local Path Planning and Motion Control

The **local planner/controller** (e.g., `DWBLocalPlanner`, `TEBLocalPlanner`) is responsible for generating short-term, dynamically feasible trajectories and velocity commands to follow the global path while avoiding dynamic obstacles and maintaining the humanoid's stability. For humanoids, this is significantly more complex:

*   **Whole-Body Control**: Instead of just commanding wheel velocities, a humanoid controller must manage the velocities and torques of many joints to maintain balance and achieve desired movements (walking, stepping, reaching).
*   **Zero Moment Point (ZMP)**: A common concept in humanoid locomotion control to ensure dynamic stability.
*   **Footstep Planning**: Planning the sequence and placement of footsteps to navigate terrain.
*   **Collision Avoidance**: Avoiding self-collision and external obstacles with the entire body, not just a simple base.

Integrating humanoid-specific locomotion controllers (e.g., from `humanoid_walk` packages) with Nav2 would involve providing appropriate interfaces for velocity commands or footstep plans. Advanced motion planning frameworks like MoveIt 2 can integrate with Nav2 to plan whole-body motions for humanoid manipulators.

## 10.6 Summary & Exercises

Chapter 10 delved into the critical aspects of autonomous robot navigation, starting with Visual SLAM (VSLAM) as the cornerstone for simultaneous mapping and localization using visual sensors. We then introduced the ROS 2 Navigation Stack (Nav2), outlining its modular architecture and key components like the Planner Server, Controller Server, and Costmap 2D. The chapter further explored mapping through occupancy grid generation and robust localization using Adaptive Monte Carlo Localization (AMCL) and particle filters. Finally, we discussed the complexities of adapting path planning and motion control within Nav2 for humanoids, emphasizing whole-body control, ZMP, and footstep planning as crucial considerations for bipedal locomotion.

### Exercises

1.  Explain the "chicken and egg" problem that SLAM aims to solve. What are the key steps in a VSLAM pipeline?
2.  List and briefly describe three core components of the ROS 2 Navigation Stack (Nav2).
3.  How does an occupancy grid map represent an environment, and why is it useful for robot navigation?
4.  Describe the role of particles in Adaptive Monte Carlo Localization (AMCL) and how they are used to estimate a robot's pose.
5.  What are the fundamental differences in path planning and motion control challenges between a wheeled mobile robot and a humanoid robot?
