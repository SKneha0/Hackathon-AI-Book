# Chapter 4: ROS 2 Fundamentals

## 4.1 What is ROS 2? Architecture and Core Concepts

The Robot Operating System (ROS) has been the de facto standard for robot software development for over a decade. ROS 2 represents a significant evolution, redesigned to address the limitations of its predecessor (ROS 1) concerning real-time performance, security, multi-robot systems, and embedded platforms. ROS 2 is not an operating system in the traditional sense, but rather a flexible framework, a set of tools, libraries, and conventions that simplify the task of creating complex robot applications across diverse hardware platforms.

### 4.1.1 Key Architectural Differences from ROS 1

*   **DDS (Data Distribution Service) as Middleware**: A fundamental change in ROS 2 is the adoption of DDS for its communication layer. DDS is a publish/subscribe middleware standard that offers qualities of service (QoS) configurations for reliability, durability, and a discovery mechanism. This replaces ROS 1's custom TCP/IP-based communication (ROS Master), enabling:
    *   **Decentralization**: No single point of failure (ROS Master is gone).
    *   **Real-time Capabilities**: Enhanced determinism and performance.
    *   **Improved Security**: DDS provides built-in security features for authentication, access control, and encryption.
    *   **Multi-robot and Embedded Systems**: Better suited for heterogeneous and distributed environments.
*   **Client Libraries**: ROS 2 provides client libraries for C++ (`rclcpp`) and Python (`rclpy`), offering language-agnostic interfaces to the underlying DDS middleware.
*   **Launch System**: A more powerful and flexible launch system using Python scripts (`.launch.py`) for starting multiple nodes and processes.

### 4.1.2 Core Concepts of ROS 2

*   **Nodes**: Executable processes that perform computation (e.g., a node for reading camera data, a node for motor control). Each node should ideally be modular, performing a single logical task.
*   **Topics**: The primary mechanism for asynchronous, one-way message passing between nodes. Publishers send messages to a named topic, and subscribers receive messages from it.
*   **Services**: A synchronous request/reply mechanism for communication between nodes. A client node sends a request to a server node and waits for a response. Useful for operations requiring an immediate result.
*   **Actions**: A higher-level, asynchronous communication method for long-running, goal-oriented tasks (e.g., "navigate to a point"). Clients send a goal, receive periodic feedback, and eventually a result.
*   **Parameters**: Configuration values that can be set by a node and changed at runtime. Useful for tuning algorithms without recompiling code.
*   **Messages**: Data structures used by Topics, Services, and Actions for inter-node communication. Defined in `.msg` files.
*   **Interfaces**: Generic term for `.msg`, `.srv`, and `.action` files which define the data types used for communication.

<!-- ![ROS 2 Communication Diagram](images/ros2_communication_diagram.png)
*Figure 4.1: An overview of ROS 2 communication mechanisms including nodes, topics, services, and actions.* -->

## 4.2 ROS 2 Installation and Workspace Setup

Before diving into programming, you need a working ROS 2 installation. This textbook primarily assumes an Ubuntu Linux environment (e.g., Ubuntu 22.04 LTS "Jammy Jellyfish" for ROS 2 Humble Hawksbill, or Ubuntu 20.04 LTS "Focal Fossa" for ROS 2 Foxy Fitzroy). While ROS 2 supports Windows and macOS, the robotics ecosystem often thrives best on Linux.

### 4.2.1 Installation Steps (Ubuntu)

1.  **Set up Locale**: Ensure a UTF-8 locale.
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 Repository**:
    ```bash
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 (Desktop Install)**:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    sudo apt install ros-humble-desktop -y # Replace 'humble' with your desired ROS 2 distribution
    ```
4.  **Environment Setup**: Source the ROS 2 setup script to make ROS 2 commands available. Add it to your `.bashrc` for persistence.
    ```bash
    source /opt/ros/humble/setup.bash # Replace 'humble' with your ROS 2 distribution
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
5.  **Install `rosdep`**: Tool for installing system dependencies.
    ```bash
    sudo apt install python3-rosdep -y
    sudo rosdep init
    rosdep update
    ```

### 4.2.2 Creating a ROS 2 Workspace

A ROS 2 workspace is a directory where you develop your own ROS 2 packages. It's convention to name it `ros2_ws`.

1.  **Create Workspace Directory**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
2.  **Initialize Workspace (Optional but Good Practice)**:
    ```bash
    # This creates a hidden .rosinstall file if you want to track source repositories
    rosinstall_generator desktop --rosdistro humble --output-dir .
    ```
3.  **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
4.  **Source the Workspace**: After building, you must source the workspace's setup file.
    ```bash
    source install/setup.bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```
    *Important*: Always source the ROS 2 distribution first (`/opt/ros/humble/setup.bash`), then your workspace (`~/ros2_ws/install/setup.bash`).

## 4.3 Command Line Tools: `ros2 run`, `ros2 topic`, `ros2 node`

ROS 2 provides a rich set of command-line tools (CLIs) for interacting with the running system, inspecting topics, debugging nodes, and more.

### 4.3.1 `ros2 run`

Used to launch an executable from a ROS 2 package.
```bash
ros2 run <package_name> <executable_name>
```
*Example*: Launching a "talker" (publisher) node from the `demo_nodes_cpp` package.
```bash
ros2 run demo_nodes_cpp talker
```

### 4.3.2 `ros2 node`

Provides commands to inspect ROS 2 nodes.
*   **`ros2 node list`**: Lists all active nodes.
*   **`ros2 node info <node_name>`**: Displays information about a specific node (topics, services, actions, parameters).

*Example*:
```bash
ros2 node list
# Output:
# /talker
# /listener
ros2 node info /talker
```

### 4.3.3 `ros2 topic`

Provides commands to inspect and interact with ROS 2 topics.
*   **`ros2 topic list`**: Lists all active topics.
*   **`ros2 topic echo <topic_name>`**: Displays messages being published on a topic.
*   **`ros2 topic info <topic_name>`**: Displays information about a topic (message type, publishers, subscribers).
*   **`ros2 topic pub <topic_name> <msg_type> <arguments>`**: Publishes a single message to a topic from the command line.

*Example*:
```bash
ros2 topic list
# Output:
# /parameter_events
# /rosout
# /chatter
ros2 topic echo /chatter
ros2 topic info /chatter
ros2 topic pub /chatter std_msgs/msg/String '{data: "Hello from CLI"}'
```

## 4.4 Understanding ROS 2 Packages and Build System (Colcon)

### 4.4.1 ROS 2 Packages

A **ROS 2 package** is the fundamental unit for organizing ROS 2 code. A package can contain ROS 2 nodes, libraries, configuration files, message definitions, and other related resources.
Every package must have a `package.xml` file, which describes its metadata (name, version, description, maintainers, dependencies), and a build system configuration (e.g., `CMakeLists.txt` for C++ packages, `setup.py` for Python packages).

### 4.4.2 Colcon Build System

**Colcon** is the build system used in ROS 2. It's a command-line tool that builds a set of packages from source. It's designed to be flexible and efficient, allowing for parallel builds and incremental compilation.

*   **`colcon build`**: Builds all packages in the current workspace.
*   **`colcon build --packages-select <package_name>`**: Builds only a specific package.
*   **`colcon build --symlink-install`**: Creates symbolic links to executables and Python files in the install space, useful for development as changes don't require re-installation.

## 4.5 Creating Your First ROS 2 Package

Let's create a simple Python package with a "hello world" publisher node.

1.  **Navigate to your workspace `src` directory**:
    ```bash
    cd ~/ros2_ws/src
    ```
2.  **Create a new Python package**:
    ```bash
    ros2 pkg create --build-type ament_python my_python_pkg
    ```
    This creates a directory `my_python_pkg` with basic `package.xml` and `setup.py` files.
3.  **Create the Python Node File**:
    Inside `~/ros2_ws/src/my_python_pkg/my_python_pkg/`, create a file `simple_publisher.py` (you might need to create the inner `my_python_pkg` directory first, or it may be created by `ros2 pkg create`).

    ```python
    # ~/ros2_ws/src/my_python_pkg/my_python_pkg/simple_publisher.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    def main(args=None):
        rclpy.init(args=args)
        node = Node("simple_publisher") # Name your node
        publisher = node.create_publisher(String, "my_topic", 10)
        msg = String()
        i = 0
        while rclpy.ok():
            msg.data = f"Hello ROS 2 from Python: {i}"
            publisher.publish(msg)
            node.get_logger().info(f'Publishing: "{msg.data}"')
            i += 1
            rclpy.spin_once(node, timeout_sec=0.5) # Process callbacks once
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
4.  **Update `setup.py`**:
    Edit `~/ros2_ws/src/my_python_pkg/setup.py` to include your new executable entry point.

    ```python
    # ... (existing content) ...
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
        ],
    },
    # ... (rest of the file) ...
    ```
5.  **Build and Source Your Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_python_pkg
    source install/setup.bash # Re-source your workspace after building
    ```
6.  **Run Your Node**:
    ```bash
    ros2 run my_python_pkg simple_publisher
    ```
    You can then open another terminal, source ROS 2 and your workspace, and echo the topic:
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 topic echo /my_topic
    ```

## 4.6 Summary & Exercises

This chapter provided a foundational understanding of ROS 2, beginning with its architecture and core concepts like nodes, topics, services, and actions. We covered the essential steps for installing ROS 2 on an Ubuntu system and setting up a development workspace. The utility of ROS 2 command-line tools for inspecting and interacting with a running system was demonstrated. Furthermore, the chapter explained the role of ROS 2 packages and the `colcon` build system, culminating in a practical guide to creating your very first ROS 2 Python package and node. This fundamental knowledge is critical for building more complex robot applications.

### Exercises

1.  Explain the key reasons for the transition from ROS 1 to ROS 2, highlighting the advantages of DDS middleware.
2.  Describe the purpose of `ros2 run`, `ros2 topic`, and `ros2 node` command-line tools, providing an example usage for each.
3.  Outline the typical steps involved in setting up a ROS 2 development workspace.
4.  Create a ROS 2 Python package that contains a subscriber node. This node should subscribe to the `/my_topic` published by the `simple_publisher` node created in this chapter and print the received messages.
5.  Discuss how ROS 2 packages and the `colcon` build system contribute to modular and manageable robot software development.
