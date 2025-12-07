# Chapter 5: ROS 2 Nodes, Topics, Services, Actions

## 5.1 ROS 2 Nodes: Modular Computation Units

In ROS 2, a **node** is the fundamental unit of computation. Each node is an executable program designed to perform a specific, modular task within the larger robotic system. By breaking down complex robot functionalities into smaller, independent nodes, ROS 2 promotes modularity, reusability, and fault tolerance. For instance, a robot's navigation system might consist of separate nodes for localization, path planning, and motor control.

Nodes can be written in various programming languages, with official client libraries available for C++ (`rclcpp`) and Python (`rclpy`). This language independence, facilitated by the underlying DDS (Data Distribution Service) middleware, allows developers to choose the best tool for each specific task without being constrained by a single language ecosystem.

### Key Characteristics of ROS 2 Nodes:

*   **Modularity**: Each node has a single responsibility.
*   **Decentralization**: Nodes communicate directly via DDS, eliminating a central master.
*   **Concurrency**: Multiple nodes can run concurrently on a single machine or distributed across multiple devices.
*   **Lifecycle Management**: Nodes can have defined states (e.g., Unconfigured, Inactive, Active) and transitions, allowing for robust system startup and shutdown.

```python
# Example: Basic ROS 2 Node in Python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node') # Initialize node with a unique name
        self.get_logger().info("My Custom Node has been started!")

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node) # Keep node alive until Ctrl+C is pressed
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*To run this node, save it as `my_custom_node.py` in your package, update `setup.py`, build, and then `ros2 run <your_package_name> my_custom_node`.*

## 5.2 Topics: Asynchronous Data Streaming (Publishers & Subscribers)

**Topics** are the most common mechanism for data exchange in ROS 2, enabling asynchronous, one-way message passing between nodes. This publish/subscribe pattern is highly flexible and promotes loose coupling between components.

### 5.2.1 Publishers

A **publisher** node sends messages to a specific named topic. Any node interested in that data can subscribe to the topic. Publishers define the type of message they will send (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`).

### 5.2.2 Subscribers

A **subscriber** node receives messages from a specific named topic. When a message is published to the topic, all subscribed nodes receive a copy of that message. Subscribers specify a callback function that is executed whenever a new message arrives.

### Quality of Service (QoS) Settings

ROS 2 topics leverage DDS's Quality of Service (QoS) profiles to configure communication behavior. QoS settings control aspects like:

*   **Reliability**: `reliable` (guaranteed delivery) vs. `best_effort` (faster, but messages might be dropped).
*   **Durability**: `transient_local` (new subscribers get the last published message) vs. `volatile` (only new messages received).
*   **History**: `keep_last` (only store a certain number of messages) vs. `keep_all` (store all messages up to resource limits).
*   **Liveliness**: How long a publisher can be inactive before its subscribers are notified.

These settings are crucial for tailoring communication to specific application needs (e.g., fast sensor data vs. critical command messages).

```python
# Example: Python Publisher and Subscriber using rclpy

# Publisher Node (part of chapter 4 example, refined)
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Example QoS: faster, allow drops
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main_publisher(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_publisher()

# Subscriber Node
# subscriber_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Must match publisher's QoS
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main_subscriber(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_subscriber()
```
*To run these, save them in your ROS 2 Python package (e.g., `my_package/my_package/publisher_node.py` and `my_package/my_package/subscriber_node.py`), update `setup.py` with entry points, build your package, source your workspace, then run `ros2 run my_package minimal_publisher` in one terminal and `ros2 run my_package minimal_subscriber` in another.*

## 5.3 Services: Synchronous Request/Reply Communication

**Services** provide a synchronous communication pattern where a client node sends a request message to a server node and waits (blocks) until it receives a response. This is ideal for requests that require an immediate, one-time answer.

### 5.3.1 Service Definition

Services are defined using `.srv` files, which specify both the request and response message types.
```
# Request part
int64 a
int64 b
---
# Response part
int64 sum
```

### 5.3.2 Service Server

A **service server** node implements the logic to handle incoming service requests and send back a response.

### 5.3.3 Service Client

A **service client** node sends a request to a service server and waits for the response.

```python
# Example: ROS 2 Python Service Server and Client

# my_service.srv (in your package's 'srv' directory)
# int64 a
# int64 b
# ---
# int64 sum

# service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your_package_name.srv.MyService

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server for add_two_ints is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sum = {response.sum}')
        return response

def main_server(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_server()

# service_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your_package_name.srv.MyService

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main_client(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(41, 1) # Example values
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_client()
```
*To run this, you would first need to create a ROS 2 package that defines the `AddTwoInts.srv` (or your custom service) and correctly configures `CMakeLists.txt` and `package.xml` to build the service interface. Then run the server and client nodes.*

## 5.4 Actions: Long-Running Goal-Oriented Tasks

**Actions** are designed for long-running, goal-oriented tasks that may take a significant amount of time to complete. They provide a full duplex (two-way) communication channel allowing clients to send a goal, receive periodic feedback on the progress, and ultimately get a result upon completion. Clients can also preempt or cancel an action.

### 5.4.1 Action Definition

Actions are defined using `.action` files, specifying a goal, feedback, and result structure.
```
# Goal
int64 order
---
# Result
int64[] sequence
---
# Feedback
int64[] partial_sequence
```

### 5.4.2 Action Server

An **action server** node executes the long-running task and sends feedback on its progress.

### 5.4.3 Action Client

An **action client** node sends goals to an action server, receives feedback, and processes the final result.

```python
# Example: ROS 2 Python Action Server and Client (Conceptual)

# fibonacci.action (in your package's 'action' directory)
# int64 order
# ---
# int64[] sequence
# ---
# int64[] partial_sequence

# action_server.py (Conceptual, requires full setup for action interface)
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci # Replace with your_package_name.action.Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.set_canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            time.sleep(1) # Simulate work

        goal_handle.set_succeeded()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f'Goal succeeded. Result: {result.sequence}')
        return result

def main_server(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_server()
```

## 5.5 Parameter Server and Dynamic Parameters

ROS 2 provides a **Parameter Server** which allows nodes to expose configurable parameters that can be set and modified at runtime, without recompiling the code. This is extremely useful for tuning algorithms, adjusting sensor thresholds, or changing robot behaviors on the fly.

### Key Features:

*   **Runtime Reconfiguration**: Parameters can be updated while a node is running.
*   **Centralized Access**: Parameters are managed by the ROS 2 system, allowing other nodes or command-line tools to read and set them.
*   **Type Safety**: Parameters have defined types (e.g., integer, float, string, boolean).

### Using Parameters in Python (`rclpy`)

```python
# Example: ROS 2 Node with Parameters
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        
        # Declare a parameter with a default value
        self.declare_parameter('my_parameter', 'default_value', 
                               descriptor=ParameterDescriptor(description='This is a custom parameter.'))
        
        # Get the parameter value
        param_value = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'Initial parameter value: {param_value}')

        # Create a timer to periodically check parameter changes
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Check if parameter has been updated
        new_param_value = self.get_parameter('my_parameter').get_parameter_value().string_value
        if new_param_value != self.get_parameter('my_parameter').get_parameter_value().string_value:
            self.get_logger().info(f'Parameter updated to: {new_param_value}')


def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*To change the parameter from the command line while the node is running:*
```bash
ros2 param set /param_node my_parameter "new_value"
ros2 param get /param_node my_parameter
```

## 5.6 Summary & Exercises

Chapter 5 delved deeper into the core communication mechanisms of ROS 2, which are essential for building distributed robotic systems. We started by solidifying the concept of ROS 2 nodes as modular computation units. Then, we thoroughly explored topics, the asynchronous backbone for data streaming, emphasizing publishers, subscribers, and the crucial role of QoS settings. Services provided a mechanism for synchronous request/reply interactions, suitable for immediate command-response scenarios. For long-running, goal-oriented tasks, actions were introduced as a robust, feedback-rich communication pattern. Finally, we examined the Parameter Server, a powerful feature allowing dynamic reconfiguration of node behaviors at runtime. Mastering these concepts is fundamental to developing complex and flexible ROS 2 applications.

### Exercises

1.  Create two Python nodes: one publisher (`talker`) that sends `std_msgs/msg/String` messages to a topic named `/my_chat`, and one subscriber (`listener`) that receives and prints these messages. Ensure they use `ReliabilityPolicy.RELIABLE` QoS.
2.  Define a custom ROS 2 service (`TwoFloatsSum.srv`) that takes two float numbers as a request and returns their sum. Implement a Python service server and client for this service.
3.  Discuss a scenario in humanoid robotics where an action would be more appropriate than a service, and explain why.
4.  Implement a ROS 2 Python node that declares a parameter for a robot's maximum linear speed. Set a default value and allow it to be updated at runtime. Verify the update using `ros2 param set`.
5.  Explain how QoS settings on a ROS 2 topic can impact the performance and reliability of a robotic system, providing an example.
