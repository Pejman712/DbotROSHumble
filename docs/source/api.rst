API
===
This section will provide 3 sample codes for your projects namely, simple publsiher, simpler listener, odometry position print.

Hello Dbot Publisher
---------------

The Hello Dbot publisher is a simple ROS2 node that publishes "Hello, Dbot" messages. This document provides an overview of its functionality and instructions for use.


Ensure ROS2 is installed on your system. If not, follow the installation instructions on the ROS2 website.

Setup Workspace
---------------

Create a new ROS2 workspace and a package for the publisher:

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_cmake --node-name hello_dbot_publisher hello_dbot

Publisher Node
--------------

The publisher node is written in Python and is responsible for publishing "Hello, Dbot" messages.

.. code-block:: python

   # hello_dbot_publisher.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class HelloDbotPublisher(Node):
       def __init__(self):
           super().__init__('hello_dbot_publisher')
           self.publisher_ = self.create_publisher(String, 'hello_dbot', 10)
           timer_period = 1  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello, Dbot'
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)

   def main(args=None):
       rclpy.init(args=args)
       hello_dbot_publisher = HelloDbotPublisher()
       rclpy.spin(hello_dbot_publisher)
       hello_dbot_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Build and Run
-------------

To build and run the publisher, use the following commands:

.. code-block:: bash

   # Build the package
   cd ~/ros2_ws
   colcon build --packages-select hello_dbot

   # Source the setup script
   source ~/ros2_ws/install/setup.bash

   # Run the Publisher
   ros2 run hello_dbot hello_dbot_publisher

The publisher will start and publish "Hello, Dbot" messages at a 1-second interval.

Verify
------

To verify the publisher is working, listen to the topic in another terminal:

.. code-block:: bash

   ros2 topic echo /hello_dbot

You should see "Hello, Dbot" messages being printed at regular intervals.


Hello Dbot Subscriber
---------------

The Hello Dbot subscriber is a ROS2 node that subscribes to messages on the "hello_dbot" topic. It prints out each "Hello, Dbot" message it receives.

Subscriber Node
---------------

The subscriber node is written in Python. It listens to the `hello_dbot` topic and logs each message received.

.. code-block:: python

   # hello_dbot_subscriber.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class HelloDbotSubscriber(Node):

       def __init__(self):
           super().__init__('hello_dbot_subscriber')
           self.subscription = self.create_subscription(
               String,
               'hello_dbot',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info('Received: "%s"' % msg.data)

   def main(args=None):
       rclpy.init(args=args)
       hello_dbot_subscriber = HelloDbotSubscriber()
       rclpy.spin(hello_dbot_subscriber)
       hello_dbot_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Usage
-----

To use this subscriber node:

1. Ensure the ROS2 environment is sourced.

2. Run the subscriber node with:

   .. code-block:: bash

      ros2 run [package_name] hello_dbot_subscriber

   Replace `[package_name]` with the name of your ROS2 package.

3. The subscriber will start and print out "Hello, Dbot" messages as they are received from the publisher.

This node can be used in conjunction with the Hello Dbot publisher to demonstrate basic ROS2 pub/sub functionality.

Odom Position Subscriber
---------------

The Odom Position Subscriber is a ROS2 node that subscribes to the `odom` (odometry) topic and prints the x and y positions. This is typically used in robotics to track the position of a robot.

Subscriber Node
---------------

The subscriber node is written in Python. It listens to the `odom` topic, which is of the type `nav_msgs/msg/Odometry`, and logs the x and y position coordinates.

.. code-block:: python

   # odom_position_subscriber.py
   import rclpy
   from rclpy.node import Node
   from nav_msgs.msg import Odometry

   class OdomPositionSubscriber(Node):

       def __init__(self):
           super().__init__('odom_position_subscriber')
           self.subscription = self.create_subscription(
               Odometry,
               'odom',
               self.odom_callback,
               10)
           self.subscription  # prevent unused variable warning

       def odom_callback(self, msg):
           position = msg.pose.pose.position
           self.get_logger().info(f'Position: x={position.x}, y={position.y}')

   def main(args=None):
       rclpy.init(args=args)
       odom_position_subscriber = OdomPositionSubscriber()
       rclpy.spin(odom_position_subscriber)
       odom_position_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Usage
-----

To use this subscriber node:

1. Ensure the ROS2 environment is sourced.

2. Place the script in the `src` directory of your ROS2 package.

3. Build the package using `colcon build`.

4. Run the subscriber node with:

   .. code-block:: bash

      ros2 run [package_name] odom_position_subscriber

   Replace `[package_name]` with the name of your ROS2 package.

5. The subscriber will start and print out the x and y positions as they are received from the `odom` topic.

This node is useful for tracking the real-time position of a robot in a 2D space, especially in a simulation or testing environment.


