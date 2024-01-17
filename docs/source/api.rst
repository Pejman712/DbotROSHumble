API
===
This section will provide 4 sample codes for your projects namely, simple publsiher, simpler listener, odometry position print, and reading canopen 

Hello Dbot Publisher
====================

The Hello Dbot publisher is a simple ROS2 node that publishes "Hello, Dbot" messages. This document provides an overview of its functionality and instructions for use.

Installation
------------

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
