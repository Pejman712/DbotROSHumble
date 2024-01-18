Motors
======
The wheel control is divided into three sections: Motor Controller (ZLTECH ZLA8015D), Hub Motors (ZLLG80ASM250 V3.0), and CAN Adapter (Peak-System PCAN-USB). OpenCAN is used to facilitate communication between the computer and the robot. "EXPLAIN CANOPEN, PDO AND SDO COMMUNICATION IN ONE PARAGRAPH". In the motor driver node, there are six files responsible for creating a connection between the computer and robot, each briefly explained as follows:

- ROS2wrapper.py: This file creates publishers (wheel velocity values) and subscribers (cmd_val) in ROS2.
- zlac80115d_canopen.py: A class that defines functions to control motors, such as setting target speed, halt, reading encoder speed, etc.
- od_definitions.py: An object dictionary for factors and units.
- Node_control.py: Conversion functions for a differential drive robot.
- log_connector.py: Connects Python logs to ROS2 logs.
- zlac8015d.eds: An electronic data sheet that zlac80115d_canopen.py reads for creating functions.

ROS2 Motor Topics
---------------

.. note::

    The follwing command is implemented in system boot up so you do not need to add it:

.. code-block:: console

    $ sudo ip link set can0 up type can bitrate 500000

You can disable this by removing the "can0-setup.service" at /etc/systemd/system/can0-setup.service

You can now initiate the motor driver node by running:

.. code-block:: console

    $ ros2 run motor_driver motor_bringup

This node creates 1 subscriber, cmd_vel, receiving twist messages, and 2 publishers, left_velocity_rpm and right_velocity_rpm, as float64 messages in rpm.

You can now publish your target speed to cmd_val, and the motor will follow. In a new terminal, you can test it with the following command:

.. code-block:: console

    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
      x: 5.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

The dbot ROS humble package is also equipped with an odometry estimator based on wheel speed. You can run this node:

.. code-block:: console

    $ ros2 run odometry_estimator odometry_estimator

This node publishes the odom topic, where you can access x and y direction pose. A sample code to read positions is provided in the API section.

Joystick Control
---------------

The Dbot wheel can be controlled with a joystick. First, ensure the joystick is connected to the Dbot by testing the output of:

.. code-block:: console

    $ sudo jstest /dev/input/js0

If you have received any feedback, run the next command, which creates joy messages in ROS from Linux input:

.. code-block:: console

    $ ros2 run joy_linux joy_linux_node

In a new terminal, run the joycontrol node with:

.. code-block:: console

    $ ros2 run joycontrol joycontrol

This node reads the joy messages and publishes cmd_vel. You can now drive around with the Dbot!
