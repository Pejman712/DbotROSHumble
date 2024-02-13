ROS2 on Multiple Machines
=========================

Utilizing the DDS (Data Distribution Service) in ROS2, it's feasible to send and receive messages across multiple machines without explicit configuration. This functionality is employed to control a robot, referred to as DBot, and to collect data from it using an external computer.

For detailed information, refer to [this guide](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/).

.. note::

   DBot operates on Ubuntu 22.04 with ROS2 Humble installed. The external computer should also have ROS2 installed, preferably the same distribution (ROS Humble).

Prerequisites
-------------

1. Ensure both machines are connected to the same network.
2. Verify that multicasting is enabled on the network. To check, run the following command on DBot to obtain its IP address:

   .. code-block:: console

       $ hostname -I

   This command will return an IP address, such as 192.168.103.122. Then, from the external computer, run:

   .. code-block:: console

       $ ping 192.168.103.122

   If the ping is successful, communication between the external computer and DBot is established.
   
3. Confirm that all machines share the same domain ID.

4. To enable joystick control from the external PC, install the 'joy_linux' ROS2 package:

   .. code-block:: console

       $ sudo apt-get install ros-humble-joy-linux

Configuring Joystick Control
----------------------------

1. Connect the joystick to the external PC and verify it by running:

   .. code-block:: console

       $ sudo jstest /dev/input/js0

   If the terminal displays two 8 index matrices, the joystick is correctly detected.

2. To publish joystick messages within the ROS environment, run:

   .. code-block:: console

       $ ros2 run joy_linux joy_linux_node

   If your joystick ID is not 'js0', replace XXX with your joystick ID and run:

   .. code-block:: console

       $ ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/XXX

   For instance, for ID 'js1', execute:

   .. code-block:: console

       $ ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/js1

   This action sets the parameter and initiates the 'joy_linux' node, allowing DBot to be controlled from the external PC.

.. note::

   With ROS functionalities shared between the two machines, access to all topics, nodes, etc., is available. However, be mindful of potential delays in data sharing, especially when recording messages (e.g., from a Velodyne sensor) on the external PC.

Setting the ROS_DOMAIN_ID
-------------------------

For ROS communication across multiple machines, a common 'ROS_DOMAIN_ID' parameter is essential. By default, this value is 0 and doesn't require explicit setting. To add more machines to the same network or to restrict communication, setting a domain ID is recommended:

.. code-block:: console

   $ export ROS_DOMAIN_ID=XX

Replace XX with any number between 0 and 101. Ensure the same ID is used for both DBot and the external PC.
