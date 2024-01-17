ROS Humble
=====

.. _introduction:

Why ROS Humble?
------------
ROS (Robot Operating System) is an open-source robotic platform designed for academic and industrial applications. ROS 2, including the Humble Hawksbill version, enhances the strengths of ROS 1 in several ways:

    Improved Security Features: ROS 2 places a stronger emphasis on security, offering tools and features like secure node communication and access control. This is crucial for applications in sensitive or critical environments.

    Enhanced Real-Time Capabilities: ROS 2 is tailored for real-time computing, essential for robotics applications that demand precise timing and high reliability. This marks a significant advancement over ROS Noetic, which offers limited real-time support.

    Cross-Platform Compatibility: ROS 2 enhances compatibility across various operating systems, including Linux, macOS, and Windows. This broadens the scope for developing and deploying ROS applications in diverse settings.

    Quality of Service (QoS) Settings: ROS 2 introduces customizable QoS settings, allowing better control over node communication. This fine-tuning is vital for the reliability and efficiency of robotic systems.

    Modernized Underlying Architecture and Communication Layer: Utilizing DDS (Data Distribution Service) as its core middleware, ROS 2 ensures more robust and scalable communication compared to ROS Noetic. This modern architecture is better suited for complex and distributed robotic systems.

For existing ROS users, ROS 2 will feel familiar yet distinct enough that they can operate commands without needing to review new lines. Notably, ROS 2 eliminates the need for the roscore command to start the system; any ROS 2 command automatically initiates ROS. Like its predecessor, ROS 2 operates on a subscriber and publisher method. This project utilizes ROS Humble, which is supported until 2027. To install ROS Humble and for more information on its usage, you can refer to the ROS Humble documentation `here <https://docs.ros.org/en/humble/index.html>.. This page provides essential commands and cheat codes required to operate a robot with ROS 2. If you are not familiar with any ROS version, it is recommended to read the documentation. To further enhance your knowledge, consider reading "A Concise Introduction to Robot Programming with ROS 2."


ROS Humble Cheat Sheet
======================

This cheat sheet covers basic commands and operations in ROS Humble.

Basic Commands
--------------
.. code-block:: none

   Initialization: 
   ros2 run

   Starting a node: 
   ros2 run [package_name] [node_name]

   Listing nodes: 
   ros2 node list

Topic Management
----------------
.. code-block:: none

   Listing topics: 
   ros2 topic list

   Publishing to a topic: 
   ros2 topic pub [topic_name] [msg_type] [args]

   Subscribing to a topic: 
   ros2 topic echo [topic_name]

Service Management
------------------
.. code-block:: none

   Listing services: 
   ros2 service list

   Calling a service: 
   ros2 service call [service_name] [srv_type] [args]

Parameter Management
--------------------
.. code-block:: none

   Listing parameters: 
   ros2 param list

   Setting a parameter: 
   ros2 param set [node_name] [param_name] [value]

   Getting a parameter: 
   ros2 param get [node_name] [param_name]

Launch Files
------------
.. code-block:: none

   Running a launch file: 
   ros2 launch [package_name] [launch_file_name]

Debugging and Logging
---------------------
.. code-block:: none

   ROS2 logger levels: Debug, Info, Warn, Error, Fatal
   Setting logger level: 
   ros2 logging set_logger_level [logger_name] [level]

Building and Compiling
----------------------
.. code-block:: none

   Building a workspace: 
   colcon build

   Sourcing the environment: 
   source install/setup.bash

Key ROS Tools
-------------
* Rviz for visualization
* Gazebo for simulation

Best Practices
--------------
* Regular backups of code
* Use of version control (e.g., Git)

Useful Resources
----------------
* Official ROS documentation
* Community forums and Q&A sites

