ROS Humble
=====


Why ROS Humble?
------------
ROS is an open-source robotic platform designed for both academic and industrial applications. The latest iteration, ROS 2, builds upon the strengths of ROS 1 and introduces several significant improvements.

Improvements in ROS 2
---------------------

1. **Improved Security Features**
   ROS 2, particularly in its Humble Hawksbill version, emphasizes security. It offers tools for secure node communication and access control, essential for sensitive or critical applications.

2. **Enhanced Real-Time Capabilities**
   Tailored for real-time computing, ROS 2 is crucial for applications requiring precise timing and high reliability. This is a notable enhancement over ROS Noetic's limited real-time support.

3. **Cross-Platform Compatibility**
   ROS 2 improves compatibility across Linux, macOS, and Windows, facilitating development and deployment in varied environments.

4. **Quality of Service (QoS) Settings**
   With customizable QoS settings, ROS 2 allows for refined control over node communication, crucial for system reliability and efficiency.

5. **Modernized Architecture and Communication Layer**
   Using DDS (Data Distribution Service) for its middleware, ROS 2 provides robust, scalable communication, a significant upgrade over ROS Noetic.

ROS 2 for Existing ROS Users
----------------------------

For those familiar with ROS, ROS 2 will feel intuitive yet distinct enough to operate without constant reference to new documentation. One key change in ROS 2 is the elimination of the `roscore` command; the system starts automatically with any ROS 2 command. Like ROS, ROS 2 uses a subscriber and publisher method.

Utilizing ROS Humble
--------------------

This project uses ROS Humble, supported until 2027. For installation and usage instructions, refer to the `ROS Humble documentation <https://docs.ros.org/en/humble/index.html>`_. This guide provides essential commands and cheat codes for operating robots with ROS 2. If you are new to ROS, reviewing this documentation is recommended. For an in-depth understanding, consider reading "A Concise Introduction to Robot Programming with ROS 2."


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

