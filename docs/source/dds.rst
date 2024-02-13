ROS2 on Multiple machines!
===================================

It is possible to send/receive messages on multiple machines without configuring anything explicitly, thanks to the DDS in ROS2. We are using this feature to control DBot and collect the data from it using an "external PC". For more information visit `here <https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/>`. 

.. note::

   DBot has Ubuntu 22.04 ROS2 Humble. The "external PC" should also have ROS2 installed (preferably the same distribution-Ros-humble).
1.Both machines should be connected to same network.
2.Multicasting is enabled in the network. To check it, run the following command in DBot to get the IP address:
.. code-block:: console

    $ hostname -I
It will return the IP address, for example: 192.168.103.122. Then, from the "external PC" run the following command:
.. code-block:: console

    $ ping 192.168.103.122
If everything is okay, you should able to send data from the "external PC" to DBot.
3. Make sure all machines have same domain ID.
4. Install 'joy_linux' ros2 package in the external PC (if want to try joystick control)
.. code-block:: console

    $ sudo apt-get install ros-humble-joy-linux
in order to control the dbot using joystick connected to the dbot first you need to verify the correct joystick. This task can be accomplished by connecting the joystick to the laptop/external PC 
and running following command. 

.. code-block:: console

    $ sudo jstest /dev/input/js0
if the terminal shows two 8 index metricies, you can run 
.. code-block:: console

    $ ros2 run joy_linux joy_linux_node
publishing the joy messages in ros environment.
If the ID is not 'js0', run the following command, replace XXX with the joystick ID.
.. code-block:: console

    $ ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/XXX
For example, if the ID is js1, run the following command:
.. code-block:: console

    $ ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/js1
This will reset the parameter, and run the 'joy_linux' node. Now the DBot can be controlled from the external PC. Note that all the ROS functionalities are shared between these two machines now, and we have access to all topics, nodes, etc. Yet, there might be a small delay between the data sharing so be cautious if you're recording (velodyne) messages in external PC.
About ROS_DOMAIN_ID:
For ROS communication to happen between multiple machines, it is important to have common 'ROS_DOMAIN_ID' parameter. By default, the value is 0 and no need to set it explicitly. In case of more machines connected to same network, or if you want to restrict the communication, it is advised to set domain id. For this, run the following command in both DBot and external PC:
.. code-block:: console

  $ export ROS_DOMAIN_ID=XX
Replace XX with any number between 0 and 101. Use same ID for both DBot and external PC.
