Velodyne VLP16
==============

The Velodyne VLP16 is connected to the computer via Ethernet, using a fixed IP address of 192.168.1.100. To access the LiDAR settings, enter 192.168.1.201 in your web browser. To initiate the Velodyne driver node, use the following command:

.. code-block:: console

    $ ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

This node publishes Velodyne packets and points. You can visualize these points using RVIZ in ROS2. To start RVIZ, execute:

.. code-block:: console

    $ rviz2 rviz2 -f velodyne 

Once RVIZ is running, navigate to the lower left corner and click on the ``"add"`` option. In the 'Add' window, go to the "by topics" tab and select 'point_clouds'. You will now be able to see the Velodyne points in real-time.

Configuring Velodyne IP
---------------

.. note::

   Do NOT change the IP address unless you are not receiving any points.

If the connection between the computer and the Velodyne is compromised, you can reset the IP address by trying the following commands:

.. code-block:: console

    $ sudo route add 192.168.1.100 enp89s0
    $ sudo route add 192.168.1.201 enp89s0
    $ sudo route add 192.168.1.255 enp89s0

