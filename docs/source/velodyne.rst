Velodyne VLP16
=====
Velodyne VLP16 is connected to the computer through ethernet with fix ip address of 192.168.1.100. you can access the LiDAR setting by typing 192.168.1.201 in your web browser. The velodyne driver node can be initiated by 

.. code-block:: console

    $ ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

This node publishes the velodyne packets and points. You can visualize the velodyne points by running rviz in ROS2. You can Initiate RVIZ by: 


.. code-block:: console

    $ rviz2 rivz2 -f velodyne 

Once the rviz program shows up, go to the button left and press the ``"add"`` options. In add window go to the "by topics " tab and select point_clouds. Now you can see the velodyne points in real time. 

Configuring Velodyne ip
=====
.. note::

   Do NOT Change ip address unless you are not reciving any points

If the connenction between the computer and velodyne is compormize you can reset the ip adress by trying follwing ip addresses 

.. code-block:: console

    $ sudo route add 192.168.1.100 enp89s0
    $ sudo route add 192.168.1.201 enp89s0
    $ sudo route add 192.168.1.255 enp89s0



