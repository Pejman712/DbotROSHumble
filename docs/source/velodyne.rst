Velodyne VLP16
=====
Velodyne VLP16 is connected to the computer through ethernet with fix ip address of 192.168.1.100. you can access the LiDAR setting by typing 192.168.1.201 in your web browser. The velodyne driver node can be initiated by 

.. code-block:: console

    $ ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

This node publishes the velodyne packets and points. You can visualize the velodyne points by running rviz in ROS2. You can Initiate RVIZ by: 


.. code-block:: console

    $ rviz2 rivz2 -f velodyne 

Once the rviz program shows up, go to the button left and press the add options. In add window go to the "by topics " tab and select point_clouds. Now you can see the velodyne points in real time. 

Configuring Velodyne ip
=====



.. _installation:

Installation
------------

To use Lumache, first install it using pip:

.. code-block:: console

   (.venv) $ pip install lumache

Creating recipes
----------------

To retrieve a list of random ingredients,
you can use the ``lumache.get_random_ingredients()`` function:

.. autofunction:: lumache.get_random_ingredients

The ``kind`` parameter should be either ``"meat"``, ``"fish"``,
or ``"veggies"``. Otherwise, :py:func:`lumache.get_random_ingredients`
will raise an exception.

.. autoexception:: lumache.InvalidKindError

For example:

>>> import lumache
>>> lumache.get_random_ingredients()
['shells', 'gorgonzola', 'parsley']
