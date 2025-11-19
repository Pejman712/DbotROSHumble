Past_Projects
=====
Autonomous Navigation of Mobile Robots
------------
The robot is designed to transport heavy loads within CERN radiation facilities, where human presence is hazardous due to high radiation levels. Previously, robots were operated via teleoperation, but it posed several challenges, including operator fatigue, camera latency, and delayed control response. Continuous manual control made it difficult for operators to take breaks, leading to inefficiency and safety concerns. Introducing autonomous navigation allows the robot to operate independently during signal loss or operator absence, improving overall safety, efficiency, and enabling short breaks for human operators.

Robot Description
^^^^^^^^^^^^^^^

At CERN, radiation survey robots are tasked with monitoring and transporting equipment in restricted zones. Additionally, there is a separate robot platform called D-Bot, developed at Aalto University, which is used for research, development, and testing of autonomous functionalities. The D-Bot is a custom-built mobile robot designed to replicate key operational conditions in a controlled environment. It is equipped with all the essential sensors and control systems needed to simulate and validate navigation and mapping before they are deployed in the real world. The key components of the robot which are relevant for this project include:

- **LiDAR Sensor**: Used for real-time environment scanning and mapping.
- **Wheel Odometry**: Provides accurate movement tracking, essential for localization and navigation, especially in signal-degraded environments.

.. for images the use slash not backslash is important. look if the block is not complete for text you have to give 2 lines spaces to end the bloack.

.. Figure:: ./../../Images/D_Bot_Image.png
    :alt: D Bot image
    :width: 400px
    :align: center

    **Figure 1.**  Aalto D-Bot.


ROS and Its Libraries
----------------
The robot utilizes the Robot Operating System (ROS) for its software framework, which provides essential libraries and tools for building robot applications. ROS facilitates communication between different components of the robot, enabling efficient data exchange and control.

Key ROS libraries and packages used include:

- **rviz**: Visualization tool for monitoring robot state, sensor data, and navigation in real time.
- **rclpy**: Python API for ROS node development.
- **nav_msgs**: Standard message types for navigation.
- **gmapping**: SLAM (Simultaneous Localization and Mapping) for 2D occupancy grid creation using LiDAR and wheel odometry.

These libraries enable modular development, allowing each component (such as sensors, actuators, and navigation algorithms) to be developed and tested independently while ensuring seamless integration within the overall system.

Algorithms are first validated in Gazebo simulation before real-world deployment.

Algorithms and Their Logic
----------------

The autonomous navigation system employs a combination of algorithms for localization, mapping, and path planning. The key algorithms include:

- **A (A-star)**: Informed search algorithm using a heuristic to efficiently find the shortest path. Balances optimality and computational efficiency.
- **Dijkstras Algorithm**: Guarantees the shortest path but explores all nodes equally, leading to higher computation time.
- **RRT (Rapidly-exploring Random Tree)**: Sampling-based method suitable for complex spaces; finds feasible but often suboptimal paths and is computationally intensive.

Algorithm Comparison Table
^^^^^^^^^^^^^^^^^^^^^^^^^

All three algorithms were tested in a controlled simulated environment using Gazebo for different 2D occupancy maps which results in following parameters while navigation from Point A to Point B i.e. from start to end:

.. for table to remain centered, different code structure should be used.

+-------------+----------------------+------------------------+------------------+
| Algorithm   | Path Length (m)      | Computation Time (ms)  | Nodes Explored   |
+=============+======================+========================+==================+
| A*          | 23.54–27.88          | 24.18–493.01           | 341–461          |
+-------------+----------------------+------------------------+------------------+
| Dijkstra    | 21.92–23.75          | 306.67–1035.63         | 341–413          |
+-------------+----------------------+------------------------+------------------+
| RRT         | 22.46–35.17          | 3207.36–6275.46        | 23–32            |
+-------------+----------------------+------------------------+------------------+

A* offers the best balance for this application, with Dijkstra being slower and RRT less optimal for path quality which evident from the table above. All the maps and their path can be seen in the figures below.

.. Figure:: /Images/DF_Cage_Map.png
    :alt: D-Bot image
    :width: 400px
    :align: center

    **Figure 2.**  Aalto Design Factory Cage Map.

.. Figure:: /Images/CERN_Map_1.png
    :alt: D-Bot image
    :width: 400px
    :align: center

    **Figure 3.** CERN Facility Map 1.

.. Figure:: /Images/CERN_Map_2.png
    :alt: D-Bot image
    :width: 400px
    :align: center

    **Figure 4.** CERN Facility Map 2.
    
.. Figure:: /Images/CERN_Map_3.png
    :alt: D-Bot image
    :width: 400px
    :align: center

    **Figure 5.** CERN Facility Map 3.

Python API
-------

Classes
^^^^^^^

Each algorithm is implemented as a ROS node class:

- **AStarNode**: Handles path planning using the A* algorithm.
- **DijkstraNode**: Implements Dijkstra’s algorithm for shortest path computation.
- **RRTNode**: Executes the RRT algorithm for sampling-based path planning.

These classes inherit from the ROS node base class and interact with ROS topics and services for receiving map data, publishing planned paths, and responding to navigation requests.

Functions
^^^^^^^

.. no spaces between the headings and the text results in error in the documentation.

The following functions are defined within the ROS node classes to facilitate the autonomous navigation process:

- **plan_path(start, goal, map)**: Computes the path from start to goal.
- **update_map(sensor_data)**: Updates the occupancy grid using LiDAR and odometry.
- **publish_path(path)**: Publishes the computed path to a ROS topic.
- **handle_signal_loss()**: Switches to autonomous mode during communication loss.

Input/Output
^^^^^^^

Each path planning algorithm is implemented as a ROS node in Python. These nodes subscribe to sensor and goal data and publish computed paths to be followed by the robot. The table below summarizes the inputs and outputs of each node:

+------------------+--------------------------------------------+-------------------------------+
| **Node**         | **Subscribes To**                          | **Publishes To**              |
+==================+============================================+===============================+
| ``AStarNode``    | ``/map``, ``/initialpose``, ``/goal``      | ``/astar_path``               |
+------------------+--------------------------------------------+-------------------------------+
| ``DijkstraNode`` | ``/map``, ``/initialpose``, ``/goal``      | ``/dijkstra_path``            |
+------------------+--------------------------------------------+-------------------------------+
| ``RRTNode``      | ``/map``, ``/initialpose``, ``/goal``      | ``/rrt_path``                 |
+------------------+--------------------------------------------+-------------------------------+

Python Examples
-----------------

A* Algorithm
^^^^^^^^^^^^

.. code-block:: none

   function A_Star(startNode, goalNode, heuristicFunction)
      nodesToExplore = {startNode}
      bestPathMap = {}
      costFromStart = {startNode: 0}
      estimatedTotalCost = {startNode: heuristicFunction(startNode)}  // f = g + h

      while nodesToExplore is not empty:
         currentNode = node in nodesToExplore with lowest estimatedTotalCost value

         if currentNode == goalNode:
               return reconstruct_path(bestPathMap, currentNode)  // Path found!

         nodesToExplore.remove(currentNode)

         for each neighborNode of currentNode:
               pathCost = costFromStart[currentNode] + distance(currentNode, neighborNode)

               if pathCost < costFromStart.get(neighborNode, Infinity):
                  bestPathMap[neighborNode] = currentNode
                  costFromStart[neighborNode] = pathCost
                  estimatedTotalCost[neighborNode] = pathCost + heuristicFunction(neighborNode)

                  if neighborNode not in nodesToExplore:
                     nodesToExplore.add(neighborNode)


Dijkstra Algorithm
^^^^^^^^^^^^^^^^^^
.. ^^^^^ this was not completely unde the dijkstra section resulting in error and no further codes are displayed.
.. code-block:: none

   function Dijkstra(startNode, goalNode, distanceFunction)
      nodesToExplore = {startNode}         // Set of nodes to be evaluated
      bestPathMap = {}                     // Maps each node to its best previous node
      costFromStart = {startNode: 0}       // Tracks shortest distance from startNode

      while nodesToExplore is not empty:
         currentNode = node in nodesToExplore with lowest costFromStart value

         if currentNode == goalNode:
               return reconstruct_path(bestPathMap, currentNode)  // Path found!

         nodesToExplore.remove(currentNode)

         for each neighborNode of currentNode:
               pathCost = costFromStart[currentNode] + distanceFunction(currentNode, neighborNode)

               if pathCost < costFromStart.get(neighborNode, Infinity):
                  bestPathMap[neighborNode] = currentNode     // Update best path
                  costFromStart[neighborNode] = pathCost

                  if neighborNode not in nodesToExplore:
                     nodesToExplore.add(neighborNode)

      return failure  // No path found
   function reconstruct_path(bestPathMap, goalNode)
      shortestPath = [goalNode]
      while goalNode in bestPathMap:
         goalNode = bestPathMap[goalNode]
         shortestPath.prepend(goalNode)  // Add previous node to the path
      return shortestPath

RRT Algorithm
^^^^^^^^^^^^^

.. code-block:: none

   function RRT(startNode, goalNode, maxIterations, stepSize, obstacleChecker)
      tree = {startNode}         // Initialize tree with start node
      pathFound = false

      for i = 1 to maxIterations:
         randomNode = generateRandomNode()
         nearestNode = findNearestNode(tree, randomNode)
         newNode = extendTowards(nearestNode, randomNode, stepSize)

         if not obstacleChecker(newNode):
               continue  // Skip if node is invalid (collision)

         tree.add(newNode)

         if distance(newNode, goalNode) < stepSize:
               tree.add(goalNode)
               pathFound = true
               break

      if pathFound:
         return reconstruct_path(tree, goalNode)
      else:
         return failure  // No path found
   function reconstruct_path(tree, goalNode)
      path = [goalNode]
      currentNode = goalNode

      while currentNode in tree:
         currentNode = findParentNode(tree, currentNode)
         path.prepend(currentNode)
      return path


These examples illustrate the core logic of each algorithm, focusing on pathfinding and grid navigation. The actual implementation in the ROS nodes includes additional functionality for integration with the robot's sensors and actuators.
