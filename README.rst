.. figure:: https://github.com/ouster-lidar/ouster_example/raw/master/docs/images/Ouster_Logo_TM_Horiz_Black_RGB_600px.png

------------------------------------------------------

==========================================================
Ouster SDK - libraries and tools for Ouster Lidar Sensors
==========================================================

Cross-platform C++/Python Ouster Sensor Development Toolkit

To get started with our sensors, client, and visualizer, please see our SDK and sensor documentation:

- `Ouster SDK Documentation <https://static.ouster.dev/sdk-docs/index.html>`_
- `Ouster Sensor Documentaion <https://static.ouster.dev/sensor-docs>`_ 

This repository contains Ouster SDK source code for connecting to and configuring ouster sensors,
reading and visualizing data, and interfacing with ROS.

* `ouster_client <ouster_client/>`_ contains an example C++ client for ouster sensors
* `ouster_pcap <ouster_pcap/>`_ contains C++ pcap functions for ouster sensors
* `ouster_viz <ouster_viz/>`_ contains a customizable point cloud visualizer
* `ouster_ros <ouster_ros/>`_ contains example ROS nodes for publishing point cloud messages
* `python <python/>`_ contains the code for the ouster sensor python SDK (``ouster-sdk`` Python package)

FOV Shrinker Feature
====================

Purpose
-------

The FOV (Field of View) Shrinker is an integrated beam filtering feature in the ROS point cloud node that allows users to reduce the vertical field of view by selecting only specific horizontal beam ranges from the LiDAR sensor. This feature is particularly useful for:

* **Performance optimization**: Reducing computational load by processing fewer points
* **Focus on region of interest**: Concentrating on specific vertical sections (e.g., road level, horizon)
* **Bandwidth reduction**: Decreasing network traffic in distributed systems

Usage
-----

The FOV shrinker can be controlled through ROS parameters:

**Enable/Disable Filtering:**

.. code-block:: xml

    <!-- In your launch file -->
    <param name="/os_cloud_node/enable_FOV_filter" value="true"/>

**Configure Beam Range:**

.. code-block:: xml

    <!-- Keep beams 16 to 48 (middle section of a 64-beam LiDAR) -->
    <param name="/os_cloud_node/start_beam" value="16"/>
    <param name="/os_cloud_node/end_beam" value="48"/>
    
    <!-- Enable debug output for performance monitoring -->
    <param name="/os_cloud_node/debug_mode" value="false"/>

**Command Line Usage:**

.. code-block:: bash

    # Launch with filtering enabled (default: middle half beams)
    roslaunch ouster_ros ouster.launch enable_FOV_filter:=true
    
    # Launch with custom beam range
    roslaunch ouster_ros ouster.launch enable_FOV_filter:=true start_beam:=10 end_beam:=40
    
    # Launch with debug output enabled
    roslaunch ouster_ros ouster.launch enable_FOV_filter:=true debug_mode:=true

**Default Behavior:**

* **Filter disabled by default**: Full point cloud is published when no parameters are set
* **Middle half beams**: When enabled without specifying ranges, keeps beams from 25% to 75% of total beam count
* **Automatic beam calculation**: Default start/end beams are calculated based on sensor configuration


The filtered point cloud maintains all original metadata and is published to the same topics (/os_cloud_node/points) as the full point cloud 


License
========

BSD 3-Clause License, `details <LICENSE>`_
