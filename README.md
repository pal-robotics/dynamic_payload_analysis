# Dynamic Payload Analysis and Visualization

This project aims to develop a ROS 2-based software tool that allows users to assess and visualize the useful payload a robot can handle, based on its kinematic and dynamic properties. The tool will process the robot’s URDF (Unified Robot Description Format), taking into account actuation constraints, it will provide intuitive visualizations of available payload in a specific configuration or a workspace representation of payload capabilities. Additionally, it will include functionality to extract joint requirements (torque, power, and stiffness) needed to achieve a desired payload, helping roboticists optimize robot design.

This tool will benefit roboticists, engineers, and researchers working on mobile robots, manipulators, and humanoid platforms. It will provide an intuitive way to evaluate, optimize, and redesign robots based on payload requirements, making it easier to select suitable actuators during the design phase.

## Outcomes of this package

  * A C++ or Python tool to compute and visualize payload capacity in a specific robot joint configuration or inside the robot’s workspace.
  * A joint requirement estimation module that suggests required actuator capabilities for achieving a specified payload.
  * Integration with ROS 2, URDF, and RViz
  * A custom RViz plugin to dynamically display payload constraints and actuator requirements.
  * Documentation and example cases for different robotic platforms (TIAGo Pro, TALOS and other available open source robot models).

## Documentation

You can find more documentation here : [Setup and Tutorials](doc/README.md)