# Tiago Pro Tutorial
This tutorial provides a guide on how to use the dynamic payload analysis for the **Tiago Pro robot**. It covers the visualization of workspace areas, joint torques, and maximum payloads.

# Prerequisites
Ensure you have the Tiago Pro robot's URDF file and the necessary ROS packages installed using rosdep.
For Tiago pro, rosdep will not install robot_description package, so you need to install it manually:
```bash
git clone https://github.com/pal-robotics/tiago_pro_robot.git
```

and install other necessary packages.
You can find the necessary URDF files in the corrisponding repository: [Tiago Pro URDF](https://github.com/pal-robotics/tiago_robot)

# Setting Up the Environment
1. In the dynamic_payload_analysis_ros package, there is a launch file that allows you to visualize the dynamic payload analysis in RViz for the Tiago Pro robot. You can run the following command to launch the node:
    ```bash
    ros2 launch dynamic_payload_analysis_ros dyn_analysis_payload_tiago_pro.launch.py
    ```
    *If you want to modify the available parameters for the dynamic analysis, you can set them in the launch command. The available parameters are:*

    - advanced_mode: If set to true, allows adding a payload to any link in the kinematic tree of the selected end effector link. If false, you can only add a payload to the end effector link.
    - resolution_ik: Sets the resolution of the inverse kinematics computation. Default is 0.20 meters, but you can set it to a different value if needed.*(Remember: with lower values the computational time will increase a lot)*
    - workspace_range: Sets the range of the workspace area to be analyzed. Default is 2.0 meters, but you can set it to a different value if needed.
    
2. After launching Rviz, you'll be able to see a view similar to the one shown below:
   <div style="text-align: center;">
   <img src="images/tiago_pro_overview.png" alt="Overview" width="600"/>
   </div>

    **Note**: Make sure to have as fixed frame the **base_footprint** of the robot, so you can visualize the markers correctly.

# Analyzing the Workspace Area
1. Right-click on the interactive markers to open a menu where you can select which kinematic chains you want to analyze by selecting the corresponding end effector link in the sub-menu.
    For tiago pro, the main kinematic chains are:
    1. Right-click on the interactive markers to open a menu where you can select which kinematic chains you want to analyze by selecting the corresponding end effector link in the sub-menu.
    For tiago pro, the main kinematic chains are:
    - **arm_left**: Left arm kinematic chain.
    - **arm_right**: Right arm kinematic chain.
    - **head**: Head kinematic chain.
        ```mermaid
        graph TD;
            A[torso_lift_link] --> B[arm_left_1_link];
            B --> C[arm_left_2_link];
            C --> D[arm_left_3_link];
            D --> E[arm_left_4_link];
            E --> F[arm_left_5_link];
            F --> G[arm_left_6_link];
            G --> H[arm_left_7_link];
            H --> I[gripper_left_outer_link];
            A[torso_lift_link] --> J[head_link_1];
            J --> K[head_link_2];
            A[torso_lift_link] --> L[arm_right_1_link];
            L --> M[arm_right_2_link];
            M --> N[arm_right_3_link];
            N --> O[arm_right_4_link];
            O --> P[arm_right_5_link];
            P --> Q[arm_right_6_link];
            Q --> R[arm_right_7_link];
            R --> S[gripper_right_outer_link];
        ```


2. After selecting the kinematic chains, you can add a payload to the end effector link or to any link in its kinematic tree (only if the advanced parameter is enabled). This step is not necessary, if you just want to visualize the workspace area without adding a payload.

3. By clicking on the **Calculate Workspace Area** button in the interactive marker menu, the node will start computing the workspace area for the selected end effector links. The computational time depends on the complexity of the kinematic chain and the level of resolution set in the parameters.

4. After the computation is finished, the workspace will be visualized in RViz as points with labels, representing the reachable positions of the selected end effector links. In order to have a better visualization, it is recommended to enable only one topic between **/workspace_area** and **/maximum_payloads** at a time. 

    When visualizing the workspace area, to analyze the requirement for a specific joint, you can select the corresponding namespace in the interactive markers menu.  

    <div style="text-align: center;">
    <img src="images/namespaces_tiago_pro.png" alt="Workspace Area" width="600"/>
    </div>
    In the image above, you can see the namespaces for the right arm joints, where each namespace has points representing the reachable positions of the end effector link selected in the right arm kinematic chain, and the color of the points represents the amount of torque for that joint in the different reachable positions.
    
    The image below shows the workspace area for the left arm kinematic chain, with the end effector link set to **gripper_left_outer_link**. The points are colored based on the torque required for the **arm_left_2_joint** with the torque limits as target values.
    <div style="text-align: center;">
    <img src="images/tiago_pro_calculated_ws.png" alt="Workspace Area" width="600"/>
    </div>

