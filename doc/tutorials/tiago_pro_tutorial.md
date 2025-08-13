# Tiago Pro Tutorial
This tutorial provides a guide on how to use the dynamic payload analysis for the **Tiago Pro robot**. It covers the visualization of workspace areas, joint torques, and maximum payloads.

# Prerequisites
Ensure you have the Tiago Pro robot's URDF file and the necessary ROS packages installed and built in the same workspace of the package **dynamic_payload_analysis_core**.
You can find the necessary URDF files in the corrisponding repository: [Tiago Pro URDF](https://github.com/pal-robotics/tiago_robot)

# Setting Up the Environment
1. Launch the Tiago Pro robot description and RViz using the following command:
    ```bash
    ros2 launch tiago_pro_description show.launch.py
    ```
    **Note:** This launch file will start also the joint state publisher, which is not needed for the dynamic payload analysis because the node publishes the joint states of the robot. You can remove the joint state publisher from the launch file if you want to avoid conflicts.

2. After opening RViz, you can add the following topics to visualize the results:
   - **/torque_visualization**: MarkerArray for visualizing the torque of the joints as labels in RViz.
   - **/workspace_area**: MarkerArray for visualizing the workspace area as points with labels in RViz.
   - **/external_forces**: MarkerArray for visualizing the external forces as arrows in RViz.
   - **/maximum_payloads**: MarkerArray for visualizing the maximum payloads as points with labels in RViz.
   - **/menu_frames**: InteractiveMarkers for visualizing the interactive marker menu in RViz.

3. After adding the markers, you'll be able to see a view similar to the one shown below:
   <div style="text-align: center;">
   <img src="images/tiago_pro_overview.png" alt="Overview" width="600"/>
   </div>

    **Note**: Make sure to have as fixed frame the **base_link** of the robot, so you can visualize the markers correctly.

# Analyzing the Workspace Area
1. Right-click on the interactive markers to open a menu where you can select which kinematic chains you want to analyze by selecting the corresponding end effector link in the sub-menu.
    for tiago pro, the main kinematic chains are:
    - **arm_left**: Left arm kinematic chain, with the following possible end effector links:
        - **arm**
        - 
    - **arm_right**: Right arm kinematic chain, with the following possible end effector links:
        - **arm**
    - **head**: Head kinematic chain, with the following possible end effector links:
        - **head_link_1**
        - **head_link_2**

2. After selecting the kinematic chains, you can add a payload to the end effector link or to any link in its kinematic tree (only if the advanced parameter is enabled). This step is not necessary, if you just want to visualize the workspace area without adding a payload.

3. By clicking on the **Calculate Workspace Area** button in the interactive marker menu, the node will start computing the workspace area for the selected end effector link. The computational time depends on the complexity of the kinematic chain and the level of resolution set in the parameters.

4. After the computation is finished, the workspace are will be visualized in RViz as points with labels, representing the reachable positions of the end effector link. In order to have a better visualization, it is recommended to enable only one topic between **/workspace_area** and **/maximum_payloads** at a time. 

    When visualizing the workspace area, to analyze the requirement for a specific joint, you can select the corresponding namespace in the interactive markers menu.  

    <div style="text-align: center;">
    <img src="images/tiago_pro_workspace.png" alt="Workspace Area" width="600"/>
    </div>

