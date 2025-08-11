# Documentation

## Installation
The package requires **Pinocchio** library to be installed. You can install it using tutorials available at [Pinocchio Installation](https://github.com/stack-of-tasks/pinocchio?tab=readme-ov-file#ros)


## Usage
After building the package, in order to use the dynamic payload analysis, you can run the following command:

```bash
ros2 run dynamic_payload_analysis_ros node_rviz_visualization_menu
```

In order to visualize the published objects in RViz, you can add the following markers:
- MarkerArray : **/torque_visualization**
- MarkerArray : **/workspace_area**
- MarkerArray : **/external_forces**
- MarkerArray : **/maximum_payloads**
- InteractiveMarkers : **/menu_frames**

After adding the markers, you'll be able to see a view similar to the one shown below:

<div style="text-align: center;">
<img src="images/photo_overview.png" alt="Overview" width="600"/>
</div>

### Calculating the workspace area
1) With a right click on the interactive markers, you can open a menu to select which kinematic chains you want to analyze by selecting the corresponding end effector link in the sub-menu. The menu will look like this:

<div style="text-align: center;">
<img src="images/selection_end_effector.gif" alt="Payload selection" width="600"/>
</div>

2) After selecting the kinematic chains, you can add a payload to the end effector link or to any link in its kinematic tree (only if the advanced parameter is enabled), as shown below:

<div style="text-align: center;">
<img src="images/add_payload.gif" alt="Add payload" width="600"/>
</div>

3) After selecting the end effector link and adding a payload, you can compute the workspace area by clicking on the **Compute workspace** button in the interactive markers menu.

4) After computing the workspace, you'll be able to see the valid configurations in Rviz as points and labels, as shown in the image below:

<div style="text-align: center;">
<img src="images/photo_workspace.png" alt="Workspace Area" width="600"/>
</div>

## Visualization logic
The position of visualized points represents the selected end points (link) of the robot kinematic chains and the color of the point represents the amount of torque based on different target torque. <br>
The script publishes a point for each joint of the selected kinematic chains, so in order to visualize only the color of a specific joint you can select only the namespace related to that joint in the interactive markers menu, as shown below:
<div style="text-align: center;">
<img src="images/namespace_selection.png" alt="Namespace Selection" width="600"/>
</div>

The script allow you to visualize the colors of the joints with different target torques:
- **Joint limits** : the colors are based on the joint limits provided by the URDF file
- **Maximum current torque** : the colors are based on the maximum current torques of the joints in the current payloads configuration


You can select the target torque in the interactive markers menu, as shown below:

<div style="text-align: center;">
<img src="images/color_points.png" alt="Target Torque Selection" width="600"/>
</div> 


## Colors legend
- **Green** : the joint torque is close to zero or the minimum current torque
- **Yellow** : the joint torque is close to the middle of the limits or the maximum current torque
- **Red** : the joint torque is close to the maximum joint torque or the maximum current torque

## Displaying of allowed configurations
The script allows you to visualize the allowed configurations of the robot arms in the workspace area. To select one of the allowed configurations, you can choose the configuration from the sub-menu of the **Compute workspace** button, as shown below:

<div style="text-align: center;">
<img src="images/config.gif" alt="Config selection" width="600"/>
</div>

Each point in the workspace area is labeled with its configuration number, which matches the configuration number in the sub-menu of the **Compute workspace** button.

## Computing the maximum payloads
The script calculates the maximum payloads for the selected end effector link and its kinematic tree. The computation is done together with the workspace area computation, so you can compute the maximum payloads by clicking on the **Compute workspace** button in the interactive markers menu.
The maximum payloads are visualized as points published in the **/maximum_payloads** topic, and they are displayed in RViz as shown below:

<div style="text-align: center;">
<img src="images/photo_maximum_payloads.png" alt="Maximum Payloads" width="600"/>
</div>

Furthermore, the maximum payloads for each valid configuration are displayed in the sub-menu of the **Compute workspace** button, as shown below: 

<div style="text-align: center;">
<img src="images/maximum_payloads.png" alt="Maximum Payloads Selection" width="600"/>
</div>

## Advanced parameters
The ros2 node has some advanced parameters that can be set in the launch command. One of the advanced parameters is the **advanced_mode** parameter, which allows you to add a payload to any link in the kinematic tree of the selected end effector link. If this parameter is set to false, you can only add a payload to the end effector link.
You can set the advanced parameters in the launch command as follows:

```bash
ros2 run dynamic_payload_analysis_ros node_rviz_visualization_menu --ros-args -p advanced_mode:=true
```

The second parameter is the **resolution_ik**, which allows you to set the resolution of the inverse kinematics computation. The default value is 0.20 meters, but you can set it to a different value if needed. You can set the resolution in the launch command as follows:

```bash
ros2 run dynamic_payload_analysis_ros node_rviz_visualization_menu --ros-args -p resolution_ik:=0.10
```
