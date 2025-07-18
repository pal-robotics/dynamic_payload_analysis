#!/usr/bin/env python3

# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import numpy as np

from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from enum import Enum

# enumeration to define the type of torque color visualization
class TorqueVisualizationType(Enum):
    TORQUE_LIMITS = 1
    MAX_CURRENT_TORQUE = 2

class MenuPayload():
    def __init__(self, node):
        # create server for interactive markers
        self.server = InteractiveMarkerServer(node, 'menu_frames')

        # array to store the checked orunchecked frames and payload selection
        self.frames_selection = np.array([],dtype=object)

        # create handler for menu
        self.menu_handler = MenuHandler()

        #current managed frame
        self.current_frame = None

        # flag to compute workspace
        self.compute_workspace = False

        # variable to store the selected configuration to display in Rviz
        self.selected_configuration = None

        # payload mass selection array
        self.payload_selection = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 1, 1.5, 1.8, 2.0, 2.5, 3.0, 3.5], dtype=float)

        # variable to store selection between torque limits or max torque in the current configuration for color visualization
        self.torque_color = TorqueVisualizationType.TORQUE_LIMITS # default is torque limits

        # insert the root menu items
        self.root_frames = self.menu_handler.insert('Select where to apply payload')
        self.workspace_button = self.menu_handler.insert('Compute workspace', callback=self.callback_workspace)
        # insert the reset payload button 
        self.reset = self.menu_handler.insert('Reset payloads', parent=self.root_frames, callback=self.callback_reset)
        
        # insert the checker for visualization color choice between torque limits or max torque in the current configuration
        self.torque_limits_checker = self.menu_handler.insert('Torque limits',command=str(TorqueVisualizationType.TORQUE_LIMITS.value) , callback=self.callback_color_selection)
        self.max_torque_checker = self.menu_handler.insert('Max torque', command=str(TorqueVisualizationType.MAX_CURRENT_TORQUE.value) , callback=self.callback_color_selection)

        self.menu_handler.setVisible(self.torque_limits_checker, False)
        self.menu_handler.setVisible(self.max_torque_checker, False)
        

        self.make_menu_marker('menu_frames')
        # add server to menu and apply changes
        self.menu_handler.apply(self.server, 'menu_frames')
        self.server.applyChanges()


    def insert_frame(self, name):
        """
        Insert a new item(frame) in the checkbox menu of frames.

        Args:
            name (str) : name of the frame  
        """
        last_item = self.menu_handler.insert(name, parent=self.root_frames, callback=self.callback_selection)
        self.menu_handler.setCheckState(last_item, MenuHandler.UNCHECKED)
        self.menu_handler.setVisible(last_item, True)
        
        # add the item to the checked frames array in order to keep track of the checked items
        self.frames_selection = np.append(self.frames_selection, {"name": name, "checked" : False, "payload" : 0.0} )

        # apply changes
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()
    
    def callback_color_selection(self, feedback):
        """
        Callback for torque selection type to change the visualization color in Rviz.
        
        Args:
            feedback: Feedback from the menu selection.
        """
        # get the handle of the selected item (id)
        handle = feedback.menu_entry_id
        # get the title of the selected item
        title = self.menu_handler.getTitle(handle)
        # get the entry object from the menu handler with all the informations about the item
        color_context = self.menu_handler.entry_contexts_[handle]

        # get the command field, in this case it is used to store if the selected item is a torque limits or max torque
        self.torque_color = TorqueVisualizationType.TORQUE_LIMITS if color_context.command == str(TorqueVisualizationType.TORQUE_LIMITS.value) else TorqueVisualizationType.MAX_CURRENT_TORQUE

        if self.torque_color == TorqueVisualizationType.TORQUE_LIMITS:
            # set the torque limits checker as checked and max torque checker as unchecked
            self.menu_handler.setCheckState(self.torque_limits_checker, MenuHandler.CHECKED)
            self.menu_handler.setCheckState(self.max_torque_checker, MenuHandler.UNCHECKED)
        elif self.torque_color == TorqueVisualizationType.MAX_CURRENT_TORQUE:
            # set the max torque checker as checked and torque limits checker as unchecked
            self.menu_handler.setCheckState(self.max_torque_checker, MenuHandler.CHECKED)
            self.menu_handler.setCheckState(self.torque_limits_checker, MenuHandler.UNCHECKED)
        
        
        # apply changes
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


    def callback_workspace(self, feedback):
        """
        Callback to compute the workspace based on the selected frames and payloads.
        This function is called when the user clicks on the "Compute workspace" button.
        """
        self.compute_workspace = True
        
        
    def insert_dropdown_configuration(self, configuration : np.ndarray):
        """
        Insert to the dropdown the possible configurations.

        Args:
            configuration (np.ndarray): Array of configuration to be displayed in the dropdown.
        """

        for i, item in enumerate(configuration):
            # insert the parent in the command field to keep track of the parent id
            last_entry = self.menu_handler.insert(f"Configuration {i} | arm: " + item["arm"], parent=self.workspace_button, command=str(self.workspace_button), callback=self.callback_configuration_selection)
            self.menu_handler.setCheckState(last_entry, MenuHandler.UNCHECKED)
            self.menu_handler.setVisible(last_entry, True)
        
        # set visible true for color selection and put the default check state
        self.menu_handler.setVisible(self.torque_limits_checker, True)
        self.menu_handler.setVisible(self.max_torque_checker, True)
        self.menu_handler.setCheckState(self.torque_limits_checker, MenuHandler.CHECKED)
        self.menu_handler.setCheckState(self.max_torque_checker, MenuHandler.UNCHECKED)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


    def callback_configuration_selection(self, feedback):
        """
        Callback for configuration selection to change the check state of the selected item.
        
        Args:
            feedback: Feedback from the menu selection.
        """
        # get the handle of the selected item (id)
        handle = feedback.menu_entry_id
        # get the title of the selected item (it contains configuration number)
        title = self.menu_handler.getTitle(handle)

        # set the checkbox as checked
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)

         # get the entry object from the menu handler with all the informations about the item (command field is used to store the parent id)
        config_context = self.menu_handler.entry_contexts_[handle]

        # get the parent id of the selected configuration stored in the command field
        parent_id = int(config_context.command)
        # get the entry object of the parent
        parent_context = self.menu_handler.entry_contexts_[parent_id]
        
        # reset all the selections in the configuration sub-menu
        # check if a item is already checked, if so remove it and set to unchecked to prevent multiple configuration selection in the same menu
        for item in parent_context.sub_entries:
            # check if the item is checked
            if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
                # if the configuration is already checked, we need to uncheck it
                self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
        
        # set the selected configuration checkbox as checked
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        # update the selected configuration
        self.selected_configuration = int(title.split()[1])  # Extract the configuration number from the title
                
        # apply changes
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


    def callback_reset(self, feedback):
        """
        Callback for reset menu selection to remove all checked items from the menu.
        """
        # remove all checked items from the array of payloads selection
        for i, item in enumerate(self.frames_selection):
            if item['checked']:
                item = {"name": item['name'], "checked": False, "payload": 0.0}
                self.frames_selection[i] = item
                
        # reset the frames selection menu (i = number of entry, item = object with name, sub entries, etc.)
        for i, item in self.menu_handler.entry_contexts_.items():
            if i == 1:
                # skip the root item (payload reset)
                continue
            
            # check if the item(frame) has sub entries (payloads selection)
            if item.sub_entries:
                # if the frame has payloads selection, we need to remove it
                for sub_item in item.sub_entries:
                    self.menu_handler.setVisible(sub_item, False)
                    self.menu_handler.setCheckState(sub_item, MenuHandler.UNCHECKED)
            
             # check if the item is the reset payloads or compute workspace item and skip the unchanging of the check state
            if item.title == "Reset payloads" or item.title == "Compute workspace":
                continue
                
            # set the checked of frame to unchecked 
            self.menu_handler.setCheckState(i,MenuHandler.UNCHECKED)
        
        # reset the selected configuration
        self.selected_configuration = None

        # hide the torque limits and max torque checkboxes when there is no configuration selected
        self.menu_handler.setVisible(self.torque_limits_checker, False)
        self.menu_handler.setVisible(self.max_torque_checker, False)

        self.torque_color = TorqueVisualizationType.TORQUE_LIMITS  # reset to default torque limits

        # reapply the menu handler and server changes
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    
    def callback_selection(self, feedback):
        """
        Callback for menu selection to change the check state of the selected item(frame).

        Args:
            feedback: Feedback from the menu selection.
        """
        # get name of the frame
        handle = feedback.menu_entry_id
        title = self.menu_handler.getTitle(handle)

        # add payloads selection as a submenu
        self.manage_payload_menu(handle)
        # update the frames_selection array to set the item as checked
        self.update_item(title, True)
        
        # set the checkbox as checked
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        
        # update the menu state
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        # print the current state of the checked frames
        print(f"{self.get_item_state()} \n")

        
    

    def update_item(self, name, check: bool):
        """
        Update the state of an item(frame) in the array of selected frames with payload.
        
        Args:
            name (str): Name of the item(frame) to update.
            check (bool): New state of the item.
        """
        
        for item in self.frames_selection:
            if item['name'] == name:
                item['checked'] = check
                break


    def update_payload(self, name, payload: float):
        """
        Update the payload mass of an item in the array of selected frames with payload.
        
        Args:
            name (str): Name of the item(frame) to update.
            payload (float): New payload mass that act on the frame.
        """
        
        for item in self.frames_selection:
            if item['name'] == name:
                item['payload'] = payload
                break


    def manage_payload_menu(self, menu_entry_id):
        """
        Add payload selection items as a sub-menu for the specified menu entry (selected frame).

        Args:
            menu_entry_id : ID of the menu entry.
        """
        # insert as a sub menu all the payload stored in the array
        for payload in self.payload_selection:
            # insert the payload mass selection items (command = str(menu_entry_id) is used to identify the parent menu entry)
            last_entry = self.menu_handler.insert(f"{payload} kg", parent=menu_entry_id, command = str(menu_entry_id), callback=self.update_payload_selection)
            self.menu_handler.setCheckState(last_entry, MenuHandler.UNCHECKED)
            self.menu_handler.setVisible(last_entry, True)
        
        # apply changes
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()
    

    def update_payload_selection(self, feedback):
        """
        Callback for payload selection to change the current payload.
        
        Args:
            feedback: Feedback from the menu selection.
        """
        # get the handle of the selected item (id)
        handle = feedback.menu_entry_id
        # get the title of the selected item (it contains number of kg)
        title = self.menu_handler.getTitle(handle)
        # get the entry object from the menu handler with all the informations about the item (command field is used to store the parent id)
        payload_context = self.menu_handler.entry_contexts_[handle]

        # get the parent id of the selected payload stored in the command field
        parent_id = int(payload_context.command)
        # get the entry object of the parent
        parent_context = self.menu_handler.entry_contexts_[parent_id]
        # get the name of the parent item (frame name)
        name_parent = self.menu_handler.getTitle(parent_id)
        
        # reset all the selections in the payload sub-menu
        # check if a item is already checked, if so remove it and set to unchecked to prevent multiple payload selection in the same menu
        for item in parent_context.sub_entries:
            # check if the item is checked
            if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
                # if the item is already checked, we need to uncheck it and set the payload to 0.0
                self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
                self.update_payload(name_parent, 0.0)
                
        # set the selected payload checkbox as checked
        self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        # uppate the array with the new payload 
        self.update_payload(name_parent, float(title.split()[0]))  # Extract the payload mass from the title
        
        # apply changes
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()


    def get_torque_color(self) -> TorqueVisualizationType:
        """
        Return the type of torque color visualization selected.
        
        Returns:
            TorqueVisualizationType: The type of torque color visualization.
        """
        return self.torque_color


    def get_selected_configuration(self) -> str:
        """
        Return the selected configuration to display in Rviz.
        
        Returns:
            str: The selected configuration number.
        """
        return self.selected_configuration

    def get_workspace_state(self) -> bool:
        """
        Return the state of the workspace computation flag.
        
        Returns:
            bool: True if the workspace should be computed, False otherwise.
        """
        return self.compute_workspace
    

    def set_workspace_state(self, state: bool):
        """
        Set the state of the workspace computation flag.
        
        Args:
            state (bool): True to compute workspace, False otherwise.
        """
        self.compute_workspace = state


    def get_item_state(self) -> np.ndarray:
        """
        Return array of checked frames in the menu list
        
        Returns:
            np.ndarray: array of checked frames
        """
        return self.frames_selection
    

    def make_label(self, msg):
        """
        Create the label used to click over in rviz
        """
        marker = Marker()

        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = "Click here to select frame"
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.1
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker
    
    def make_label_control(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_label(msg))
        msg.controls.append(control)
        
        return control


    def make_empty_marker(self, dummyBox=True):
        """
        Create interactive marker
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose.position.z = 2.0
        int_marker.scale = 0.5
        
        return int_marker


    def make_menu_marker(self, name):
        """
        Create interactive menu 
        """
        int_marker = self.make_empty_marker()
        int_marker.name = name

        control = InteractiveMarkerControl()

        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append(self.make_label(int_marker))
        int_marker.controls.append(control)

        self.server.insert(int_marker)


    def shutdown(self):
        """
        Shutdown the interactive marker server.
        """
        self.server.shutdown()