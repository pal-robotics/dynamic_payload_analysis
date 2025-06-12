#!/usr/bin/env python3


import sys
import numpy as np

from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker


class MenuPayload():
    def __init__(self, node):
        # create server for interactive markers
        self.server = InteractiveMarkerServer(node, 'menu_frames')
        # array to store the checked frames
        self.checked_frames = np.array([],dtype=object)
        # position of the marker in rviz
        self.marker_pos = 2
        # create handler for menu
        self.menu_handler = MenuHandler()
        # insert the root menu item
        self.root = self.menu_handler.insert('Select where to apply payload')
        
        self.make_menu_marker('menu_frames')
        # add server to menu and apply changes
        self.menu_handler.apply(self.server, 'menu_frames')
        self.server.applyChanges()

    def insert(self, name):
        """
        Insert a new item in the checkbox menu.
        """
        last_item = self.menu_handler.insert(name, parent=self.root, callback=self.callback_selection)
        self.menu_handler.setCheckState(last_item, MenuHandler.UNCHECKED)
        self.menu_handler.setVisible(last_item, True)
        
        self.checked_frames = np.append(self.checked_frames, {"name": name, "checked" : False} )
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()
        
        
    def callback_selection(self, feedback):
        """
        Callback for menu selection
        """
        # change the check state of the selected item
        handle = feedback.menu_entry_id
        title = self.menu_handler.getTitle(handle)
        if self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.update_item(title, False)
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.update_item(title, True)
        
        # update the menu state
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        # print the current state of the checked frames
        print(f"{self.get_item_state()} \n")
    

    def update_item(self, name, status: bool):
        """
        Update the state of an item in the array.
        
        Args:
            name (str): Name of the item to update.
            status (bool): New checked state of the item.
        """
        
        for item in self.checked_frames:
            if item['name'] == name:
                item['checked'] = status
                break
    

    def get_item_state(self) -> np.ndarray:
        """
        Return array of checked frames in the menu list
        
        Returns:
            np.ndarray: array of checked frames
        """
        return self.checked_frames
    

    def make_box(self, msg):
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
    
    def make_box_control(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_box(msg))
        msg.controls.append(control)
        
        return control


    def make_empty_marker(self, dummyBox=True):
        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose.position.z = 1.0 * self.marker_pos
        int_marker.scale = 0.5
        
        
        return int_marker


    def make_menu_marker(self, name):
        int_marker = self.make_empty_marker()
        int_marker.name = name

        control = InteractiveMarkerControl()

        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append(self.make_box(int_marker))
        int_marker.controls.append(control)

        self.server.insert(int_marker)


    def shutdown(self):
        """
        Shutdown the interactive marker server.
        """
        self.server.shutdown()