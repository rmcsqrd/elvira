## NODE CHEATSHEET

There are several nodes in this catkin workspace, only edit the ones in the "editable" list (or stuff will break).

### Editable Nodes
- __motor_driver__
This node handles all the motor driver code on Elvira

- __julia_ros__
This node handles the motion planning. It takes in image data as the state space and sends actions to the motor driver

- __multip_blob__  
This node characterizes certain color types and groups them into blobs. 

### "Don't Touch" Nodes

- ddynamic_reconfigure
- image_common
- realsense-ros
