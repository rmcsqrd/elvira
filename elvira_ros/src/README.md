## NODE CHEATSHEET

There are several nodes in this catkin workspace, only edit the ones in the "editable" list (or stuff will break).

### Editable Nodes

- __circledraw__  
This node is really more of a "hello world" program to verify that opencv is working.

- __blob_detect__  
This node can filter based on pixel color. Any pixels that pass the threshold are turned green. This is mostly used to demonstrate how to edit individual pixels and create a header file.

- __multip_blob__  
This node characterizes certain color types and groups them into blobs. 

### "Don't Touch" Nodes

- ddynamic_reconfigure
- image_common
- realsense-ros
