# ros pose estimation module
This module is for pose estimation given the segmented point cloud of object, returning the 6-DOF pose of the object w.r.t the eye-in-hand camera.

### Preparation
Install the Point Cloud Library ([PCL](https://pointclouds.org/))

`catkin_make` to compile the `pcl_test` node, using FPFH-based RANSAC and ICP algorithm based on template matching.

### How to run 
Once the point cloud of target object has been published through `ros_seg`, run the following command to find the pose.
- `rosrun pcl_test pcl_estimation`

Other auxiliary files could be found in `./src` to write, read or visualize the point cloud.
