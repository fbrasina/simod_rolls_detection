This software is part of the [SiMOD](https://www.unibo.it/it/ricerca/progetti-e-iniziative/pr-fesr-emilia-romagna-2021-2027/1223/20430/20650) project.

simod_rolls_detection
=====================

Pallet detection
----------------

### Pallet detection node

The `pallet_detection_node` node locates a specific configuration of boxes (aka "*pallet*") in a RGB-D image. Detection is started by calling a ROS action.

In summary, inputs are:

- A description of the configuration of boxes to be located.
- A RGB-D image of the boxes, taken from the front with the camera principal axis roughly parallel to the ground.
- The camera pose with respect to a *base* reference frame.<br />**Note**: Any frame can be used here, but the Z axis of the *base* frame must be  vertical, i.e. perpendicular to the ground.
- The height of the ground with respect to the *base* frame.
- An initial guess of the pose of the pallet.

Outputs are:

- The pose of the pallet with respect to the *base* frame.
- The pose of each box in the pallet, with respect to the *base* frame.

**Parameters**

The algorithm within the node has many configuration parameters, which are beyond the scope of this readme. Useful parameters are:

- `detect_pallet_action`: the name of the action server which will be created by this node. Default: `/detect_pallet`.
- `depth_image_topic`, `rgb_image_topic` and `camera_info_topic`: the node will subscribe to these topics to read depth, color and camera_info of the RGB-D image.
- `discard_first_camera_frames`: if the first images sent by the camera are corrupted for some reason, set this to a positive integer value to ignore this many first images received.
- `auto_generate_plane_pillars` (bool): if true, planes and pillars are auto-generated from box definition (see the *pallet description* section below).

When the action is called, the node first waits for messages from the camera on the `depth_image_topic`, `rgb_image_topic` and `camera_info_topic` topics.
The detection algorithm is executed only when at least one message is received for each topic.

**Action**

Detection is started by calling the `/detect_pallet` action. The action is of type `simod_rolls_detection/DetectPallet.action`. The action goal has these fields:

- `camera_pose`: pose of the camera with respect to the *base* frame.
- `pallet_description_filename`: a string containing the file name where the box configuration is described.
- `floor_z`: height of the ground with respect to the *base* frame. Points below this level will be ignored, as they are considered part of the ground.
- `initial_guess_center`: initial guess of the center of the pallet (meters).
- `initial_guess_rotation`: initial guess of the pallet rotation (rad), around an axis passing through the center of the pallet and parallel to the Z axis of the *base* frame.
- `initial_guess_size`: size (meters) of a bounding box centered in `initial_guess_center` and oriented using `initial_guess_rotation`. Points outside this box will be ignored, hence the pallet can only be located within this box.

**Pallet description**

The pallet description file (whose name is in `pallet_description_filename`) contains a list of elements which describe the pallet, one for each line.
Available elements are *plane*, *pillar* and *box*. The *guess* element is also available but it is now unused.

Each element may be followed by `name *word*` which defines a name for the element. `*word*` may be only a single word.

Comments may also be added starting with `#` at the beginning of the line.

Coordinates of the elements are with respect to a local *pallet* reference frame, which is an arbitrary reference frame which should be roughly located at the center of the pallet.

If the parameter `auto_generate_plane_pillars` is true, the *plane* and *pillar* elements are ignored. Instead, plane and pillar elements are auto-generated from the *box* elements and a virtual viewpoint. Only planes and pillars facing the virtual viewpoint are created.  
The virtual viewpoint is located at the 2D coordinates specified by the parameters `auto_generate_plane_pillars_viewpoint_x` and `auto_generate_plane_pillars_viewpoint_y`. These parameters should be set to the approximate position of the camera in the local coordinates of the pallet. Default values are `(-1, 0)`, i.e., the virtual viewpoint is located along negative X axis.

*plane*

Planes are defined with the string `plane` followed by four numbers.
The four numbers are the plane equation parameters `a`, `b`, `c`, `d` so that:

```
ax + by + cz + d = 0
```

*pillar*

Pillars are vertical edges.
They are defined with the string `pillar` followed by four numbers: `x`, `y`, `min_z`, `max_z`.
`x` and `y` represent the position of the pillar on the ground.
The pillar extends from a minimum height of `min_z` and a maximum height of `max_z`.

Optionally, pillar definitions may be followed by one or more relations with planes in the pallet, by adding `left *plane_name*` or `right *plane_name*` at the end of the line.
These indicate that the pillar is in contact with a vertical box face which belongs to the plane.
`left` indicates that the pillar is at the left edge of the face, when the face is observed from the outside of the pallet and the pallet is upright (with respect to the *base* frame, regardless of camera pose). Conversely, `right` indicates that the pillar is at the right edge of the face.

*box*

The box command indicates the pose of a box within the pallet.
They are defined with the string `box` followed by 7 numbers: the box size `size_x`, `size_y`, `size_z`, the position of the center `x`, `y`, `z`, and the rotation around the vertical axis (rad).

*Example*

The following example defines two planes, with two pillars each, and three boxes.

```
# plane: a b c d z_min z_max
# pillar: x y z_min z_max
plane -1    0    0      0    0  0.2 name front_plane
pillar 0    0.425  0.0    0.2 left front_plane
pillar 0    -0.425 0.0    0.2 right front_plane
plane -1    0    0      0.35  0.2 0.6 name back_plane
pillar 0.35  0.425  0.2    0.6 left back_plane
pillar 0.35  -0.425 0.2    0.6 right back_plane
# box: size_x size_y size_z x    y    z     rotation_z
box    0.3    0.85   0.2    0.15 0.0   0.1  0
box    0.3    0.85   0.2    0.45 0.0   0.3  0
box    0.3    0.85   0.2    0.45 0.0   0.5  0
```

**Results**

The action result has these fields:
- `pallet_pose`: pose of the *pallet* reference frame, with respect to the *base* frame.
- `box_poses`: array of poses of each box in the pallet.
- `consensus`: integer, consensus size of the final RANSAC.
- `success`: a Boolean reporting if detection was successful. The consensus of the final RANSAC must be at least 2 to achieve a success. In case of failure, the initial guess will be returned as `pallet_pose`.

**Debug information**

These topics are published by the `pallet_detection_node` with some useful debug information:

- `point_cloud` (sensor_msgs/PointCloud2): point cloud with detected vertical planes in different colors.
- `input_point_cloud` (sensor_msgs/PointCloud2): initial point cloud computed from the depth image.
- `valid_points_cloud` (sensor_msgs/PointCloud2): point clouds where invalid points have been filtered according to some heuristics.
- `depth_image` (sensor_msgs/Image): depth imagewhere invalid points have been filtered according to some heuristics.
- `cluster_image` (sensor_msgs/Image): image with vertical planes in grayscale.
- `edge_image` (sensor_msgs/Image): image with detected lines.
- `plane_image` (sensor_msgs/Image): image with vertical planes in different colors.
- `markers` (visualization_msgs/MarkerArray): visualization of pallet elements (*yellow*: elements of the pallet description, *blue*: elements detected on the image, *green*: elements of the description transformed into the pallet pose, *red*: same as green, but with pose refinement).

Moreover, the TF reference frame of each detected box is published in the form `box_X` with respect to the *base* TF frame.

### Saving images

The node `save_images` is a utility to save the information needed by the `pallet_detection` node, so that it can be used offline.
A launch file `save_images.launch` is also provided.
The launch file also starts the OAK-D Pro camera driver `oak_d_pro.launch` from `simod_camera`.

Upon startup, the node opens an OpenCV window with the current RGB frame from the camera. Press any key except `Esc` to save a frame. Press `Esc` to exit.

The following files are saved:

- `depthX.png`: depth image as 16-bit grayscale PNG file
- `imageX.png`: RGB image as 8-bit color PNG file
- `camera_info.txt`: focal lengths and image center from camera info
- `poseX.txt`: transformation between two TF frames, usually the *base* reference frame and the camera frame

`X` starts from 0 and auto-increments. `X` is reset to 0 if the node is restarted.

Parameters:

- `save_folder_name`: folder where the files will be saved.
- `tf_base_frame` name of the *base* reference TF frame.
- `tf_camera_frame` name of the camera TF frame.
- `depth_image_topic`, `color_image_topic` and `camera_info_topic`: topic names for the depth image, the color image and the camera info, respectively.
- `save_color`, `save_depth`, `save_pose` and `save_camera_info`: boolean flags. Set these to false to disable saving of the respective file.

### Pallet detection test

The `pallet_detection_test` node (`pallet_detection_test.cpp`) is an example node which calls the action of `pallet_detection_node`. An example launch file `pallet_detection.launch` is also provided.

The node can operate in two modes, depending on the parameter `use_real_camera`. If `use_real_camera` is false, the node loads the RGB and depth images from file and publishes them while the action is executing, to simulate the camera.

If `use_real_camera` is true, the node will not publish to the topics. Instead, in the launch file the `oak_d_pro.launch` file is included to connect to the real camera.

The node reads the current camera pose with respect to the *base* reference frame from TF. The *base* TF frame is configured using parameter `world_frame_id`, and camera frame is configured using parameter `camera_frame_id`. If `use_real_camera` is false, the node also loads the camera pose from file (see parameter `camera_pose_filename`) and publishes it between these two TF, to simulate an external source which provides the camera pose.

**Parameters**

- `rgb_filename`: name of the color image file (only if `use_real_camera` is false).
- `depth_filename`: name of the color image file (16-bit PNG) (only if `use_real_camera` is false).
- `camera_info_filename`: name of the camera info file (text file) (only if `use_real_camera` is false).
- `expected_pallet_filename`: file name of the pallet description.
- `camera_pose_filename`: file name of the camera pose (only if `use_real_camera` is false).
- `initial_guess_filename`: file name of the initial guess.
- `world_frame_id`: name of the *base* TF frame.
- `camera_frame_id`: name of the *camera* TF frame.

*Camera info file*

Format of the camera info file is the same which is saved by the `save_images` node:

```
fx 1029.6
fy 1029.11
cx 632.373
cy 365.709
```

where `fx` and `fy` are the focal lengths and `cx`, `cy` is the image center.

*Initial guess file*

Format of the initial guess file is:

```
center   0.0 0.0 0.0
size     4.0 4.0 3.0
rotation 0.0
floor    -0.35
```

where `center` is the initial guess of the pallet center in *base* coordinates,  `size` is the initial guess size, `rotation` is the initial guess rotation, and `floor` is the ground height `floor_z`.

*Camera pose file*

The camera pose file contains a 4x4 3D transformation matrix, with four numbers on each line. Example:

```
0 0 1 0
1 0 0 0
0 1 0 0
0 0 0 1
```

Close line detection
--------------------

### Close line detection node

The `close_line_detection_node` detects the separation line between two plastic-wrapped boxes of paper rolls. The two boxes are observed from above using a RGB camera.

In summary, inputs of the node are:

- A RGB image
- Intrinsic parameters of the camera (focal length, image center)
- The camera pose with respect to a *base* reference frame.<br />**Note**: The Z axis of the *base* frame must be vertical, i.e. perpendicular to the top surface of the boxes, and the Y axis must be roughly perpendicular to the line to be located.
- The height of the top surface of the boxes with respect to the *base* frame (along Z).
- An initial guess of the position of the line, that is an estimate value of the Y coordinate of the line in the *base* frame.

The output of the algorithm is a sequence of 3D points in the *base* reference frame. The points represent the detected line as a polyline.

The `close_line_detection` detection node works by first re-projecting the image onto a virtual view from above, and then using pattern matching on the image. The patterns are circular or semi-circular patterns which match rolls surrounded by a dark region.

**Parameters**

Main parameters of the node are:

- `rgb_image_topic` (string): name of the topic where the RGB image will be published.
- `camera_info_topic` (string): name of the topic where the camera info will be published.
- `discard_first_camera_frames` (int): the first `discard_first_camera_frames` images received will be ignored. Use this if the first frames are bad for some reason (e.g. auto-exposure still initializing).
- `mode_basic` (bool): if true, always outputs a polyline with only two points, i.e., a single segment.
- `detect_close_line_action` (string): name of the action to be called to perform the detection. By default, `/detect_close_line`.
- `roll_diameter` (double): diameter of the paper rolls, in meters.

**Action**

Detection is started by calling the `/detect_close_line` action. The action is of type `simod_rolls_detection/DetectCloseLine.action`. The action goal has these fields:

- `camera_pose`: pose of the camera with respect to the *base* frame.
- `layer_z`: height of the top surface of the boxes in the *base* frame.
- `initial_guess_y`: initial guess of the Y coordinate of the line.
- `initial_guess_window_size_y`: uncertainty on the initial guess Y, in meters. A good value may be `0.2`.
- `initial_guess_x`: initial guess of the X coordinate of the line.
- `initial_guess_window_size_x`: uncertainty on the initial guess X, in meters.<br />**Note**: initial guess X is not really important, so you may just set `initial_guess_x` to `0` and `initial_guess_window_size_x` to a large value.

When the action is called, the node first waits for messages from the camera on the `rgb_image_topic` and `camera_info_topic` topics.
The detection algorithm is executed only when at least one message is received for each topic.

The the action result contains these fields:

- `success` (bool): true if the detection succeeded.
- `points` (array of geometry_msgs/Point): the sequence of 3D points representing the polyline.

**Debug information**

During execution of the action, several debug images are published to several topics. The most interesting topic is likely the `~max_line_image`, which shows the detected line on the (re-projected) image.

In case of detection error, also the `~correlation_total_image` may be interesting to see the regions of the image which incorrectly matched the patters.

### Close line detection test

The `close_line_detection_test` node (`close_line_detection_test.cpp`) is an example node which calls the action of `close_line_detection_node`. An example launch file `close_line_detection.launch` is also provided.

The node can operate in two modes, depending on the parameter `use_real_camera`. If `use_real_camera` is false, the node loads the RGB image and the camera info from file and publishes them while the action is executing, to simulate the camera.

If `use_real_camera` is true, the node will not publish to the topics. Instead, in the launch file the `oak_d_pro.launch` file is included to connect to the real camera.

The node reads the current camera pose with respect to the *base* reference frame from TF. The *base* TF frame is configured using parameter `world_frame_id`, and camera frame is configured using parameter `camera_frame_id`. If `use_real_camera` is false, the node also loads the camera pose from file (see parameter `camera_pose_filename`) and publishes it between these two TF, to simulate an external source which provides the camera pose.

After the execution of the action, the `close_line_detection_test` publishes all the points in the result as TF frames. The frames are named in the form `line_points_X` where X is a point.

**Parameters**

- `rgb_filename`: name of the color image file (only if `use_real_camera` is false).
- `camera_info_filename`: name of the camera info file (text file) (only if `use_real_camera` is false).
- `camera_pose_filename`: file name of the camera pose (only if `use_real_camera` is false).
- `world_frame_id`: name of the *base* TF frame.
- `camera_frame_id`: name of the *camera* TF frame.
- `use_real_camera`: see above.
- `layer_height`: see the `layer_z` field in the action.
- `initial_guess_y`, `initial_guess_window_size_y`, `initial_guess_x`, `initial_guess_window_size_x`: see the corresponding parameters in the action.

Roll packs detection
--------------------

The `roll_pack_detection_node` detects plastic-wrapped packs of paper rolls. The packs are observed from the _front_ using a RGB-D camera. The packs are deformable and are composed of a number of sub-packs (seven sub-packs in the current implementation).

In summary, inputs of the node are:

- A RGB image (required).
- A depth image.<br />**Note**: The depth image is optional. If it is not provided or all values are invalid (i.e., zero), only the RGB image will be used.
- Intrinsic parameters of the camera (focal length, image center).
- The camera pose with respect to a *base* reference frame.<br />**Note**: The Z axis of the *base* frame must be vertical, i.e. parallel to the front plane of the packs.
- A set of image templates (*references*) of the packs, observed from the front. Each template is composed of:
    - A color image template (*reference image*) of one of the packs, observed from the front.
    - An image mask of the image template (*reference mask*), with value 255 (white) where the template contains features of the pack, and 0 in regions where the features of the pack must be ignored.
    - A *reference description*, containing metadata about the reference image (see the "Reference description file" section below).

The output of the algorithm is a list of packs in the *base* reference frame. Each pack is described by:

- The 3D pose of the central sub-pack.
- Nominal width (x-axis), height (z-axis) and depth (y-axis) of the pack.
- Distance of the left edge of the pack and of the right edge of the pack, with respect to the center of the pack, along the x-axis. These may be different from `width/2`, since the pack is deformable.
- Index of the reference image which was matched to that pack.

The `roll_pack_detection_node` uses RANSAC feature matching with SIFT features. The matched model is not the standard homography, but it also takes into account the pack deformation as horizontal and vertical translation of the sub-packs within the pack.

**Parameters**

Main parameters of the node are:

- `rgb_image_topic` (string): name of the topic where the RGB image will be published.
- `depth_image_topic` (string): name of the topic where the depth image will be published.
- `camera_info_topic` (string): name of the topic where the camera info will be published.
- `discard_first_camera_frames` (int): the first `discard_first_camera_frames` images received will be ignored. Use this if the first frames are bad for some reason (e.g. auto-exposure still initializing).
- `detect_packs_action` (string): name of the action to be called to perform the detection. By default, `/detect_packs_action`.
- `ransac_iterations` (string): number of RANSAC iterations: more iterations mean higher probability of the correct result, at the expense of computation time. By default, `200000`.
- `no_depth` (bool): if true, the node does not use the depth image, only the RGB image.
- `max_valid_depth` (double): pixels with depth greater than this value (in meters) will be ignored. Default: 3 meters.

**Action**

Detection is started by calling the `/detect_packs_action` action. The action is of type `simod_rolls_detection/DetectPacks.action`. The action goal contains these fields:

- `camera_pose`: pose of the camera with respect to the *base* frame.
- An array of reference images using the `simod_rolls_detection/ReferenceImage` message type. In turn, this message type has these fields:
    - `reference_image_filename` (string): path of a color image containing the RGB reference image.
    - `reference_mask_filename` (string): path of a color image containing the mask of the reference image.
    - `reference_description_filename` (string): path of a file containing the metadata about the reference mask.
- `flip_image` (bool): if the input RGB image must be rotated 180 degrees before matching. This should not affect accuracy as SIFT features are rotation-invariant, but in practice if your camera is upside down setting this to true may be an improvement.

When the action is called, the node first waits for messages from the camera on the `rgb_image_topic`, `depth_image_topic` (optional), and `camera_info_topic` topics.
The detection algorithm is executed only when at least one message is received for each topic.

The the action result contains several array fields. The arrays have all the same length, equal to the number of detected packs.

- `pack_poses` (array of geometry_msgs/Pose): pose of the detected packs.
- `pack_edge_x_left` (array of float): distance of the left edge from the pack center, along the pack x-axis.
- `pack_edge_x_right` (array of float): distance of the right edge from the pack center, along the pack x-axis.
- `pack_height` (array of float): nominal height of the pack.
- `pack_depth` (array of float): nominal depth of the pack.
- `pack_reference_id` (array of int): id of the reference image which was matched to this pack.
- `is_upside_down` (array of bool): whether the pack was detected upright or upside down.

**Reference description file**

The reference description file contains metadata about the pack template. The main goal of this file is to establish a correspondence between the points in the template and real points in 3D space.

The following commands are available:

```
p name px py x y
```
where `name` is a unique string, `px` and `py` are integer pixel values and `x`, `y` are real coordinate values. Indicates the correspondence between point `px`, `py` in the template image and the real point with coordinates `[x, y, depth/2]` where depth is the pack depth as defined by the `box_size_whd` command.

```
box_size_whd width height depth
```
where `width`, `height` and `depth` define the box size, in meters. The `width` and `height` are the two dimensions which are facing the camera, `depth` is the third dimension.

```
pixel_size_in_meters s
```
Defines the approximate size `s` of a pixel of the template in the real world, in meters.

```
element_center px py
```
where `px` and `py` are integer pixel values. Creates a sub-pack in the template with center at those coordinates. As there should always be exactly `7` sub-packs in the current implementation, this command is currently ignored.

An example description is provided in the file `sollevamento_pacchi_reference2_points.txt` in the `data` folder.

**Debug information**

During the execution of the action, an image is published to the `~/matches_image` topic, showing the matched SIFT features and the packs contours in the image. Also, the input RGB-D image, converted into a point cloud, is published to the `~/input_cloud` topic.

### Roll pack detection test

The `roll_pack_detection_test` node (`src/roll_pack_detection_test.cpp`) is an example node which calls the action of the `roll_pack_detection_node`. An example launch file `roll_packs_detection.launch` is also provided.

The node can operate in two modes, depending on the parameter `use_real_camera`. If `use_real_camera` is false, the node loads the RGB image, the depth image and the camera info from file and publishes them while the action is executing, to simulate the camera.

If `use_real_camera` is true, the node will not publish to the topics. Instead, in the launch file the `oak_d_pro.launch` file is included to connect to the real camera.

The node reads the current camera pose with respect to the *base* reference frame from TF. The *base* TF frame is configured using parameter `world_frame_id`, and camera frame is configured using parameter `camera_frame_id`. If `use_real_camera` is false, the node also loads the camera pose from file (see parameter `camera_pose_filename`) and publishes it between these two TF, to simulate an external source which provides the camera pose.

**Parameters**

- `rgb_filename`: name of the color image file (only if `use_real_camera` is false).
- `depth_filename`: name of the color image file (only if `use_real_camera` is false).
- `camera_info_filename`: name of the camera info file (text file) (only if `use_real_camera` is false).
- `camera_pose_filename`: file name of the camera pose (only if `use_real_camera` is false).
- `world_frame_id`: name of the *base* TF frame.
- `camera_frame_id`: name of the *camera* TF frame.
- `use_real_camera`: see above.
- `flip_image`: see the `DetectPacks` action definition above.
- `reference_image_filename_X`, `reference_mask_filename_X`, `reference_description_filename_X`, where X is a number from `0` to `9`: the array of reference images, see the action definition above. <br />At most `10` reference images can be defined. <br />For backward compatibility, also the parameter names `reference_image_filename`, `reference_mask_filename` and `reference_description_filename` are supported.

**Debug information**

During the execution of the `roll_pack_detection_test` node, the detected packs are published as visualization markers to the topic `/detect_pack_markers`. 

For each pack, three TFs are also published, one in the pack center, one at the pack left edge and one at the pack right edge.