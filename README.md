# depth2laser - ROS node
depth image converter to laser scan
## NOTE
the node is still a work in progress!
## Story
Lasers are costy. Xtions and Kinects are cheap.
(there are other motivations back, i'll finish coding first then i'll have time for chit chat)

## How to use it
To launch the node and the static TF tree, just write

 `roslaunch depth2laser tree.launch `
 
If you want to try it on a mobile robot fastest way is edit `tree.launch` and set a trasform between the camera frame ("xtion" in that file) and the base link frame.

Available params with deaults:

* _base_frame /world
* _camera_frame /xtion
* _virtual_laser_frame /laser
* _depth_image_raw_subscribed_topic /camera/depth/image_raw
* _depth_image_camera_info_subscribed_topic /camera/depth/camera_info
* _pointcloud_published_topic /pointcloud
* _virtual_scan_pointcloud_topic /laser
* _virtual_scan_topic /scan
* _publish_pointcloud 1
* _publish_laser_pointcloud 0
* _laser_beams 2047

## Examples
![alt text](http://i.imgur.com/a4774Lz.png "Laser scan overimposed xtion's pointcloud")

Laser scan overimposed xtion's pointcloud

![alt text](http://i.imgur.com/X3dYuHS.png "Any orientation is a good orientation")

Any orientation is a good orientation

![alt text](http://i.imgur.com/GHLpk0m.png "Map with gmapping")

Using gmapping and a mobile robots you can build you own maps!
