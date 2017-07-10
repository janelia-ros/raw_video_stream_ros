# raw_video_stream

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
ROS_NAMESPACE=camera roslaunch raw_video_stream raw_video_file.launch manager:=camera_nodelet_manager video_stream_provider:=/home/polidorop/zebrafish_tracker/Videos/dark3_uint8_1024x1200_2500frames.raw width:=1024 height:=1200 frame_count:=2500 fps:=30
```

```shell
rosrun image_view image_view image:=/camera/videofile/image_raw
```
