✔️ What you told me (the original request)

You said:

“ok lets create a new launch, called vision_launch that include these 3 calls:

ros2 run vision vision_node --ros-args -p image_topic:=/kinect/rgb/image_raw

ros2 run vision yolo_server --ros-args -p image_topic:=/kinect/rgb/image_raw -p cloud_topic:=/kinect/depth/points

ros2 launch robotino_webots sim_teleop_joy.launch.py”

That’s exactly what the file above reproduces.