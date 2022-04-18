# pozyx_bridge

This repository aims to establish communication between a pozyx gateway with ROS. 

It has been divided in the following packages:

- `pozyx_msgs`: stores the needed messages in order to send the pozyx tags information to ROS topics.
- `pozyx_bridge`: has the code to host the connection between ROS and the pozyx gateway by retrieving the information using MQTT.
