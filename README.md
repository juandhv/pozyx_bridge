# pozyx_bridge
This ROS package aims to provide a connection from the UWB Pozyx system to a ROS server. This package contains a node to connect via MQTT to the Pozyx gateway and then publish the transform of the tags' and their pose in a topic.

## Dependencies

Run the following command in a shell prompt to install paho.mqtt library:

````shell
sudo pip install paho.mqtt
````

## Quick Start

First you need to either modify the config file `pozyx_bridge_config.yaml` in the `config` folder or create a new file.

Then change the config file path in `pozyx_bridge_demo.launch`.

Finally run:

```` bash
roslaunch pozxy_bridge pozyx_bridge_demo.launch  
````

## Configuration Parameters

* `frame_id`: this is the parent frame used for the transform of all the defined tags.
* `frequency`: this is the frequency of publish of the transform and position of the tags. This frequency considers the rate of transferred data for each tag individually. It means that if a frequency of 100hz is defined with 4 tags, the rate of info for each tag is about 25hz.
* `tag_list`: list of tag names as strings.
* `id`: tag id configured previously, defined as a number.


## API Reference

### Published Topics

#### uwb_sensor (geometry_msgs/TransformStamped)
Position and orientation of the tags. All the tags are published individually.

<!-- ## Operation  
Configur information in pozyx_bridge/launch/config/pozyx_bridge_config.yaml  

tag_list: ["Your tag's name1","Your tag's name2"]  
<You tag's name1>：  
id: \<Your tagId check by Pozyx procol \>  
<You tag's name2>:    
id:  \<Your tagId check by Pozyx procol \> 

Published topic:  
/uwb_sensor  
/tf  

Node:  
/pozyx_bridge   -->


