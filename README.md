# pozyx_bridge
This ROS package aims to provide a connection from the UWB Pozyx system to a ROS server. This package contains a node to connect via MQTT to the Pozyx gateway and then publish the transform of the tags' and their pose in a topic.

## Dependencies

Run the following command to install paho.mqtt library:

````shell
sudo pip install paho.mqtt
````

## Quick Start 

First you need to either modify the config file in the `config` 

roslaunch pozxy_bridge configure.launch  

## Operation  
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
/pozyx_bridge  


## API reference
