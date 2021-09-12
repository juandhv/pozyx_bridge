# pozyx_bridge
This is a driver providing data from UWB Pozyx system. 

Quick Start   
roslaunch pozxy_bridge configure.launch  
You need to setting active tag information in pozyx_bridge_config.yaml  

Operation  
Configur information in pozyx_bridge_config.yaml  

tag_list: ["Your tag's name1","Your tag's name2"]  
<You tag's name1>：  
id: \<Your tagId check by Pozyx procol \>  
<You tag's name2>:    
id:  \<Your tagId check by Pozyx procol \> 

Published topic  
/uwb_sensor  
/tf  
