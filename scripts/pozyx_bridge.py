#!/usr/bin/env python
# license removed for brevity
import rospy
import tf2_ros
import json
import paho.mqtt.client as mqtt
import time

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

HOST = '10.0.0.254'
PORT = 1883
TOPIC = 'tags'
DURATION = 500


class PozyxBridge(object):

    def __init__(self):
        # Flags
        self.is_data_available = False  # Set a Flag when useful data come system start record
        # Setting initial position
        self.tagdic = {}  # Dictionary to store active tag in POZYX system
        self.taglist = rospy.get_param("/tag_list")  # get each tag's name from yaml file
        self.paramdic = {}  # Dictionary for tags record in yaml file
        self.tempdic = {}  # Dictionary to store last position of tag
        for i in self.taglist:  # Load all information from tag into paramdic and tempdic
            self.paramdic[rospy.get_param("/" + i + "/id")] = i
            self.tempdic[rospy.get_param("/" + i + "/id")] = i
        rospy.loginfo("These tag are in tag list %s", self.paramdic)
        # Publisher use MQTT clint
        self.client = mqtt.Client()  # create new instance
        self.pub = rospy.Publisher('uwb_sensor', TransformStamped, queue_size=10)  # Set up a ros publisher
        self.timer = rospy.Timer(rospy.Duration(0.25), self.time_record)  # Set up a ros timer inorder to control
        # frequency of publisher
        self.br = tf2_ros.TransformBroadcaster()  # Set up a TF broadcaster

    def time_record(self, event):
        if not self.is_data_available is True:
            return
        for i in self.tagdic.keys():  # Control frequency of publisher
            self.tagdic[i].header.stamp = rospy.Time.now()
            self.pub.publish(self.tagdic[i])
            self.br.sendTransform(self.tagdic[i])

    # MQTT client setup
    def setup_client(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message  # attach function to callback
        self.client.on_subscribe = self.on_subscribe
        self.client.connect(HOST, port=PORT)  # connect to host
        self.client.subscribe(TOPIC)  # subscribe to topic

    def run(self):
        self.client.loop_start()  # start the loop
        # rate = rospy.Rate(10)  # 10hz
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rate.sleep()

    def on_message(self, client, userdata, message):
        datapack = json.loads(message.payload.decode())  # load MQTT data package from JSON type
        Id = int(datapack[0]['tagId'])

        if Id not in self.paramdic.keys(): # Give an error if the tag is not define in yaml file
            rospy.logwarn("Active tag %s is not define in Parameter Id!", Id)
            for i in self.paramdic.keys():
                if i not in self.tagdic.keys(): # Give a warn that the tag information is incorrect
                    rospy.logwarn("Parameter Id %s are not active", i)
            return
        if Id not in self.tagdic.keys():
            rospy.loginfo("Tag %s is now active", Id)
            transform_msg = TransformStamped()  # Set up a new TransformStamped for each coming data pack
            transform_msg.header.frame_id = rospy.get_param("/frame_id")
            self.tagdic[Id] = transform_msg # Let id be the key and transform_msg be value in tagdic
            temtag = self.paramdic[Id]
            self.tagdic[Id].child_frame_id = "/pozyx" + temtag  # Collect data from
            self.tagdic[Id].transform.rotation.w = 1
            # yaml file and sett up it into tagdic
            self.tempdic[Id] = {'x': 0, 'y': 0, 'z': 0, 'quaternion': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}

        else:

            try:
                x = datapack[0]['data']['coordinates']['x'] / 1000.0                # Collect data from sensor
                y = datapack[0]['data']['coordinates']['y'] / 1000.0
                z = datapack[0]['data']['coordinates']['z'] / 1000.0

                self.tempdic[Id]['x'] = x           # Store available data in tempdic
                self.tempdic[Id]['y'] = y
                self.tempdic[Id]['z'] = z

                self.tagdic[Id].transform.translation.x = x
                self.tagdic[Id].transform.translation.y = y
                self.tagdic[Id].transform.translation.z = z

                if not self.is_data_available:
                    self.is_data_available = True

                if 'quaternion' in datapack[0]['data']['tagData'].keys():   # Master tag will collect oration data
                    quaternion = datapack[0]['data']['tagData']['quaternion']
                    self.tempdic[Id]['quaternion'] = quaternion

                    self.tagdic[Id].transform.rotation.x = quaternion['z']
                    self.tagdic[Id].transform.rotation.y = quaternion['y']
                    self.tagdic[Id].transform.rotation.z = quaternion['x']
                    self.tagdic[Id].transform.rotation.w = quaternion['w']

            except:
                self.tagdic[Id].transform.translation.x = self.tempdic[Id]['x'] # If no available data come the dictionary will use data in tempdic
                self.tagdic[Id].transform.translation.y = self.tempdic[Id]['y']
                self.tagdic[Id].transform.translation.z = self.tempdic[Id]['z']
                print(self.tempdic[Id]['quaternion'])
                self.tagdic[Id].transform.rotation.x = self.tempdic[Id]['quaternion']['z']
                self.tagdic[Id].transform.rotation.y = self.tempdic[Id]['quaternion']['y']
                self.tagdic[Id].transform.rotation.z = self.tempdic[Id]['quaternion']['x']
                self.tagdic[Id].transform.rotation.w = self.tempdic[Id]['quaternion']['w']

        for i in self.paramdic.keys():    # Double check
            if i not in self.tagdic.keys():
                rospy.logwarn("Parameter Id %s are not active", i)

    def on_connect(self, client, userdata, flags, rc):
        print(mqtt.connack_string(rc))

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed to topic!")


if __name__ == '__main__':
    rospy.init_node('pozyx_bridge')
    try:
        pozyx_bridge = PozyxBridge()
        pozyx_bridge.setup_client()
        pozyx_bridge.run()

    except rospy.ROSInterruptException:
        pass
