#!/usr/bin/env python

import json
import rospy
import tf2_ros
import paho.mqtt.client as mqtt
from geometry_msgs.msg import TransformStamped

HOST = "10.0.0.254"
PORT = 1883
TOPIC = "tags"
DURATION = 500


def on_subscribe(client, userdata, mid, granted_qos):
    """process to run when clients subscribes successfully to a topic"""
    print("Subscribed to topic!")


def on_connect(client, userdata, flags, rc):
    """process to run when clients subscribes successfully to the broker server"""
    print(mqtt.connack_string(rc))


class PozyxBridge(object):
    """Connects Pozyx Gateway with ROS"""

    def __init__(self):
        # Flags
        self.is_data_available = (
            False  # Set a Flag when useful data come system start record
        )
        # Setting initial position
        self.tagdic = {}  # Dictionary to store active tag in POZYX system
        self.taglist = rospy.get_param(
            "/tag_list"
        )  # get each tag's name from yaml file
        self.paramdic = {}  # Dictionary for tags record in yaml file
        self.tempdic = {}  # Dictionary to store last position of tag
        for i in self.taglist:
            self.paramdic[rospy.get_param("/" + i + "/id")] = i
            self.tempdic[rospy.get_param("/" + i + "/id")] = i
        rospy.loginfo("These tag are in tag list %s", self.paramdic)
        # Publisher use MQTT client
        self.client = mqtt.Client()  # create new instance
        self.pub = rospy.Publisher(
            "uwb_sensor", TransformStamped, queue_size=10
        )  # Set up a ros publisher
        self.timer = rospy.Timer(
            rospy.Duration(0.25), self.time_record
        )  # Set up a ros timer inorder to control
        # frequency of publisher
        self._br = tf2_ros.TransformBroadcaster()  # Set up a TF broadcaster

    def time_record(self, event):
        """publishes data and transform of tags"""
        if not self.is_data_available is True:
            return
        for i in self.tagdic.items():  # Control frequency of publisher
            i[1].header.stamp = rospy.Time.now()
            self.pub.publish(i[1])
            self._br.sendTransform(i[1])

    # MQTT client setup
    def setup_client(self):
        """connects MQTT broker"""
        self.client.on_connect = on_connect
        self.client.on_message = self.on_message  # attach function to callback
        self.client.on_subscribe = on_subscribe
        self.client.connect(HOST, port=PORT)  # connect to host
        self.client.subscribe(TOPIC)  # subscribe to topic

    def run(self):
        """loops and mantains the node"""
        self.client.loop_start()  # start the loop
        # rate = rospy.Rate(10)  # 10hz
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rate.sleep()

    def on_message(self, client, userdata, message):
        """process to run when message arrives"""
        datapack = json.loads(
            message.payload.decode()
        )  # load MQTT data package from JSON type
        _id = int(datapack[0]["tagId"])

        if _id not in self.paramdic.keys():
            rospy.logwarn("Active tag %s is not define in Parameter Id!", _id)
            for i in self.paramdic:
                if i not in self.tagdic:
                    rospy.logwarn("Parameter Id %s are not active", i)
            return
        if _id not in self.tagdic:
            rospy.loginfo("Tag %s is now active", _id)
            transform_msg = (
                TransformStamped()
            )  # Set up a new TransformStamped for each coming data pack
            transform_msg.header.frame_id = rospy.get_param("/frame_id")
            self.tagdic[
                _id
            ] = transform_msg  # Let id be the key and transform_msg be value in tagdic
            temtag = self.paramdic[_id]
            self.tagdic[_id].child_frame_id = "/pozyx" + temtag  # Collect data from
            self.tagdic[_id].transform.rotation.w = 1
            # yaml file and sett up it into tagdic
            self.tempdic[_id] = {
                "x": 0,
                "y": 0,
                "z": 0,
                "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }

        else:

            try:
                _x = (
                    datapack[0]["data"]["coordinates"]["x"] / 1000.0
                )  # Collect data from sensor
                _y = datapack[0]["data"]["coordinates"]["y"] / 1000.0
                _z = datapack[0]["data"]["coordinates"]["z"] / 1000.0

                self.tempdic[_id]["x"] = _x  # Store available data in tempdic
                self.tempdic[_id]["y"] = _y
                self.tempdic[_id]["z"] = _z

                self.tagdic[_id].transform.translation.x = _x
                self.tagdic[_id].transform.translation.y = _y
                self.tagdic[_id].transform.translation.z = _z

                if not self.is_data_available:
                    self.is_data_available = True

                if "quaternion" in datapack[0]["data"]["tagData"].keys():
                    quaternion = datapack[0]["data"]["tagData"]["quaternion"]
                    self.tempdic[_id]["quaternion"] = quaternion

                    self.tagdic[_id].transform.rotation.x = quaternion["z"]
                    self.tagdic[_id].transform.rotation.y = quaternion["y"]
                    self.tagdic[_id].transform.rotation.z = quaternion["x"]
                    self.tagdic[_id].transform.rotation.w = quaternion["w"]

            except:
                self.tagdic[_id].transform.translation.x = self.tempdic[_id][
                    "x"
                ]  # If no available data come the dictionary will use data in tempdic
                self.tagdic[_id].transform.translation.y = self.tempdic[_id]["y"]
                self.tagdic[_id].transform.translation.z = self.tempdic[_id]["z"]
                print(self.tempdic[_id]["quaternion"])
                self.tagdic[_id].transform.rotation.x = self.tempdic[_id]["quaternion"][
                    "z"
                ]
                self.tagdic[_id].transform.rotation.y = self.tempdic[_id]["quaternion"][
                    "y"
                ]
                self.tagdic[_id].transform.rotation.z = self.tempdic[_id]["quaternion"][
                    "x"
                ]
                self.tagdic[_id].transform.rotation.w = self.tempdic[_id]["quaternion"][
                    "w"
                ]

        for i in self.paramdic:
            if i not in self.tagdic:
                rospy.logwarn("Parameter Id %s are not active", i)


if __name__ == "__main__":
    rospy.init_node("pozyx_bridge")
    try:
        pozyx_bridge = PozyxBridge()
        pozyx_bridge.setup_client()
        pozyx_bridge.run()

    except rospy.ROSInterruptException:
        pass
