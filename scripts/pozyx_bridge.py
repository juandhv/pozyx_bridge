#!/usr/bin/env python

import json
import rospy
import tf2_ros
import paho.mqtt.client as mqtt
from pozyx_bridge.msg import UwbTransformStamped, UwbTransformStampedArray

HOST = "10.0.0.254"
PORT = 1883
TOPIC = "tags"

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
        self.is_data_available = False
        self.tagdic = {}  # tags information
        self.taglist = rospy.get_param("/tag_list")  # tag's names
        self.paramdic = {}
        self.tempdic = {}  # last tags data
        for i in self.taglist:
            self.paramdic[rospy.get_param("/" + i + "/id")] = i
            self.tempdic[rospy.get_param("/" + i + "/id")] = i
        rospy.loginfo("These tag are in tag list %s", self.paramdic)

        self.client = mqtt.Client()
        self.pub = rospy.Publisher("uwb_sensor", UwbTransformStampedArray, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1 / float(rospy.get_param("/frequency", 50))),
            self.time_record,
        )  # periodic function execution
        self._br = tf2_ros.TransformBroadcaster()

    def time_record(self, event):
        """publishes data and transform of tags"""
        if not self.is_data_available is True:
            return

        transform_stamped_array = []
        for i in self.tagdic.items():
            i[1].header.stamp = rospy.Time.now()
            transform_stamped_array.append(i[1])
            self._br.sendTransform(i[1].transform)

        transform_stamped_array_msg = UwbTransformStampedArray()
        transform_stamped_array_msg.transforms_array = transform_stamped_array
        self.pub.publish(transform_stamped_array_msg)

    # MQTT client setup
    def setup_client(self):
        """connects MQTT broker"""
        self.client.on_connect = on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = on_subscribe
        self.client.connect(HOST, port=PORT)
        self.client.subscribe(TOPIC)

    def run(self):
        """loops and mantains the node"""
        self.client.loop_start()
        rospy.spin()

    def on_message(self, client, userdata, message):
        """process to run when message arrives"""
        datapack = json.loads(message.payload.decode())
        _id = int(datapack[0]["tagId"])

        if _id not in self.paramdic.keys():
            rospy.logwarn("Active tag %s is not define in Parameter Id!", _id)
            for i in self.paramdic:
                if i not in self.tagdic:
                    rospy.logwarn("Parameter Id %s are not active", i)
            return
        if _id not in self.tagdic:
            rospy.loginfo("Tag %s is now active", _id)
            transform_msg = UwbTransformStamped()
            transform_msg.transform.header.frame_id = rospy.get_param("/frame_id")
            self.tagdic[_id] = transform_msg
            temtag = self.paramdic[_id]
            self.tagdic[_id].transform.child_frame_id = "/pozyx" + temtag
            self.tagdic[_id].transform.transform.rotation.w = 1
            self.tempdic[_id] = {
                "x": 0,
                "y": 0,
                "z": 0,
                "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }

        else:
            try:
                # values in meters
                _x = datapack[0]["data"]["coordinates"]["x"] / 1000.0
                _y = datapack[0]["data"]["coordinates"]["y"] / 1000.0
                _z = datapack[0]["data"]["coordinates"]["z"] / 1000.0

                self.tempdic[_id]["x"] = _x
                self.tempdic[_id]["y"] = _y
                self.tempdic[_id]["z"] = _z

                self.tagdic[_id].transform.transform.translation.x = _x
                self.tagdic[_id].transform.transform.translation.y = _y
                self.tagdic[_id].transform.transform.translation.z = _z

                if not self.is_data_available:
                    self.is_data_available = True

                if "quaternion" in datapack[0]["data"]["tagData"].keys():
                    quaternion = datapack[0]["data"]["tagData"]["quaternion"]
                    self.tempdic[_id]["quaternion"] = quaternion

                    self.tagdic[_id].transform.transform.rotation.x = quaternion["z"]
                    self.tagdic[_id].transform.transform.rotation.y = quaternion["y"]
                    self.tagdic[_id].transform.transform.rotation.z = quaternion["x"]
                    self.tagdic[_id].transform.transform.rotation.w = quaternion["w"]

            except:
                # use of last available data
                self.tagdic[_id].transform.transform.translation.x = self.tempdic[_id]["x"]
                self.tagdic[_id].transform.transform.translation.y = self.tempdic[_id]["y"]
                self.tagdic[_id].transform.transform.translation.z = self.tempdic[_id]["z"]
                # print(self.tempdic[_id]["quaternion"])
                self.tagdic[_id].transform.transform.rotation.x = self.tempdic[_id]["quaternion"][
                    "z"
                ]
                self.tagdic[_id].transform.transform.rotation.y = self.tempdic[_id]["quaternion"][
                    "y"
                ]
                self.tagdic[_id].transform.transform.rotation.z = self.tempdic[_id]["quaternion"][
                    "x"
                ]
                self.tagdic[_id].transform.transform.rotation.w = self.tempdic[_id]["quaternion"][
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
