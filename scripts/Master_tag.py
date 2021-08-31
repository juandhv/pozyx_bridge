import json

import paho.mqtt.client as mqtt
import time
import sys
import json

HOST = '10.0.0.254'
PORT = 1883
TOPIC = 'tags'
DURATION = 300

def on_message(client, userdata, message):
  datpack = json.loads(message.payload.decode())  # 'utf-8'
  # try:
  if 'quaternion' not in datpack[0]['data']['tagData'].keys():
      print('A')
  # except:
  else:
    print(datpack[0]['data']['coordinates']['x'])

client = mqtt.Client()  # create new instance
client.connect(HOST, port=PORT)  # connect to host
client.on_message = on_message  # attach function to callback
client.loop_start()  # start the loop
client.subscribe(TOPIC)  # subscribe to topic
time.sleep(DURATION)  # wait for duration seconds
client.disconnect()  # disconnect