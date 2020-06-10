import paho.mqtt.client as paho
import serial
import time
import threading
import matplotlib.pyplot as plt
import re
# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client
# MQTT broker hosted on local machine
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
measures = []
received_messages = []
t_data = []
X_data = []
Y_data = []
Z_data = []
def Xbee_works():
      s.write("+++".encode())
      char = s.read(2)
      print("Enter AT mode.")
      print(char.decode())

      s.write("ATMY 0x140\r\n".encode())
      char = s.read(3)
      print("Set MY 0x140.")
      print(char.decode())

      s.write("ATDL 0x240\r\n".encode())
      char = s.read(3)
      print("Set DL 0x240.")
      print(char.decode())

      s.write("ATID 0x26\r\n".encode())
      char = s.read(3)
      print("Set PAN ID 0x26.")
      print(char.decode())


      s.write("ATWR\r\n".encode())
      char = s.read(3)
      print("Write config.")
      print(char.decode())


      s.write("ATMY\r\n".encode())
      char = s.read(4)
      print("MY :")
      print(char.decode())


      s.write("ATDL\r\n".encode())
      char = s.read(4)
      print("DL : ")
      print(char.decode())


      s.write("ATCN\r\n".encode())
      char = s.read(3)
      print("Exit AT mode.")
      print(char.decode())
      
      print("start sending RPC")
      while(1):
            time.sleep(1)
            print("RPC GO~")
            s.write("/getCounts/run\r".encode())
            string = ""
            while(1):
                  try:
                        c = s.read(1).decode("UTF-8")
                  except:
                        break
                  if c=='\r':
                        break
                  string = string + c
            print(string)
            try:
                  a = int(string.replace("\n",""))
                  measures.append(a)
            except:
                  measures.append(0)

t = threading.Thread(target = Xbee_works)

t.start()
##################################### MATPLOTLIB ##########################################
def graph_work():
      print("graph pending")
      time.sleep(20)
      print(measures)
      for i in received_messages:
            k = i.split(" ")
            print(k)
            t_data.append(int(k[1]))
            X_data.append(float(k[4]))
            Y_data.append(float(k[6]))
            Z_data.append(float(k[8]))
      plt.subplot(2,1,1)
      plt.plot(t_data, X_data, 'r',label = 'X',marker = 'o') # plotting t, a separately 
      plt.plot(t_data, Y_data, 'b',label = 'Y',marker = 'o') # plotting t, b separately 
      plt.plot(t_data, Z_data, 'g',label = 'Z',marker = 'o') # plotting t, c separately
      plt.legend(loc = 'upper right')
      plt.subplot(2,1,2)
      plt.plot(range(0,len(measures),1),measures)
      plt.show()


g = threading.Thread(target = graph_work)

g.start()


##################################### MQTT #################################################
mqttc = paho.Client()
# Settings for connection
# TODO: revise host to your ip

host = "192.168.43.189"
topic = "Mbed"

# Callbacks
def on_connect(self, mosq, obj, rc):
      print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):

      print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
      received_messages.append(msg.payload.decode("UTF-8"))

def on_subscribe(mosq, obj, mid, granted_qos):
      print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
      print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

# Publish messages from Python
num = 0

# Loop forever, receiving messages
time.sleep(5)
print("MQTT ONlINE")
mqttc.loop_forever()

########################### XBEE CODES ###############################################


