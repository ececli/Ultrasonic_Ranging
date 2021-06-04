import paho.mqtt.client as mqtt
from queue import Queue
import time
import numpy as np

# Use MQTT to transfer data among Raspberry Pi's which are under the same network. 
class myMQTT:

    def __init__(self, address, clientName={}):
        self.q = {} # setup a queue dictionary
        self.qcounter = 0 
        self.clientName = clientName
        self.broker_address = address
        self.client = mqtt.Client(clientName) # create an MQTT instance
        self.client.connected_flag = False
        self.client.on_connect = self.on_connect # bind callback function
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect
        try: 
            self.client.connect(address, port = 1883, keepalive = 60)
        except:
            print("Connected Failed")
            exit(1)

        self.client.loop_start() # this will let the client run in a new thread
        while not self.client.connected_flag:
            print("Connecting...")
            time.sleep(0.5)
        

    def on_connect(self, client, userdata, flags, rc): # callback function
        if rc == 0: # rc: return code
            self.client.connected_flag = True
            print("Connected Successfully")
        else:
            print("Connected Failed with return code ", rc)
    
    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed Successfully")

    def on_message(self, client, userdata, msg): 
        # this callback function handle how to store/use the received data
        message = str(msg.payload.decode("utf-8"))
        if msg.topic in self.q.keys():
            self.q[msg.topic].put(message)

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected Successfully")


    def registerTopic(self, topic, qos = 0):
        # when subscribe a topic, generate a new queue
        self.client.subscribe(topic, qos)
        self.q[topic]= Queue()
        self.qcounter = self.qcounter + 1
    
    def readBrokerAddress(self):
        return self.broker_address
    
    def checkTopicDataLength(self, topic):
        if topic in self.q.keys():
            return self.q[topic].qsize()
        else:
            return -1 # if no such topic, return -1

    def getQData(self, q): 
        # pull data from queue
        data = []
        while not q.empty():
            temp = q.get()
            data.append(int(temp))
        return data

    def readTopicData(self, topic):
        if topic in self.q.keys():
            return self.getQData(self.q[topic])
        else:
            return -1 # if no such topic, return -1
    
    def sendMsg(self, topic, msg):
        # send the data out. 
        # msg could be an array, but currently only integer will be sent
        # I may change this in future to support float and string
        if isinstance(msg,list) or isinstance(msg,np.ndarray):
            for k in msg:
                self.client.publish(topic, int(k))
        else:
            self.client.publish(topic, int(msg))
    
    def closeClient(self):
        self.client.loop_stop()
        self.client.disconnect()
