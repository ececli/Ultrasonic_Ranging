import paho.mqtt.client as mqtt
from queue import Queue
import time

# http://www.steves-internet-guide.com/mqtt-python-beginners-course/
# https://appcodelabs.com/introduction-to-iot-build-an-mqtt-server-using-raspberry-pi

class myMQTT:

    def __init__(self, address, clientName={}):
        self.q = {}
        self.qcounter = 0
        self.clientName = clientName
        self.broker_address = address
        self.client = mqtt.Client(clientName) 
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

        self.client.loop_start()
        while not self.client.connected_flag:
            print("Connecting...")
            time.sleep(0.5)
        

    def on_connect(self, client, userdata, flags, rc): # callback function
        # self.client.subscribe([(myMQTT.topic1, 0),(myMQTT.topic2, 0)])
        if rc == 0:
            self.client.connected_flag = True
            print("Connected Successfully")
        else:
            print("Connected Failed with return code ", rc)
    
    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed Successfully")

    def on_message(self, client, userdata, msg):
        message = str(msg.payload.decode("utf-8"))
        if msg.topic in self.q.keys():
            self.q[msg.topic].put(message)

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected Successfully")


    def registerTopic(self, topic, qos = 0):
        self.client.subscribe(topic, qos)
        self.q[topic]= Queue()
        self.qcounter = self.qcounter + 1
    
    def readBrokerAddress(self):
        return self.broker_address
    
    def checkTopicDataLength(self, topic):
        if topic in self.q.keys():
            return self.q[topic].qsize()
        else:
            return -1

    def getQData(self, q):
        data = []
        while not q.empty():
            temp = q.get()
            data.append(int(temp))
        return data

    def readTopicData(self, topic):
        if topic in self.q.keys():
            return self.getQData(self.q[topic])
        else:
            return -1
    
    def sendMsg(self, topic, msg):
        for k in msg:
            self.client.publish(topic, msg)

    def closeClient(self):
        self.client.loop_stop()
        self.client.disconnect()

