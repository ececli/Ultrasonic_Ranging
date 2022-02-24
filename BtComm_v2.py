import zmq
import bluetooth
import time

def func_determineRole(own_address,target_address):
    if own_address < target_address:
        return 1 # server
    else:
        return 2 # client


def func_sendBluetoothData(raw_bt_data,ID):
    if ID == 1:
        client_sock.send(raw_bt_data)
    else:
        bt_sock.send(raw_bt_data)


def func_recvBluetoothData(ID, length=255):
    if ID == 1:
        return client_sock.recv(length)
    else:
        return bt_sock.recv(length)
        # return raw_bt_data



address_list = ['DC:A6:32:E1:9F:C8', 'DC:A6:32:E8:BF:E0']
port = 1


own_address = bluetooth.read_local_bdaddr()[0]

for addr in address_list:
    if own_address.upper() != addr.upper():
        target_address = addr

print("Own Address is ", own_address)
print("Target Address is ",target_address)


ID = func_determineRole(own_address,target_address)
print("ID = ", ID)

bt_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)




# Connect Bluetooth

if ID == 1: # responder will bind and serve as a server
    bt_sock.bind(("", port))   
    bt_sock.listen(1)
    print("[BLUETOOTH] Binded to own device with port %d." % port)

    print('[BLUETOOTH] Waiting data from the other device')
    client_sock, address = bt_sock.accept()  
    print("[BLUETOOTH] Accepted connection from ", address)
    client_sock.setblocking(0)

else: # initiator will act as a client
    time.sleep(2)
    bt_sock.connect((target_address, port))
    print("[BLUETOOTH] Connected to the other device.")
    bt_sock.setblocking(0)





# setup ZMQ
context = zmq.Context()

publisher = context.socket(zmq.PUB)
publisher.bind("ipc:///dev/shm/Blue2Data")

subscriber = context.socket(zmq.SUB)
subscriber.connect("ipc:///dev/shm/Data2Blue")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')









while True:
    try:
        data2blue = subscriber.recv(flags=zmq.NOBLOCK)

        # print("Received data from ZMQ, ", data)
        func_sendBluetoothData(data2blue,ID)
        # print("Sent data via Bluetooth")

    except zmq.Again as e:
        #No messages waiting to be processed
        pass

    try:
        blue2data = func_recvBluetoothData(ID)
        publisher.send(blue2data)

    except bluetooth.BluetoothError as e:
        if e.errno == 11: 
            pass
        else:
            raise e
            print(e)
            break






bt_sock.close()
if ID == 1:
    client_sock.close()
print('Disconnect.')





