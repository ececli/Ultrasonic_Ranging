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

else: # initiator will act as a client
    time.sleep(2)
    bt_sock.connect((target_address, port))
    print("[BLUETOOTH] Connected to the other device.")





# setup ZMQ
context = zmq.Context()

publisher = context.socket(zmq.PUB)
publisher.bind("ipc:///dev/shm/Blue2Data")

subscriber = context.socket(zmq.SUB)
subscriber.connect("ipc:///dev/shm/Data2Blue")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')









while True:


    







        if Flag_sendBluetooth:
            raw_bt_data = bt_data_prepare.tobytes()
            if ID == 1:
                client_sock.send(raw_bt_data)
            else:
                bt_sock.send(raw_bt_data)
            # print("T3-T2 has been sent: ",T3T2)
            Flag_sendBluetooth = False

        if Flag_recvBluetooth:
            if ID == 1:
                raw_bt_data = client_sock.recv(255)
            else:
                raw_bt_data = bt_sock.recv(255)
            # print("Received Bluetooth Data at ",time.time())
            bt_data = np.frombuffer(raw_bt_data, dtype=dt_bt)
            Flag_recvBluetooth = False
            Flag_recvdBluetooth = True




if ID == 2:
    # responder needs to sub and get data from data processing program and send it out via Bluetooth



    time.sleep(5)
    try:
        while True:
            T3T2 = T3T2_subscriber.recv()
            print("Received T3T2 from ZMQ, ", T3T2)
            bt_sock.send(T3T2)
            print("Sent T3T2 via Bluetooth")
    except Exception as e: 
        print(e)
        bt_sock.close()
        print('Error! Disconnect!')
    except KeyboardInterrupt:
        bt_sock.close()
        print('Disconnect by user.')



if ID == 1:
    # initiator needs to wait for bluetooth data



    try:
        while True:
            print('Waiting data from the other device')
            client_sock, address = bt_sock.accept()  
            print("Accepted connection from ", address)

            while True:
                T3T2 = client_sock.recv(255)# .decode() 
                # T3T2 = int(float(T3T2))
                print("received T3T2: ", T3T2)
                zmq_socket.send(T3T2)
                print("Sent T3T2 to ZMQ")

    except Exception as e: 
        print(e)
        client_sock.close()
        bt_sock.close()
        print('Error! Disconnect!')
    except KeyboardInterrupt:
        client_sock.close()
        bt_sock.close()
        print('Disconnect by user.')



'''
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", bluetooth.PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

bluetooth.advertise_service(server_sock, "SampleServer", service_id=uuid,
                            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                            profiles=[bluetooth.SERIAL_PORT_PROFILE],
                            # protocols=[bluetooth.OBEX_UUID]
                            )

print("Waiting for connection on RFCOMM channel", port)

client_sock, client_info = server_sock.accept()
print("Accepted connection from", client_info)

try:
    while True:
        data = client_sock.recv(1024)
        if not data:
            break
        print("Received", data)
except OSError:
    pass

print("Disconnected.")

client_sock.close()
server_sock.close()
print("All done.")
'''