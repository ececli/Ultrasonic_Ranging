import zmq
import bluetooth
import time



address_list = ['DC:A6:32:E1:9F:C8', 'DC:A6:32:E8:BF:E0']

own_address = bluetooth.read_local_bdaddr()[0]

for addr in address_list:
    if own_address.upper() != addr.upper():
        target_address = addr

print("Own Address is ", own_address)
print("Target Address is ",target_address)

bt_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 1

context = zmq.Context()
controlData_client = context.socket(zmq.REQ)
controlData_client.connect("ipc:///dev/shm/control_data")
controlData_client.send_string("Inquiry Roll")
ID = controlData_client.recv_pyobj()
print("ID of this device is ", ID)



if ID == 2:
    # responder needs to sub and get data from data processing program and send it out via Bluetooth


    T3T2_subscriber = context.socket(zmq.SUB)
    T3T2_subscriber.connect("ipc:///dev/shm/T3T2_data")
    T3T2_subscriber.setsockopt(zmq.SUBSCRIBE, b'')
    time.sleep(5)
    try:
        bt_sock.connect((target_address, port))
        print("Connected to target Device.")
        while True:
            T3T2 = T3T2_subscriber.recv_pyobj()
            print("Received T3T2 from ZMQ, ", T3T2)
            bt_sock.send(str(T3T2).encode())
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

    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("ipc:///dev/shm/T3T2_data")

    bt_sock.bind(("", port))   
    bt_sock.listen(1)

    try:
        while True:
            print('Waiting data from the other device')
            client_sock, address = bt_sock.accept()  
            print("Accepted connection from ", address)

            while True:
                T3T2 = client_sock.recv(255).decode() 
                T3T2 = int(float(T3T2))
                print("received T3T2: %d" % T3T2)
                zmq_socket.send_pyobj(T3T2)
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