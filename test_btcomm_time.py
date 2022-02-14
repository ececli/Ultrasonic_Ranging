import zmq
import bluetooth
import time


T3T2 = 2234.0

address_list = ['DC:A6:32:E1:9F:C8', 'DC:A6:32:E8:BF:E0']

own_address = bluetooth.read_local_bdaddr()[0]

for addr in address_list:
    if own_address.upper() != addr.upper():
        target_address = addr

print("Own Address is ", own_address)
print("Target Address is ",target_address)



if own_address[-1] == '0':
    print("I am into 1")
    port1 = 1
    port2 = 2
else:
    print("I am into 2")
    port1 = 2
    port2 = 1



bt_sock_T=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
bt_sock_R=bluetooth.BluetoothSocket(bluetooth.RFCOMM)


bt_sock_R.bind(("", port1))   
bt_sock_R.listen(1)
print("Binded to own device with port %d." % port1)
time.sleep(5)
bt_sock_T.connect((target_address, port2))
print("Connected to the other device.")



try:
    while True:
        print('Waiting data from the other device')
        client_sock, address = bt_sock_R.accept()  
        print("Accepted connection from ", address)

        if own_address[-1] == 0:
            time.sleep(1)
            bt_sock_T.send(str(T3T2).encode())


        counter = 0
        while True:
            T3T2 = client_sock.recv(255).decode() 
            print("received T3T2: %s at %d" % (T3T2,counter))
            if counter == 0:
                time.sleep(1)
            bt_sock.send(str(T3T2).encode())
            counter = counter + 1



except Exception as e: 
    print(e)
    client_sock.close()
    bt_sock_1.close()
    bt_sock_2.close()
    print('Error! Disconnect!')
except KeyboardInterrupt:
    client_sock.close()
    bt_sock_1.close()
    bt_sock_2.close()
    print('Disconnect by user.')   



