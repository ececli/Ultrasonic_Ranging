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

port = 1


# bt_sock_T=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
bt_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)


bt_sock.bind(("", port))   
bt_sock.listen(1)
print("Binded to own device with port %d." % port)


try:
    while True:
        print('Waiting data from the other device')
        client_sock, address = bt_sock.accept()  
        print("Accepted connection from ", address)


        counter = 0
        while True:
            T3T2 = client_sock.recv(255).decode() 
            print("received T3T2: %s at %d" % (T3T2,counter))

            # client_sock.send(str(T3T2).encode())
            counter = counter + 1
            time.sleep(1)



except Exception as e: 
    print(e)
    client_sock.close()
    bt_sock.close()

    print('Error! Disconnect!')
except KeyboardInterrupt:
    client_sock.close()
    bt_sock.close()
    print('Disconnect by user.')
