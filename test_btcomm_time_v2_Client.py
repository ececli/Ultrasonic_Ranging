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


time.sleep(5)
# bt_sock_T.connect((target_address, port2))
bt_sock.connect((target_address, port))
print("Connected to the other device.")
time.sleep(1)
counter = 0
startTime = time.time()

try:
    while True:

            
            # bt_sock_T.send(str(T3T2).encode())
        bt_sock.send(str(T3T2).encode())
        Received = bt_sock.recv(255).decode() 
        # print(Received)
        counter = counter + 1

        if counter == 10000:
            duration = time.time() - startTime
            break






except Exception as e: 
    print(e)
    bt_sock.close()
    print('Error! Disconnect!')
except KeyboardInterrupt:

    bt_sock.close()
    print('Disconnect by user.')   
print("Duration is ", duration)
bt_sock.close()

