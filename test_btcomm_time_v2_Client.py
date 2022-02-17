import zmq
import bluetooth
import time
import numpy as np


dt_bt = np.dtype([('status', 'i4'),
               ('NumSamples', 'i4')
               ])

bt_data = np.rec.array(np.zeros(1, dtype=dt_bt))

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

        bt_data.status = 0 
        bt_data.NumSamples = counter
        raw_bt_data = bt_data.tobytes()
        bt_sock.send(raw_bt_data)
        print("Sent ",counter)
        # Received = bt_sock.recv(255).decode() 
        # print(Received)
        counter = counter + 1
        time.sleep(0.5)

        if counter == 100:
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

