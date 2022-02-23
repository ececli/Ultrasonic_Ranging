import zmq
import bluetooth
import time
import numpy as np


dt_bt = np.dtype([('Flag_Valid','?'),
                ('Flag_NumSamples', '?'),
                ('Flag_LastReq','?'),
                ('NumSamples', 'i4'),
                ('NumReTransmission', "i4")
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
# port = 0x1001



bt_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
# bt_sock=bluetooth.BluetoothSocket(bluetooth.L2CAP)


time.sleep(5)
# bt_sock_T.connect((target_address, port2))
bt_sock.connect((target_address, port))
print("Connected to the other device.")
# print("Current flush timeout is {} ms.".format(bluetooth.read_flush_timeout(target_address)))
# bluetooth.set_packet_timeout( target_address, 1 )

time.sleep(1)
counter = 0
startTime = time.time()

try:
    while True:
        bt_data.Flag_NumSamples = True
        bt_data.Flag_Valid = True 
        bt_data.NumSamples = counter
        bt_data.NumReTransmission = 0
        bt_data.Flag_LastReq = False
        raw_bt_data = bt_data.tobytes()
        bt_sock.send(raw_bt_data)
        # print("Sent ",counter)
        # Received = bt_sock.recv(255)
        # print(Received)
        # bt_data_R = np.frombuffer(Received, dtype=dt_bt)
        # print(Received)
        counter = counter + 1
        time.sleep(0.5)

        if counter == 10:
            duration = time.time() - startTime
            break






except Exception as e: 
    print(e)
    bt_sock.close()
    print('Error! Disconnect!')
except KeyboardInterrupt:

    bt_sock.close()
    print('Disconnect by user.')

time.sleep(10)
print("Duration is ", duration)
bt_sock.close()

