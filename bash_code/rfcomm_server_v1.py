import bluetooth
import time

server_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 1
server_sock.bind(("", port))   
server_sock.listen(1)

try:
    while True:
        print('Waiting data from the other device')
        client_sock,address=server_sock.accept()  
        print("Accepted connection from ", address)
        time.sleep(2)
        while True:
            data =client_sock.recv(1024).decode()
            print("received ", data, type(data))
            if int(data) == 5:
                break
except Exception as e: 
    print(e)
    client_sock.close()
    server_sock.close()
    print('disconnect!')


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