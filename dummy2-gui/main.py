import socket
from collections import deque
import threading
import time

class UDPClient:
    def __init__(self, server_address, server_port, buffer_size=1024):
        self.server_address = server_address
        self.server_port = server_port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connected = False
        self.buffer = deque(maxlen=buffer_size)
        self.receive_thread = None

    def connect(self):
        try:
            self.server_address_port = (self.server_address, self.server_port)
            self.connected = True
            return True
        except Exception as e:
            print(f'Error: {e}')
            self.connected = False
            return False

    def send_message(self, a):
        message = bytearray(12)  # Total size of the message
        if a == "enable":
            message[0] = 0xbb  # message.Head
            message[1] = 0x02
            message[2] = 7#command id
            #3 4 5 6 7 8 9 10 message
            #0 1 2 3 4 5 6 7  rdive data
            message[3] = 4#byte 0 3 is 0
        else:
            message[0] = 0xbb  # message.Head
            message[1] = 0x02
            message[2] = 0x0c
            message[3] = 50
            message[7] = 20
            message[9] = 0
        
        # Fill in the rest of the message bytes as needed
        message[11] = 0xcc  # message.Tail        
        if self.connected:            
            try:
                self.client_socket.sendto(message, self.server_address_port)
                print('Message sent successfully.')
            except Exception as e:
                print(f'Error: {e}')
        else:
            print('Not connected to the server.')

    def receive_messages(self):
        if self.connected:
            while True:
                try:
                    data, address = self.client_socket.recvfrom(1024)
                    if data:
                        hex_data = ' '.join(f'{b:02X}' for b in data)
                        self.buffer.append(hex_data)
                        # print(f'Received message from {address}: {hex_data}')
                except (KeyboardInterrupt, SystemExit):
                    # Handle keyboard interrupt (Ctrl+C) and system exit
                    break
                except Exception as e:
                    print(f'Error: {e}')
        else:
            print('Not connected to the server.')

    def start_receive_thread(self):
        self.receive_thread = threading.Thread(target=self.receive_messages)
        self.receive_thread.start()

    def get_buffer_data(self):
        if self.buffer:
            return self.buffer.popleft()
        else:
            return None

    def close(self):
        if self.receive_thread:
            self.receive_thread.join()
        self.client_socket.close()

# Usage
client = UDPClient('192.168.0.88', 9999)

if client.connect():
    print('Connected to the server.')
    client.send_message("disable")
    client.start_receive_thread()  # Start receiving messages in a separate thread

    # Get data from the buffer one by one
    while True:
        data = client.get_buffer_data()
        if data is None:
            # print('Buffer is empty.')
            time.sleep(0.01)
            continue
            # break
        else:
            print(f'Buffer data: {data}')

else:
    print('Failed to connect to the server.')

client.close()