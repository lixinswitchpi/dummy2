import socket
from collections import deque
import threading
import struct

can_lock = threading.Lock()

Message_type = {
    "short": 0,
    "full": 1,
}

Command_id = {
    "Get_Version": 0,
    "Heartbeat": 1,
    "Set_Axis_State": 7,
    "Get_Encoder_Estimates": 9,
    "Set_Input_Pos": 12, #0x0c
    "Get_Iq": 20,#0x14
    "Get_Temperature": 21, #0x15
    "Reboot": 22, #0x16
    "Get_Bus_Voltage_Current": 23,#0x17
}

AxisState = {
    "IDLE": 1,
    "CLOSED_LOOP_CONTROL": 8, 
}

Reboot = {
    "Reboot": 0,
    "Save_configs": 1, 
    "Erase_configs": 2, 
    "Enter_dfu": 3, 
}

can_message = {
    "header": b'\xaa',
    "id": b'\x00',
    "type": b'\x00',
    "body": bytes([0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]),
    "tail": b'\x88',
}

motors_cfg = {'M1': 
                {'position': 0.0, 
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 30,
                'teaching': 0,
                'ccw': 0,
                },
                'M2':
                {'position': 0.0,
                'velocity': 0.0,                   
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 0,
                },
                'M3':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 1,
                },
                'M4':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 30,
                'teaching': 0,
                'ccw': 0,
                },
                'M5':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 30,
                'teaching': 0,
                'ccw': 1,
                },
                'M6':
                {'position': 0.0,
                'velocity': 0.0,
                'status': 0,
                'error': 0,
                'iq': 0.0,
                'voltage': 0.0,
                'ibus': 0.0,
                'reduction': 50,
                'teaching': 0,
                'ccw': 0,
                },
                }

def pack_can_message(bytes_array):
    format_string = "<BBB8sB"  # "<" for little-endian byte order, "B" for unsigned char (1 byte)
    # Pack bytes_array into struct
    packed_data = struct.pack(format_string, bytes_array[0], bytes_array[1], bytes_array[2], bytes_array[3:11], bytes_array[11])
    unpacked_data = struct.unpack(format_string, packed_data)
    with can_lock:
        can_message.update(header = unpacked_data[0])
        can_message.update(id = unpacked_data[1])
        can_message.update(type = unpacked_data[2])
        can_message.update(body = unpacked_data[3])
        can_message.update(tail = unpacked_data[4])
    return can_message
    # print(can_message)
    
class UDPClient:
    def __init__(self, server_address, server_port, buffer_size=512):
        self.server_address = server_address
        self.server_port = server_port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connected = False
        self.buffer = deque(maxlen=buffer_size)
        self.receive_thread = None
        self.callback = None

    def register_callback(self, callback):
        self.callback = callback

    def unregister_callback(self):
        self.callback = None     

    def connect(self):
        try:
            self.server_address_port = (self.server_address, self.server_port)
            self.connected = True
            return True
        except Exception as e:
            print(f'Error: {e}')
            self.connected = False
            return False

    def is_server_up(self):
        try:
            # Create a UDP socket
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                # Set a timeout for the connection attempt
                s.settimeout(2)  # Adjust the timeout as needed

                # Send a dummy message to the server
                s.sendto(b'', (self.server_address, self.server_port))
                print(f"Server {self.server_address}:{self.server_port} is up and active.")
                self.connected = True
                return True
        except Exception as e:
            print(f"Server {self.server_address}:{self.server_port} is not reachable: {e}")
            self.connected = False
            return False

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
                
    def send_message(self, id, cmd, body1, body2, type):
        if type ==  Message_type['short']:
            message = bytearray(12)  # Total size of the message
            message[0] = 0xbb  # message.Head
            message[1] = id#.to_bytes(1, byteorder='little')
            # if  command_id['Set_Axis_State'] == type:
            message[2] = cmd#.to_bytes(1, byteorder='little')#command id
            #3 4 5 6 7 8 9 10 message
            #0 1 2 3 4 5 6 7  rdrive data
            # body1 = cmd1#.to_bytes(4, byteorder='little')
            # body2 = cmd2#.to_bytes(4, byteorder='little')
            message[3] = body1[0]#byte 0 3 is 0
            message[4] = body1[1]
            message[5] = body1[2]
            message[6] = body1[3]

            message[7] = body2[0]#byte 0 3 is 0
            message[8] = body2[1]
            message[9] = body2[2]
            message[10] = body2[3]
            
            # Fill in the rest of the message bytes as needed
            message[11] = self.calculate_checksum(body1)  # message.Tail  

        if type ==  Message_type['full']:
            message = bytearray(52)  # Total size of the message
            message[0] = 0xb8  # message.Head
            message[1] = id#.to_bytes(1, byteorder='little')
            # if  command_id['Set_Axis_State'] == type:
            message[2] = cmd#.to_bytes(1, byteorder='little')#command id
            #3 4 5 6 7 8 9 10 message
            #0 1 2 3 4 5 6 7  rdive data
            # body1 = cmd1#.to_bytes(4, byteorder='little')
            # body2 = cmd2#.to_bytes(4, byteorder='little')
            for i in range(len(body1)):
                message[i+3] = body1[i]
            message[51] = 0xcc  # message.Tail  
            # hex_data = ' '.join(hex(byte) for byte in message)
            # print(hex_data)

        # print(message)      
        if self.connected:            
            try:
                self.client_socket.sendto(message, self.server_address_port)
                # print('Message sent successfully.')
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
                        # self.buffer.append(hex_data)
                        if self.callback:
                            self.callback(hex_data)
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

# # Usage
# client = UDPClient('192.168.8.88', 9999)

# if client.connect():
#     print('Connected to the server.')
#     client.send_message("disable")
#     client.start_receive_thread()  # Start receiving messages in a separate thread

#     # Get data from the buffer one by one
#     while True:
#         data = client.get_buffer_data()
#         if data is None:
#             # print('Buffer is empty.')
#             time.sleep(0.01)
#             continue
#             # break
#         else:
#             print(f'Buffer data: {data}')

# else:
#     print('Failed to connect to the server.')

# client.close()
