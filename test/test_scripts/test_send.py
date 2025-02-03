import socket
import struct

UDP_IP = "192.168.1.30"  # Loopback address
UDP_PORT = 5555  # Port number
FLOAT_VALUE = 6969.4  # Example float

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create UDP socket
packed_data = struct.pack("f", FLOAT_VALUE)  # Pack float into bytes

sock.sendto(packed_data, (UDP_IP, UDP_PORT))  # Send data
print(f"Sent: {FLOAT_VALUE}")

sock.close()