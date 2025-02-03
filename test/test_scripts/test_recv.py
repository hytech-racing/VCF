import socket
import struct

UDP_IP = "192.168.1.31"  # Loopback address
UDP_PORT = 4444  # Port number

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create UDP socket
sock.bind((UDP_IP, UDP_PORT))  # Bind to IP and port

print(f"Listening on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)  # Receive data
    received_float = struct.unpack("f", data)[0]  # Unpack bytes into float
    print(f"Received from {addr}: {received_float}")
