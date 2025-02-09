import socket

BROADCAST_IP = "255.255.255.255"
PORT = 5005
MESSAGE = "Hello, UDP!"

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

try:
    print(f"Sending message: {MESSAGE}")
    sock.sendto(MESSAGE.encode(), (BROADCAST_IP, PORT))
finally:
    sock.close()