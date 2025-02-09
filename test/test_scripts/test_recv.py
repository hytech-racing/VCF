
#import socket
#import struct

#UDP_IP = "192.168.1.31"  # Loopback address
#UDP_PORT = 4444  # Port number

##sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create UDP socket
#ock.bind((UDP_IP, UDP_PORT))  # Bind to IP and port

#print(f"Listening on {UDP_IP}:{UDP_PORT}...")

import socket
import struct
 
MULTICAST_IP = "239.1.1.1"
PORT = 5000
 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", PORT))
 
mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
 
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr}: {data.decode()}")


