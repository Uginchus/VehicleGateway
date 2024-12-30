import socket
import struct
import time

# Define MSP command codes
MSP_SET_RAW_RC = 200

# Define the TCP connection parameters
tcp_host = '127.0.0.1'
tcp_port = 5761

# Open the TCP connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((tcp_host, tcp_port))

def checksum(data):
    return sum(data) % 256

def send_msp(command, data=[]):
    payload_size = len(data)
    header = [ord('$'), ord('M'), ord('<'), payload_size, command]
    checksum_value = checksum(header[3:] + data)
    message = header + data + [checksum_value]
    sock.sendall(bytearray(message))

def send_raw_rc(channels):
    data = []
    for channel in channels:
        data += list(struct.pack('<H', channel))
    send_msp(MSP_SET_RAW_RC, data)

# Example usage: Send RC values (1000-2000) for 4 channels (roll, pitch, yaw, throttle)
channels = [1500, 1500, 1500, 1000]  # Update these values as needed
send_raw_rc(channels)

# Close the TCP connection
sock.close()