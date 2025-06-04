import struct
import socket
import select
import time

listen_ip = "0.0.0.0"
listen_port = 5006

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((listen_ip, listen_port))
udp_socket.setblocking(False)

print(f"Listening for UDP packets on {listen_ip}:{listen_port}...")

message_count = 0
start_time = time.time()

while True:
    try:
        ready = select.select([udp_socket], [], [], 0.001)  # Non-blocking with 1ms timeout
        if ready[0]:  # If data is available
            data, addr = udp_socket.recvfrom(32)
            if len(data) == 32:
                unpacked_data = struct.unpack("hhhhhhBffff", data)
                B_pos = unpacked_data[0:3]
                C_pos = unpacked_data[3:6]
                hooked = unpacked_data[6]
                quatRot = unpacked_data[7:11]

                message_count += 1
                elapsed_time = time.time() - start_time
                if elapsed_time >= 1.0:
                    hz = message_count / elapsed_time
                    print(f"Message frequency: {hz:.2f} Hz")
                    message_count = 0
                    start_time = time.time()

                print(f"Received from {addr}: B_pos={B_pos}, C_pos={C_pos}, quatRot={quatRot}, hooked={hooked}")
            else:
                print(f"Warning: Received unexpected data length ({len(data)} bytes) from {addr}")
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.001)  # Small delay to reduce CPU usage

