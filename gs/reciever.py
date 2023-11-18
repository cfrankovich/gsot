import socket
import struct

def start_server(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()

        print(f"Server started, listening on {host}:{port}")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                try:
                    while True:
                        data = conn.recv(72) # 9 bytes 
                        if not data:
                            break
                        metric_data = struct.unpack('ddddddddq', data)
                        print("Received MetricData:", metric_data)
                except ConnectionResetError:
                    print("Connection reset by client")
                except Exception as e:
                    print(f"Error: {e}")

                print("Connection closed")

if __name__ == "__main__":
    HOST = ''  
    PORT = 12345  
    start_server(HOST, PORT)
