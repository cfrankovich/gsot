import socket

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
                        data = conn.recv(1024)
                        if not data:
                            break
                        print("Received data:", data.decode())
                except ConnectionResetError:
                    print("Connection reset by client")
                except Exception as e:
                    print(f"Error: {e}")

                print("Connection closed")

if __name__ == "__main__":
    HOST = ''  
    PORT = 12345  
    start_server(HOST, PORT)
