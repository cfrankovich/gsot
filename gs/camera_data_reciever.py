import socket
import cv2
import numpy as np

def start_server(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((host, port))
        print(f"UDP Server started, listening on {host}:{port}")

        try:
            while True:
                frame_data, addr = s.recvfrom(65527)  

                if not frame_data:
                    break

                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is not None:
                    cv2.imshow('Received Video', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        except Exception as e:
            print(f"Error: {e}")

        print("Server closed")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    HOST = ''
    PORT = 12302
    start_server(HOST, PORT)
