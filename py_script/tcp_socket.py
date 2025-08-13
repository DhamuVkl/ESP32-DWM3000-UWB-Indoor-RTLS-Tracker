import socket
import threading

# ---------------- CONFIGURATION ---------------- #
HOST = "192.168.29.64"  # Your PC's IP address
PORT = 7007  # Must match ESP32 port
# ----------------------------------------------- #

server_running = True


def wifi_server():
    """Simple TCP server to receive and print raw data from ESP32"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")

        while server_running:
            try:
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    while server_running:
                        try:
                            data = conn.recv(1024)
                            if not data:
                                break
                            decoded = data.decode("utf-8").strip()
                            print(decoded)
                        except ConnectionResetError:
                            print("Client disconnected")
                            break
            except OSError as e:
                if server_running:
                    print(f"Server error: {e}")
                break


if __name__ == "__main__":
    try:
        server_thread = threading.Thread(target=wifi_server, daemon=True)
        server_thread.start()

        # Keep main thread alive until Ctrl+C
        while True:
            pass

    except KeyboardInterrupt:
        print("\nShutting down server...")
        server_running = False
