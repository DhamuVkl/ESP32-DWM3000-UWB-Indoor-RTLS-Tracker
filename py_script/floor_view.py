import socket
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy as np
from scipy.optimize import least_squares
import json

# ---------------- CONFIGURATION ---------------- #
HOST = "192.168.xx.xx"  # Your PC's IP address
PORT = 7007  # Must match ESP32 port
ANCHOR_POSITIONS = np.array(
    [[15, 5], [290, 5], [165, 625]]  # Anchor 1  # Anchor 2  # Anchor 3
)
MARGIN = 20  # For dynamic zoom (not needed for fixed view)
ROOM_WIDTH = 480  # cm
ROOM_HEIGHT = 650  # cm
IMAGE_FILE = "floorplan.png"  # Background image of your room
# ------------------------------------------------ #

# Global variables
latest_data = None
latest_signal_strengths = None  # New variable for signal strengths
data_lock = threading.Lock()
server_running = True
buffer = ""


def trilaterate(distances, anchor_positions):
    """Calculate tag position using trilateration with 3 anchors."""

    def equations(p):
        x, y = p
        return [
            np.sqrt(
                (x - anchor_positions[0][0]) ** 2 + (y - anchor_positions[0][1]) ** 2
            )
            - distances[0],
            np.sqrt(
                (x - anchor_positions[1][0]) ** 2 + (y - anchor_positions[1][1]) ** 2
            )
            - distances[1],
            np.sqrt(
                (x - anchor_positions[2][0]) ** 2 + (y - anchor_positions[2][1]) ** 2
            )
            - distances[2],
        ]

    initial_guess = np.mean(anchor_positions, axis=0)

    try:
        result = least_squares(equations, initial_guess, method="lm")
        return result.x if result.success else None
    except Exception as e:
        print(f"Trilateration error: {e}")
        return None


def wifi_server():
    """TCP server to receive data from ESP32"""
    global latest_data, latest_signal_strengths, server_running, buffer

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
                            decoded = data.decode("utf-8")
                            buffer += decoded

                            while "\n" in buffer:
                                line, buffer = buffer.split("\n", 1)
                                line = line.strip()
                                if line:
                                    try:
                                        # Parse JSON data
                                        json_data = json.loads(line)
                                        anchors = json_data[
                                            "anchors"
                                        ]  # Extract the anchors dictionary
                                        d1 = float(anchors["A1"]["distance"])
                                        d2 = float(anchors["A2"]["distance"])
                                        d3 = float(anchors["A3"]["distance"])
                                        s1 = float(anchors["A1"]["rssi"])
                                        s2 = float(anchors["A2"]["rssi"])
                                        s3 = float(anchors["A3"]["rssi"])
                                        with data_lock:
                                            latest_data = (d1, d2, d3)
                                            latest_signal_strengths = (s1, s2, s3)

                                        print(f"Tag ID: {json_data['tag_id']}")
                                        print(
                                            f"Received data: d1={d1:.2f} cm, d2={d2:.2f} cm, d3={d3:.2f} cm"
                                        )
                                        print(
                                            f"Signal strengths: s1={s1:.2f} dBm, s2={s2:.2f} dBm, s3={s3:.2f} dBm"
                                        )

                                    except (
                                        ValueError,
                                        KeyError,
                                        json.JSONDecodeError,
                                    ) as e:
                                        print(f"Invalid data: {line} - Error: {e}")
                        except ConnectionResetError:
                            print("Client disconnected")
                            break
            except OSError as e:
                if server_running:
                    print(f"Server error: {e}")
                break


# ---------------- PLOTTING SETUP ---------------- #
fig, ax = plt.subplots(figsize=(10, 8))

# Load and display room background image
bg_img = mpimg.imread(IMAGE_FILE)
img_extent = [0, ROOM_WIDTH, 0, ROOM_HEIGHT]
ax.imshow(bg_img, extent=img_extent, origin="lower", alpha=0.6, zorder=-1)

# Plot anchors and create text annotations for signal strength
anchor_texts = []
for i, (x, y) in enumerate(ANCHOR_POSITIONS):
    color = ["g", "b", "m"][i]
    ax.plot(x, y, f"{color}^", markersize=12, label=f"Anchor {i + 1}")
    # Create text object for signal strength, positioned above each anchor
    txt = ax.text(x, y + 20, "", color=color, ha="center", va="bottom")
    anchor_texts.append(txt)

# Tag and path
(tag_dot,) = ax.plot([], [], "ro", markersize=10, label="Tag Position")
(path_line,) = ax.plot([], [], "b-", alpha=0.5, linewidth=1, label="Tag Path")

path_x, path_y = [], []

# Set fixed room size view
ax.set_xlim(0, ROOM_WIDTH)
ax.set_ylim(0, ROOM_HEIGHT)
ax.set_aspect("equal")
ax.grid(True, linestyle="--", alpha=0.7)
ax.legend(loc="upper right")
ax.set_title("Real-time UWB Tag Position Tracking (3 Anchors)", pad=40)
ax.set_xlabel("X Position (cm)", labelpad=10)
ax.set_ylabel("Y Position (cm)", labelpad=10)


# ---------------- ANIMATION FUNCTION ---------------- #
def update(frame):
    global latest_data, latest_signal_strengths, path_x, path_y

    with data_lock:
        if latest_data and latest_signal_strengths:
            d1, d2, d3 = latest_data
            s1, s2, s3 = latest_signal_strengths

            # Update signal strength texts
            for i, (txt, sig) in enumerate(zip(anchor_texts, (s1, s2, s3))):
                txt.set_text(f"{sig:.1f} dBm")

            pos = trilaterate([d1, d2, d3], ANCHOR_POSITIONS)
            if pos is not None:
                x_cm, y_cm = pos
                print(f"Tag position: x={x_cm:.1f} cm, y={y_cm:.1f} cm")

                tag_dot.set_data([x_cm], [y_cm])
                path_x.append(x_cm)
                path_y.append(y_cm)
                if len(path_x) > 100:
                    path_x.pop(0)
                    path_y.pop(0)
                path_line.set_data(path_x, path_y)

    return tag_dot, path_line, *anchor_texts


# ---------------- MAIN EXECUTION ---------------- #
if __name__ == "__main__":
    server_thread = threading.Thread(target=wifi_server, daemon=True)
    server_thread.start()

    ani = animation.FuncAnimation(
        fig, update, interval=100, cache_frame_data=False, blit=False
    )

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        server_running = False
        plt.close()
