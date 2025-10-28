import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
from time import sleep
from cube_visualizer2 import CubeVisualizer2

class VisualizerTab:
    def __init__(self, parent, connection):
        """
        Initializes the 3D Visualizer tab.

        Args:
            parent: The parent Notebook widget.
            connection: The ESP32Connection object used to communicate with ESP32.
        """
        self.parent = parent
        self.connection = connection
        self.tab = ttk.Frame(parent)

        # Queue to communicate orientation data between thread and visualizer
        self.data_queue = queue.Queue()

        # The 3D cube viewer instance
        self.viewer = None

        self.build_tab()

    def build_tab(self):
        """
        Builds the GUI elements of the visualizer tab.
        """
        vis_frame = ttk.LabelFrame(self.tab, text="3D Cube Visualizer", padding=10)
        vis_frame.pack(padx=20, pady=20, fill="both", expand=True)

        # Buttons to start and stop the cube visualization
        ttk.Button(vis_frame, text="Start Cube", command=self.start_cube).pack(pady=10)
        ttk.Button(vis_frame, text="Stop Cube", command=self.stop_cube).pack(pady=10)

    def start_cube(self):
        """
        Sends command to start the cube data stream from ESP32,
        then starts a background thread to continuously receive and process orientation data.
        """
        response = self.connection.send_command("startCubeStream")

        # Optional: process and ignore pre-stream data
        while response != "CUBE_STREAM_START":
            print(response)
            parts = response.strip().split(',')
            if len(parts) == 3:
                try:
                    floats = list(map(float, parts))
                    print("i'm in")  # Debug: Received valid data even before official stream start
                except:
                    pass
            response = self.connection.recv_data(timeout=0.5)

        sleep(0.1)

        if "CUBE_STREAM_START" not in response:
            messagebox.showerror("Error", "Failed to start cube stream")
            return

        # Launch cube visualizer window (only if not already running)
        if not self.viewer or not self.viewer.is_alive():
            self.data_queue = queue.Queue()
            self.viewer = CubeVisualizer2(self.data_queue)
            self.viewer.start()

        # Background thread to receive cube stream data
        def cube_stream_loop():
            buffer = ""
            try:
                self.connection.sock.settimeout(None)  # Block until data arrives

                while True:
                    try:
                        # Read incoming data from ESP32
                        chunk = self.connection.sock.recv(1024).decode(errors='ignore')
                        if not chunk:
                            print("[Cube Stream] Disconnected from ESP32.")
                            break

                        buffer += chunk
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            data = line.strip()
                            if not data:
                                continue

                            if "CUBE_STREAM_STOPPED" in data:
                                return  # Stop signal received from ESP32

                            try:
                                gx, gy, gz = map(float, data.split(','))
                                print(f"gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}")
                                self.data_queue.put((gx, gy, gz))  # Send data to visualizer
                            except ValueError:
                                continue  # Ignore malformed lines

                    except Exception as e:
                        print(f"[Cube Stream Error - inner] {e}")
                        break

            except Exception as e:
                print(f"[Cube Stream Error - outer] {e}")

        # Start the streaming thread
        threading.Thread(target=cube_stream_loop, daemon=True).start()

    def stop_cube(self):
        """
        Stops the cube stream both on ESP32 and in the local visualizer.
        """
        response = self.connection.send_command("stopCubeStream")
        sleep(0.1)

        if "CUBE_STREAM_STOPPED" not in response:
            messagebox.showerror("Error", "Failed to stop cube stream")

        # Stop and clean up the visualizer thread
        if self.viewer and self.viewer.is_alive():
            try:
                self.viewer.stop()
                self.viewer.join(timeout=2)
                self.viewer = None

                # Clear any remaining data
                with self.data_queue.mutex:
                    self.data_queue.queue.clear()
            except Exception as e:
                messagebox.showerror("Visualizer Error", f"Failed to stop cube visualizer: {e}")

        # Restore timeout to non-blocking behavior
        self.connection.sock.settimeout(0.05)

    def cleanup(self):
        """
        Cleans up resources (called when app is closing).
        """
        if self.viewer and self.viewer.is_alive():
            self.viewer.stop()
            self.viewer.join()
