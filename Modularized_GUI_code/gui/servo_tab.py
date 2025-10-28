import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue

class ServoTab:
    def __init__(self, parent, connection):
        """
        Initializes the ServoTab, which visualizes incoming PWM signals from ESP32.
        Args:
            parent: The parent widget (usually a Notebook tab).
            connection: The ESP32Connection object used for communication.
        """
        self.parent = parent
        self.connection = connection
        self.tab = ttk.Frame(parent)

        # Thread control flags
        self.pwm_stream_thread_started = False
        self.pwm_thread = None
        self.stop_pwm_thread = threading.Event()

        # Build the tab UI
        self.build_tab()

    def build_tab(self):
        """
        Creates the UI layout for PWM bars and labels.
        """
        servo_frame = ttk.LabelFrame(self.tab, text="Servo Signal Inputs", padding=10)
        servo_frame.pack(padx=20, pady=20, fill="both", expand=True)

        self.pwm_canvases = []  # Canvas widgets to draw PWM bars
        self.pwm_bars = []      # Actual bar rectangles on canvas
        self.pwm_labels = []    # Labels showing numeric PWM percentages

        # Create 3 PWM bars
        for i in range(3):
            canvas = tk.Canvas(servo_frame, height=30, width=300, bg="white", bd=1, relief="sunken")
            canvas.pack(pady=10)

            bar = canvas.create_rectangle(0, 0, 0, 30, fill="green")  # Initial empty bar
            label = ttk.Label(servo_frame, text=f"PWM {i+1}: 0%", font=("Arial", 12))
            label.pack()

            self.pwm_canvases.append(canvas)
            self.pwm_bars.append(bar)
            self.pwm_labels.append(label)

        # Label for displaying auto-pilot status
        self.ap_label = ttk.Label(servo_frame, text="Auto Pilot: Unknown", font=("Arial", 12), foreground="gray")
        self.ap_label.pack(pady=10)

        # Stop button to halt PWM streaming manually
        stop_button = ttk.Button(servo_frame, text="Stop PWM Stream", command=self.stop_pwm_stream_and_send)
        stop_button.pack(pady=10)

    def start_pwm_stream(self):
        """
        Starts the background thread to receive PWM stream data.
        """
        self.pwm_stream_thread_started = True
        self._start_pwm_stream_thread()

    def _start_pwm_stream_thread(self):
        """
        Internal method to run the stream thread logic for receiving and updating PWM values.
        """
        def stream_pwm():
            self.stop_pwm_thread.clear()
            try:
                # Send command to start stream
                response = self.connection.send_command("startPWMStream")
                if response != "PWM_STREAM_START":
                    raise Exception("Failed to start PWM stream")

                buffer = ""
                while not self.stop_pwm_thread.is_set():
                    chunk = self.connection.recv_data(timeout=0.01)
                    if not chunk:
                        continue
                    buffer += chunk
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        data = line.strip()
                        if not data:
                            continue

                        print(f"[PWM STREAM DATA] {data}")

                        # Parse and normalize PWM values
                        if "No signal" in data:
                            percentages = [0, 0, 0, 0]
                        else:
                            try:
                                percentages = list(map(int, data.split(',')))
                                percentages = [max(0, min(100, p)) for p in percentages]
                                if len(percentages) < 4:
                                    percentages += [0] * (4 - len(percentages))
                            except:
                                continue

                        # Update the UI on the main thread
                        self.parent.after(0, self.update_pwm_bar_display, percentages)
            except Exception as e:
                self.parent.after(0, lambda: messagebox.showerror("PWM Stream Error", str(e)))

        # Launch the PWM stream thread
        self.pwm_thread = threading.Thread(target=stream_pwm, daemon=True)
        self.pwm_thread.start()

    def update_pwm_bar_display(self, percentages):
        """
        Update the bar length and label for each PWM channel based on incoming percentages.
        Args:
            percentages (list): List of 4 values (PWM1, PWM2, PWM3, AutoPilot)
        """
        for i, percent in enumerate(percentages[:3]):
            self.pwm_canvases[i].coords(self.pwm_bars[i], 0, 0, 3 * percent, 30)
            self.pwm_labels[i].config(text=f"PWM {i+1}: {percent}%")

        # AutoPilot status indicator
        if len(percentages) >= 4:
            autopilot_percent = percentages[3]
            if 0 <= autopilot_percent <= 10:
                self.ap_label.config(text="Auto Pilot: OFF", foreground="red")
            elif 90 <= autopilot_percent <= 100:
                self.ap_label.config(text="Auto Pilot: ON", foreground="green")
            else:
                self.ap_label.config(text="Auto Pilot: Unknown", foreground="gray")

    def stop_pwm_stream(self):
        """
        Stops the PWM stream thread and ensures it exits cleanly.
        """
        self.stop_pwm_thread.set()
        if self.pwm_thread and self.pwm_thread.is_alive():
            self.pwm_thread.join(timeout=1)
            self.pwm_thread = None
        self.pwm_stream_thread_started = False

    def stop_pwm_stream_and_send(self):
        """
        Sends the stop command to the ESP32 and stops the local thread.
        """
        try:
            response = self.connection.send_command("stopPWMStream")
            print(f"[INFO] Sent 'stopPWMStream' command. Response: {response}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send stop command: {e}")
        finally:
            self.stop_pwm_stream()
