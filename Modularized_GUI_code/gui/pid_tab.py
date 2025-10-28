import tkinter as tk
from tkinter import ttk, messagebox

class PIDTab:
    def __init__(self, parent, connection, pid_type):
        """
        Initialize the PIDTab UI for a specific PID type (e.g., Pitch or Roll).
        Args:
            parent: The parent container (typically a ttk.Notebook).
            connection: An ESP32Connection instance for communication.
            pid_type: A string specifying the PID type, like "Pitch" or "Roll".
        """
        self.parent = parent
        self.connection = connection
        self.pid_type = pid_type  # Used to differentiate between Pitch and Roll
        self.tab = ttk.Frame(parent)  # The actual tab widget
        self.sliders = {}  # Holds slider and label widgets
        self.vars = {}     # Holds DoubleVar instances for Kp, Ki, Kd
        self.build_tab()   # Build the UI

    def build_tab(self):
        """
        Create the UI components for the PID tab.
        Includes sliders for Kp, Ki, and Kd, along with labels and buttons.
        """
        # Main frame for PID controls
        controls_frame = ttk.LabelFrame(
            self.tab, text=f"{self.pid_type} PID Parameters", padding=10
        )
        controls_frame.pack(fill="both", padx=10, pady=5, expand=True)

        # List of PID parameters with their ranges
        pid_params = [
            ('Kp', 0, 10),
            ('Ki', 0, 5),
            ('Kd', 0, 5)
        ]

        # Create a slider + label for each PID parameter
        for idx, (name, min_val, max_val) in enumerate(pid_params):
            ttk.Label(controls_frame, text=name).grid(row=idx, column=0, sticky="w", pady=2)

            var_name = f"{self.pid_type}_{name}"
            self.vars[var_name] = tk.DoubleVar(value=0.0)

            slider = ttk.Scale(
                controls_frame,
                from_=min_val,
                to=max_val,
                variable=self.vars[var_name],
                command=lambda val, n=var_name: self.on_slider_change(n, val),
                length=250
            )
            slider.grid(row=idx, column=1, padx=5, pady=2)

            value_label = ttk.Label(controls_frame, text="0.000", width=8)
            value_label.grid(row=idx, column=2, padx=5, pady=2)

            self.sliders[var_name] = {
                'slider': slider,
                'label': value_label
            }

        # Frame for Read and Save buttons
        actions_frame = ttk.Frame(self.tab, padding=10)
        actions_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(actions_frame, text="Read Current Values", 
                   command=self.read_values).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Save to EEPROM", 
                   command=self.save_values).pack(side="right", padx=5)

    def reconnect(self):
        """
        Attempt to reconnect to the ESP32 and update a status label (currently unused).
        """
        if self.connection.connect():
            self.status_label.config(text="Status: Connected", foreground="green")
            return True
        else:
            self.status_label.config(text="Status: Connection failed", foreground="red")
            return False

    def read_values(self):
        """
        Request the current PID values from the ESP32 and update the sliders accordingly.
        """
        response = self.connection.send_command(f"get{self.pid_type}PID")
        try:
            if response:
                values = response.split(',')
                if len(values) == 3:
                    kp, ki, kd = map(float, values)

                    # Update UI with received values
                    for param, val in zip(['Kp', 'Ki', 'Kd'], [kp, ki, kd]):
                        key = f"{self.pid_type}_{param}"
                        self.vars[key].set(val)
                        self.sliders[key]['label'].config(text=f"{val:.3f}")
                else:
                    raise ValueError("Invalid number of values received")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid response from ESP32: {e}\nResponse: {response}")

    def save_values(self):
        """
        Send the updated PID values to the ESP32 and trigger EEPROM save.
        """
        # Read current values from sliders
        kp = self.vars[f"{self.pid_type}_Kp"].get()
        ki = self.vars[f"{self.pid_type}_Ki"].get()
        kd = self.vars[f"{self.pid_type}_Kd"].get()

        # Send each parameter individually
        self.connection.send_command(f"set{self.pid_type}P{kp:.3f}")
        self.connection.send_command(f"set{self.pid_type}I{ki:.3f}")
        self.connection.send_command(f"set{self.pid_type}D{kd:.3f}")

        # Send 'save' command to persist values in EEPROM
        response = self.connection.send_command("save")
        if "OK" in response:
            messagebox.showinfo("EEPROM", f"{self.pid_type} PID values saved to EEPROM.")
        else:
            messagebox.showerror("EEPROM", "Failed to save values.")

    def on_slider_change(self, name, value):
        """
        Update the text label next to a slider when it is adjusted.
        Args:
            name (str): The variable name (e.g., "Pitch_Kp").
            value (str or float): The new value of the slider.
        """
        val = float(value)
        if name in self.sliders:
            self.sliders[name]['label'].config(text=f"{val:.3f}")
        else:
            print(f"[WARN] Unknown slider name: {name}")
