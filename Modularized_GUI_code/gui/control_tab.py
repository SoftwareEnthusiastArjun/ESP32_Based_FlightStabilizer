import tkinter as tk
from tkinter import ttk, messagebox

class ControlTab:
    def __init__(self, parent, connection):
        """
        Initialize the ControlTab for adjusting filter parameters.
        Args:
            parent: The parent widget (usually a ttk.Notebook).
            connection: An instance of ESP32Connection for sending commands.
        """
        self.parent = parent
        self.connection = connection
        self.tab = ttk.Frame(parent)  # Main frame for the tab content
        self.filter_sliders = {}      # Dictionary to store slider and label widgets
        self.filter_vars = {}         # Dictionary to store Tkinter DoubleVars for filter values
        self.build_tab()              # Build the tab UI

    def build_tab(self):
        """
        Create the UI elements within the tab: connection controls, sliders, and action buttons.
        """
        # Connection status and reconnect button
        connection_frame = ttk.LabelFrame(self.tab, text="Connection", padding=10)
        connection_frame.pack(fill="x", padx=10, pady=5)

        self.status_label = ttk.Label(connection_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side="left")

        connect_btn = ttk.Button(connection_frame, text="Reconnect", command=self.reconnect)
        connect_btn.pack(side="right")

        # Frame containing filter sliders
        controls_frame = ttk.LabelFrame(self.tab, text="Filter Parameters", padding=10)
        controls_frame.pack(fill="both", padx=10, pady=5, expand=True)

        # Filter configuration: (label, min value, max value)
        filters = [
            ('ACCEL_FILTER', 0, 1),
            ('GYRO_FILTER', 0, 1),
            ('COMP_FILTER', 0, 1)
        ]

        # Create slider + value display for each filter
        for idx, (name, min_val, max_val) in enumerate(filters):
            ttk.Label(controls_frame, text=name).grid(row=idx, column=0, sticky="w", pady=2)

            # Variable to bind slider value
            self.filter_vars[name] = tk.DoubleVar(value=0.0)

            # Create the slider
            slider = ttk.Scale(
                controls_frame,
                from_=min_val,
                to=max_val,
                variable=self.filter_vars[name],
                command=lambda val, n=name: self.on_slider_change(n, val),
                length=250
            )
            slider.grid(row=idx, column=1, padx=5, pady=2)

            # Label to show the current value beside the slider
            value_label = ttk.Label(controls_frame, text="0.000", width=8)
            value_label.grid(row=idx, column=2, padx=5, pady=2)

            # Store references for easy updates
            self.filter_sliders[name] = {
                'slider': slider,
                'label': value_label
            }

        # Action buttons (Read and Save)
        actions_frame = ttk.Frame(self.tab, padding=10)
        actions_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(actions_frame, text="Read Current Values", command=self.read_values).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Save to EEPROM", command=self.save_values).pack(side="right", padx=5)

    def reconnect(self):
        """
        Attempt to reconnect to the ESP32 and update the status label.
        Returns:
            bool: True if connection succeeded, False otherwise.
        """
        if self.connection.connect():
            self.status_label.config(text="Status: Connected", foreground="green")
            return True
        else:
            self.status_label.config(text="Status: Connection failed", foreground="red")
            return False

    def read_values(self):
        """
        Send a command to read current filter values from ESP32 and update sliders.
        """
        response = self.connection.send_command("get")
        try:
            if response:
                values = response.split(',')
                if len(values) == 3:
                    # Parse and update UI for each filter
                    a, g, c = map(float, values)
                    self.filter_vars['ACCEL_FILTER'].set(a)
                    self.filter_sliders['ACCEL_FILTER']['label'].config(text=f"{a:.3f}")
                    self.filter_vars['GYRO_FILTER'].set(g)
                    self.filter_sliders['GYRO_FILTER']['label'].config(text=f"{g:.3f}")
                    self.filter_vars['COMP_FILTER'].set(c)
                    self.filter_sliders['COMP_FILTER']['label'].config(text=f"{c:.3f}")
                else:
                    raise ValueError("Invalid number of values received")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid response from ESP32: {e}\nResponse: {response}")

    def save_values(self):
        """
        Send current slider values to the ESP32 and trigger saving them to EEPROM.
        """
        # Read values from the sliders
        a = self.filter_vars['ACCEL_FILTER'].get()
        g = self.filter_vars['GYRO_FILTER'].get()
        c = self.filter_vars['COMP_FILTER'].get()

        # Send individual set commands
        self.connection.send_command(f"setA{a:.3f}")
        self.connection.send_command(f"setG{g:.3f}")
        self.connection.send_command(f"setC{c:.3f}")

        # Send the save command to store values in EEPROM
        response = self.connection.send_command("save")
        if "OK" in response:
            messagebox.showinfo("EEPROM", "Values saved to EEPROM.")
        else:
            messagebox.showerror("EEPROM", "Failed to save values.")

    def on_slider_change(self, name, value):
        """
        Update the value label when a slider is moved.
        Args:
            name (str): Filter name (e.g., "ACCEL_FILTER")
            value (str or float): Current slider value
        """
        val = float(value)
        if name in self.filter_sliders:
            self.filter_sliders[name]['label'].config(text=f"{val:.3f}")
        else:
            print(f"[WARN] Unknown slider name: {name}")
