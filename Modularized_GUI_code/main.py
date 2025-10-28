# Import the tkinter GUI library
import tkinter as tk
from tkinter import ttk

# Import the ESP32 connection class to handle serial communication
from connection import ESP32Connection

# Import the individual tab classes for the GUI
from gui.control_tab import ControlTab
from gui.servo_tab import ServoTab
from gui.visualizer_tab import VisualizerTab
from gui.pid_tab import PIDTab

class ESP32ControlPanel:
    def __init__(self, root):
        # Store the main root window
        self.root = root
        root.title("ESP32 Control Panel")  # Set the window title
        root.geometry("700x550")          # Set the window size

        # Create a connection object to communicate with the ESP32
        self.connection = ESP32Connection()

        # Create a Notebook (tabbed interface) and add it to the window
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(expand=1, fill="both")  # Make the tabs fill the window

        # Set up the individual tabs
        self.setup_tabs()

        # Bind an event that triggers when the user switches tabs
        self.tabs.bind("<<NotebookTabChanged>>", self.on_tab_change)

    def setup_tabs(self):
        # Create each tab and pass the connection object to allow communication with ESP32
        self.control_tab = ControlTab(self.tabs, self.connection)
        self.servo_tab = ServoTab(self.tabs, self.connection)
        self.visualizer_tab = VisualizerTab(self.tabs, self.connection)
        self.pitch_pid_tab = PIDTab(self.tabs, self.connection, "Pitch")
        self.roll_pid_tab = PIDTab(self.tabs, self.connection, "Roll")

        # Add the tabs to the notebook with appropriate labels
        self.tabs.add(self.control_tab.tab, text="Filter Control")
        self.tabs.add(self.servo_tab.tab, text="Servo Input")
        self.tabs.add(self.visualizer_tab.tab, text="3D Visualizer")
        self.tabs.add(self.pitch_pid_tab.tab, text="Pitch PID")
        self.tabs.add(self.roll_pid_tab.tab, text="Roll PID")

    def on_tab_change(self, event):
        # This method is called whenever the user changes tabs

        # Get the name of the currently selected tab
        selected_tab = self.tabs.tab(self.tabs.select(), "text")

        # If the "Servo Input" tab is selected and the PWM stream hasn't started yet, start it
        if selected_tab == "Servo Input" and not self.servo_tab.pwm_stream_thread_started:
            self.servo_tab.start_pwm_stream()
        # If another tab is selected and the PWM stream is running, stop it
        elif selected_tab != "Servo Input" and self.servo_tab.pwm_stream_thread_started:
            self.servo_tab.stop_pwm_stream()

    def cleanup(self):
        # Cleanup resources when the application is closing

        # Close the ESP32 serial connection
        self.connection.close()

        # Stop any threads or resources in the visualizer tab
        self.visualizer_tab.cleanup()

# Main entry point of the application
if __name__ == "__main__":
    root = tk.Tk()  # Create the main application window

    # Set a custom theme for better styling
    style = ttk.Style()
    style.theme_use('clam')

    # Create the main application object
    app = ESP32ControlPanel(root)

    # Run the application inside a try-finally block to ensure cleanup
    try:
        root.mainloop()  # Start the Tkinter event loop
    finally:
        app.cleanup()    # Cleanup when the app exits (e.g., on window close)
