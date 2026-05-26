# # gcs/visualizer_tab.py — HUD tab
# import tkinter as tk
# from tkinter import ttk, messagebox
# import threading
# import queue

# from gcs.hud import FlightHUD
# from gcs.theme import TH


# class VisualizerTab:
#     """🛩  HUD — streams roll/pitch/yaw and renders the flight HUD."""

#     def __init__(self, parent, connection):
#         self.connection     = connection
#         self.tab            = tk.Frame(parent, bg=TH["bg"])
#         self._queue         = queue.Queue()
#         self._viewer        = None
#         self._stream_thread = None
#         self._stop_event    = threading.Event()
#         self._build()

#     # ------------------------------------------------------------------
#     def _build(self):
#         root = self.tab

#         hdr = tk.Frame(root, bg=TH["bg"])
#         hdr.pack(fill="x", padx=14, pady=(14, 4))
#         tk.Label(hdr, text="Orientation HUD",
#                  bg=TH["bg"], fg=TH["accent"],
#                  font=("Segoe UI", 13, "bold")).pack(anchor="w")
#         tk.Label(hdr,
#                  text="Opens a separate window with a real-time "
#                       "artificial horizon, pitch ladder, roll scale and compass.",
#                  bg=TH["bg"], fg=TH["dim"],
#                  font=("Segoe UI", 9)).pack(anchor="w")

#         tk.Frame(root, bg=TH["border"], height=1).pack(
#             fill="x", padx=14, pady=6)

#         panel = tk.Frame(root, bg=TH["panel"])
#         panel.pack(fill="both", expand=True, padx=14, pady=4)

#         tk.Label(panel, text="🛩",
#                  bg=TH["panel"], fg=TH["accent"],
#                  font=("Segoe UI", 52)).pack(pady=(28, 6))

#         tk.Label(panel,
#                  text="Artificial Horizon  ·  Pitch Ladder  ·  Roll Arc  ·  Compass",
#                  bg=TH["panel"], fg=TH["dim"],
#                  font=("Segoe UI", 9)).pack(pady=(0, 16))

#         for feat in [
#             "Real-time artificial horizon with sky / ground split",
#             "Pitch ladder with degree markings (±30°)",
#             "Roll scale arc with animated gold pointer",
#             "Scrolling compass heading tape (N / S / E / W)",
#             "Numeric PITCH / ROLL / YAW sidebar — warnings above limits",
#         ]:
#             row = tk.Frame(panel, bg=TH["panel"])
#             row.pack(anchor="w", padx=40, pady=1)
#             tk.Label(row, text="▸",
#                      bg=TH["panel"], fg=TH["accent"],
#                      font=("Segoe UI", 9)).pack(side="left")
#             tk.Label(row, text=feat,
#                      bg=TH["panel"], fg=TH["text"],
#                      font=("Segoe UI", 9)).pack(side="left", padx=6)

#         tk.Frame(panel, bg=TH["border"], height=1).pack(
#             fill="x", padx=24, pady=16)

#         btn_row = tk.Frame(panel, bg=TH["panel"])
#         btn_row.pack(pady=4)

#         self._start_btn = tk.Button(btn_row,
#             text="▶  Launch HUD",
#             command=self._start,
#             bg=TH["acc2"], fg=TH["bright"],
#             font=("Segoe UI", 11, "bold"),
#             relief="flat", bd=0, padx=20, pady=8,
#             cursor="hand2",
#             activebackground=TH["accent"],
#             activeforeground="#000")
#         self._start_btn.pack(side="left", padx=(0, 12))

#         self._stop_btn = tk.Button(btn_row,
#             text="⏹  Stop HUD",
#             command=self._stop,
#             bg=TH["panel2"], fg=TH["dim"],
#             font=("Segoe UI", 11, "bold"),
#             relief="flat", bd=0, padx=20, pady=8,
#             cursor="hand2",
#             activebackground=TH["red"],
#             activeforeground=TH["bright"],
#             state="disabled")
#         self._stop_btn.pack(side="left")

#         self._status_lbl = tk.Label(panel, text="",
#                                      bg=TH["panel"], fg=TH["dim"],
#                                      font=("Segoe UI", 9))
#         self._status_lbl.pack(pady=(10, 0))

#     # ------------------------------------------------------------------
#     def _start(self):
#         self._stop_event.clear()
#         self._queue = queue.Queue()

#         resp = self.connection.send_command("startCubeStream", timeout=2.0)
#         if "CUBE_STREAM_START" not in resp:
#             messagebox.showerror("Error",
#                 f"ESP32 did not acknowledge. Got: {resp!r}")
#             return

#         if not self._viewer or not self._viewer.is_alive():
#             self._viewer = FlightHUD(self._queue)
#             self._viewer.start()

#         self._stream_thread = threading.Thread(
#             target=self._stream_worker, daemon=True)
#         self._stream_thread.start()

#         self._start_btn.config(state="disabled")
#         self._stop_btn.config(state="normal",
#                                bg=TH["red"], fg=TH["bright"])
#         self._status_lbl.config(text="● Streaming", fg=TH["green"])

#     # ------------------------------------------------------------------
#     def _stop(self):
#         self._stop_event.set()
#         try:
#             self.connection.send_command("stopCubeStream", timeout=1.0)
#         except Exception:
#             pass
#         if self._viewer and self._viewer.is_alive():
#             self._viewer.stop()
#             self._viewer.join(timeout=2.0)
#             self._viewer = None
#         self._start_btn.config(state="normal")
#         self._stop_btn.config(state="disabled",
#                                bg=TH["panel2"], fg=TH["dim"])
#         self._status_lbl.config(text="Stopped.", fg=TH["dim"])

#     # ------------------------------------------------------------------
#     def _stream_worker(self):
#         buf = ""
#         try:
#             while not self._stop_event.is_set():
#                 if self._viewer and not self._viewer.is_alive():
#                     self.tab.after(0, self._stop)
#                     break
#                 chunk = self.connection.recv_data(bufsize=2048, timeout=0.05)
#                 if not chunk:
#                     continue
#                 buf += chunk
#                 while "\n" in buf:
#                     line, buf = buf.split("\n", 1)
#                     line = line.strip()
#                     if not line or "STOPPED" in line:
#                         continue
#                     try:
#                         pitch, roll, yaw = map(float, line.split(","))
#                         self._queue.put((-roll, -pitch, yaw))
#                     except ValueError:
#                         continue
#         except Exception as e:
#             print(f"[VisualizerTab] stream error: {e}")

# gcs/visualizer_tab.py — HUD tab
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue

from gcs.hud import FlightHUD
from gcs.theme import TH


class VisualizerTab:
    """🛩  HUD — streams roll/pitch/yaw and renders the flight HUD."""

    def __init__(self, parent, connection):
        self.connection     = connection
        self.tab            = tk.Frame(parent, bg=TH["bg"])
        self._queue         = queue.Queue()
        self._viewer        = None
        self._stream_thread = None
        self._stop_event    = threading.Event()
        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        root = self.tab

        hdr = tk.Frame(root, bg=TH["bg"])
        hdr.pack(fill="x", padx=14, pady=(14, 4))
        tk.Label(hdr, text="Orientation HUD",
                 bg=TH["bg"], fg=TH["accent"],
                 font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(hdr,
                 text="Opens a separate window with a real-time "
                      "artificial horizon, pitch ladder, roll scale and compass.",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")

        tk.Frame(root, bg=TH["border"], height=1).pack(
            fill="x", padx=14, pady=6)

        panel = tk.Frame(root, bg=TH["panel"])
        panel.pack(fill="both", expand=True, padx=14, pady=4)

        tk.Label(panel, text="🛩",
                 bg=TH["panel"], fg=TH["accent"],
                 font=("Segoe UI", 52)).pack(pady=(28, 6))

        tk.Label(panel,
                 text="Artificial Horizon  ·  Pitch Ladder  ·  Roll Arc  ·  Compass",
                 bg=TH["panel"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(pady=(0, 16))

        for feat in [
            "Real-time artificial horizon with sky / ground split",
            "Pitch ladder with degree markings (±30°)",
            "Roll scale arc with animated gold pointer",
            "Scrolling compass heading tape (N / S / E / W)",
            "Numeric PITCH / ROLL / YAW sidebar — warnings above limits",
        ]:
            row = tk.Frame(panel, bg=TH["panel"])
            row.pack(anchor="w", padx=40, pady=1)
            tk.Label(row, text="▸",
                     bg=TH["panel"], fg=TH["accent"],
                     font=("Segoe UI", 9)).pack(side="left")
            tk.Label(row, text=feat,
                     bg=TH["panel"], fg=TH["text"],
                     font=("Segoe UI", 9)).pack(side="left", padx=6)

        tk.Frame(panel, bg=TH["border"], height=1).pack(
            fill="x", padx=24, pady=16)

        btn_row = tk.Frame(panel, bg=TH["panel"])
        btn_row.pack(pady=4)

        self._start_btn = tk.Button(btn_row,
            text="▶  Launch HUD",
            command=self._start,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 11, "bold"),
            relief="flat", bd=0, padx=20, pady=8,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000")
        self._start_btn.pack(side="left", padx=(0, 12))

        self._stop_btn = tk.Button(btn_row,
            text="⏹  Stop HUD",
            command=self._stop,
            bg=TH["panel2"], fg=TH["dim"],
            font=("Segoe UI", 11, "bold"),
            relief="flat", bd=0, padx=20, pady=8,
            cursor="hand2",
            activebackground=TH["red"],
            activeforeground=TH["bright"],
            state="disabled")
        self._stop_btn.pack(side="left")

        self._status_lbl = tk.Label(panel, text="",
                                     bg=TH["panel"], fg=TH["dim"],
                                     font=("Segoe UI", 9))
        self._status_lbl.pack(pady=(10, 0))

    # ------------------------------------------------------------------
    def _start(self):
        self._stop_event.clear()
        self._queue = queue.Queue()

        resp = self.connection.send_command("startCubeStream", timeout=2.0)
        if "CUBE_STREAM_START" not in resp:
            messagebox.showerror("Error",
                f"ESP32 did not acknowledge. Got: {resp!r}")
            return

        if not self._viewer or not self._viewer.is_alive():
            self._viewer = FlightHUD(self._queue)
            self._viewer.start()

        self._stream_thread = threading.Thread(
            target=self._stream_worker, daemon=True)
        self._stream_thread.start()

        self._start_btn.config(state="disabled")
        self._stop_btn.config(state="normal",
                               bg=TH["red"], fg=TH["bright"])
        self._status_lbl.config(text="● Streaming", fg=TH["green"])

    # ------------------------------------------------------------------
    def _stop(self):
        """Stop streaming and the HUD window. Safe to call even after
        the tkinter window has been destroyed (e.g. from cleanup())."""
        self._stop_event.set()

        # Tell ESP32 to stop streaming
        try:
            self.connection.send_command("stopCubeStream", timeout=1.0)
        except Exception:
            pass

        # Stop the HUD pygame window
        if self._viewer and self._viewer.is_alive():
            self._viewer.stop()
            self._viewer.join(timeout=2.0)
            self._viewer = None

        # Update widgets only if they still exist (tkinter may have
        # already destroyed them during window close)
        try:
            self._start_btn.config(state="normal")
            self._stop_btn.config(state="disabled",
                                   bg=TH["panel2"], fg=TH["dim"])
            self._status_lbl.config(text="Stopped.", fg=TH["dim"])
        except Exception:
            pass   # widgets already destroyed — safe to ignore

    # ------------------------------------------------------------------
    def _stream_worker(self):
        buf = ""
        try:
            while not self._stop_event.is_set():
                if self._viewer and not self._viewer.is_alive():
                    self.tab.after(0, self._stop)
                    break
                chunk = self.connection.recv_data(bufsize=2048, timeout=0.05)
                if not chunk:
                    continue
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line or "STOPPED" in line:
                        continue
                    try:
                        pitch, roll, yaw = map(float, line.split(","))
                        self._queue.put((-roll, -pitch, yaw))
                    except ValueError:
                        continue
        except Exception as e:
            print(f"[VisualizerTab] stream error: {e}")