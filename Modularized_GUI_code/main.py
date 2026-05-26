"""
ESP32 Flight Stabilizer — Ground Control Station
Run:  python main.py
"""

import tkinter as tk
from tkinter import ttk
import threading
import datetime
import sys
import os

# Allow imports from the gcs/ package when run from this folder
sys.path.insert(0, os.path.dirname(__file__))

from gcs.connection      import ESP32Connection, discover_esp32
from gcs.theme           import TH, apply_ttk_theme, mk_btn
from gcs.control_tab     import ControlTab
from gcs.servo_tab       import ServoTab
from gcs.visualizer_tab  import VisualizerTab
from gcs.pid_tab         import PIDTab
from gcs.diagnostics_tab import DiagnosticsTab


class GCS:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("ESP32 Flight Stabilizer  —  Ground Control Station")
        root.geometry("860x700")
        root.minsize(740, 560)
        root.resizable(True, True)
        root.configure(bg=TH["bg"])

        apply_ttk_theme()

        self.conn = ESP32Connection(host="esp32.local", port=12345)
        self._manual_visible = False

        self._build_titlebar()
        self._build_conn_bar()
        self._build_notebook()
        self._build_statusbar()

    # ── Title bar ──────────────────────────────────────────────────────
    def _build_titlebar(self):
        bar = tk.Frame(self.root, bg=TH["bg"])
        bar.pack(fill="x", padx=10, pady=(8, 0))

        pill = tk.Frame(bar, bg=TH["accent"], width=36, height=36)
        pill.pack(side="left", padx=(0, 12))
        pill.pack_propagate(False)
        tk.Label(pill, text="✈", font=("Segoe UI", 18),
                 bg=TH["accent"], fg="#000"
                 ).place(relx=.5, rely=.5, anchor="center")

        tk.Label(bar, text="FLIGHT STABILIZER",
                 font=("Courier New", 14, "bold"),
                 bg=TH["bg"], fg=TH["bright"]).pack(side="left")
        tk.Label(bar, text="  GROUND CONTROL STATION",
                 font=("Courier New", 14),
                 bg=TH["bg"], fg=TH["accent"]).pack(side="left")
        tk.Label(bar, text=" - Powered by ESP32",
                 font=("Segoe UI", 9),
                 bg=TH["bg"], fg=TH["dim"]).pack(side="left", pady=(6, 0))

    # ── Connection bar ─────────────────────────────────────────────────
    def _build_conn_bar(self):
        bar = tk.Frame(self.root, bg=TH["panel"], height=48)
        bar.pack(fill="x", padx=10, pady=(6, 2))
        bar.pack_propagate(False)

        # Status dot
        self._dot_cv = tk.Canvas(bar, width=14, height=14,
                                  bg=TH["panel"], highlightthickness=0)
        self._dot_cv.pack(side="left", padx=(14, 6))
        self._dot = self._dot_cv.create_oval(2, 2, 12, 12,
                                              fill=TH["red"], outline="")

        self._info_var = tk.StringVar(value="Not connected")
        self._info_lbl = tk.Label(bar, textvariable=self._info_var,
                                   bg=TH["panel"], fg=TH["dim"],
                                   font=("Consolas", 10),
                                   width=38, anchor="w")
        self._info_lbl.pack(side="left", padx=(0, 10))

        self._auto_btn = mk_btn(bar, "⚡  Auto Connect",
                                 self._auto_connect,
                                 bg="#1a6fa8", abg=TH["accent"],
                                 padx=14, pady=5)
        self._auto_btn.pack(side="left", padx=(0, 8))

        mk_btn(bar, "Disconnect", self._disconnect,
               bg=TH["panel2"], abg=TH["border"],
               padx=10, pady=5).pack(side="left")

        mk_btn(bar, "Manual IP ▾", self._toggle_manual,
               bg=TH["panel"], fg=TH["dim"],
               abg=TH["panel2"], afg=TH["text"],
               font=("Segoe UI", 9),
               padx=8, pady=5).pack(side="right", padx=12)

        # ── Manual IP row (hidden by default) ────────────────────────
        self._manual_row = tk.Frame(self.root, bg="#0a0c12", pady=5)

        tk.Label(self._manual_row, text="  IP / Host:",
                 bg="#0a0c12", fg=TH["dim"],
                 font=("Consolas", 10)).pack(side="left")
        self._host_var = tk.StringVar(value="192.168.0.112")
        tk.Entry(self._manual_row, textvariable=self._host_var,
                 width=18, font=("Consolas", 10),
                 bg=TH["panel2"], fg=TH["text"],
                 insertbackground=TH["accent"],
                 relief="flat", bd=2
                 ).pack(side="left", padx=(4, 0), ipady=3)

        tk.Label(self._manual_row, text="  Port:",
                 bg="#0a0c12", fg=TH["dim"],
                 font=("Consolas", 10)).pack(side="left")
        self._port_var = tk.StringVar(value="12345")
        tk.Entry(self._manual_row, textvariable=self._port_var,
                 width=6, font=("Consolas", 10),
                 bg=TH["panel2"], fg=TH["text"],
                 insertbackground=TH["accent"],
                 relief="flat", bd=2
                 ).pack(side="left", padx=(4, 10), ipady=3)

        mk_btn(self._manual_row, "Connect", self._manual_connect,
               bg=TH["green"], fg="#000", abg="#00ff88", afg="#000",
               padx=10, pady=4).pack(side="left")

    # ── Notebook ───────────────────────────────────────────────────────
    def _build_notebook(self):
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=10, pady=(4, 0))

        self.ctrl_tab  = ControlTab(self.nb,    self.conn)
        self.servo_tab = ServoTab(self.nb,       self.conn)
        self.vis_tab   = VisualizerTab(self.nb,  self.conn)
        self.pitch_tab = PIDTab(self.nb, self.conn, "Pitch")
        self.roll_tab  = PIDTab(self.nb, self.conn, "Roll")
        self.diag_tab  = DiagnosticsTab(self.nb, self.conn)

        for frame, label in [
            (self.ctrl_tab.tab,  "  ⚙  Filters  "),
            (self.servo_tab.tab, "  🎮  RC Input  "),
            (self.vis_tab.tab,   "  🛩  HUD  "),
            (self.pitch_tab.tab, "  ↕  Pitch PID  "),
            (self.roll_tab.tab,  "  ↔  Roll PID  "),
            (self.diag_tab.tab,  "  📊  Diagnostics  "),
        ]:
            self.nb.add(frame, text=label)

        self.nb.bind("<<NotebookTabChanged>>", self._on_tab_change) #

    # ── Status bar ─────────────────────────────────────────────────────
    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=TH["panel"], height=24)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        self._status_var = tk.StringVar(value="Ready.")
        tk.Label(bar, textvariable=self._status_var,
                 bg=TH["panel"], fg=TH["dim"],
                 font=("Segoe UI", 9), anchor="w"
                 ).pack(side="left", padx=10)

        self._clock_var = tk.StringVar()
        tk.Label(bar, textvariable=self._clock_var,
                 bg=TH["panel"], fg=TH["dim"],
                 font=("Consolas", 9)).pack(side="right", padx=10)
        self._tick_clock()

    def _tick_clock(self):
        self._clock_var.set(datetime.datetime.now().strftime("%H:%M:%S"))
        self.root.after(1000, self._tick_clock)

    # ── Connection helpers ─────────────────────────────────────────────
    def _set_status(self, connected: bool, msg: str):
        self._dot_cv.itemconfig(self._dot,
            fill=TH["green"] if connected else TH["red"])
        self._info_lbl.config(
            fg=TH["text"] if connected else TH["dim"])
        self._info_var.set(msg)
        self._status_var.set(msg)
        self.ctrl_tab.set_connected(connected)

    def _auto_connect(self):
        self._auto_btn.config(state="disabled", text="Searching…")
        self._set_status(False, "Scanning network for ESP32…")

        def _run():
            def progress(msg):
                self.root.after(0, lambda m=msg: self._info_var.set(m))

            ip = discover_esp32(port=12345, hostname="esp32.local",
                                timeout=5.0, progress_cb=progress)

            def _done():
                self._auto_btn.config(state="normal",
                                       text="⚡  Auto Connect")
                if ip:
                    ok = self.conn.connect_to_ip(ip, 12345)
                    self._set_status(ok,
                        f"Connected  {ip}:12345" if ok
                        else f"Found {ip} but TCP failed")
                else:
                    self._set_status(False,
                        "ESP32 not found — check WiFi / AP isolation")

            self.root.after(0, _done)

        threading.Thread(target=_run, daemon=True).start()

    def _manual_connect(self):
        host = self._host_var.get().strip()
        try:
            port = int(self._port_var.get().strip())
        except ValueError:
            port = 12345
        if not host:
            return
        self._set_status(False, f"Connecting to {host}:{port}…")
        self.root.update_idletasks()
        ok = self.conn.connect_to_ip(host, port)
        self._set_status(ok,
            f"Connected  {host}:{port}" if ok
            else f"Failed — {host}:{port} unreachable")

    def _disconnect(self):
        self.conn.close()
        self._set_status(False, "Disconnected")

    def _toggle_manual(self):
        if self._manual_visible:
            self._manual_row.pack_forget()
            self._manual_visible = False
        else:
            self._manual_row.pack(fill="x", before=self.nb)
            self._manual_visible = True

    def _on_tab_change(self, _event):
        sel = self.nb.tab(self.nb.select(), "text").strip()
        if "RC Input" in sel:
            if not self.servo_tab.pwm_stream_thread_started:
                self.servo_tab.start_pwm_stream()
        else:
            if self.servo_tab.pwm_stream_thread_started:
                self.servo_tab.stop_pwm_stream_and_send()
        if "Diagnostics" not in sel:
            self.diag_tab.on_hide()

    def cleanup(self):
        """Called from the finally block after mainloop exits.
        tkinter widgets may already be destroyed at this point,
        so we only clean up threads and network connections."""
        try:
            if self.servo_tab.pwm_stream_thread_started:
                self.servo_tab.stop_pwm_stream_and_send()
        except Exception:
            pass
        try:
            self.diag_tab.on_hide()
        except Exception:
            pass
        try:
            # _stop() already handles destroyed widgets gracefully
            self.vis_tab._stop()
        except Exception:
            pass
        try:
            self.conn.close()
        except Exception:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    if sys.platform == "win32":
        try:
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    root = tk.Tk()
    root.option_add("*Font", "TkDefaultFont 10")
    app = GCS(root)
    try:
        root.mainloop()
    finally:
        app.cleanup()