"""
=============================================================================
  Flight Controller Serial Monitor  —  GUI Edition  v3.0
  flight_serial_monitor_gui.py

  Requirements:  pip install pyserial
  Run:           python flight_serial_monitor_gui.py
=============================================================================
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import threading
import time
import queue
import os
import sys
import datetime

# ── Auto-install pyserial if missing ─────────────────────────────────────────
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
    import serial.tools.list_ports


# ═════════════════════════════════════════════════════════════════════════════
#  THEME
# ═════════════════════════════════════════════════════════════════════════════
T = {
    "bg":      "#0d0f14",
    "panel":   "#13161e",
    "panel2":  "#1a1e28",
    "border":  "#252a38",
    "accent":  "#00c8ff",
    "acc2":    "#005f80",
    "green":   "#00e676",
    "yellow":  "#ffd740",
    "red":     "#ff5252",
    "orange":  "#ff9100",
    "magenta": "#e040fb",
    "grey":    "#546e7a",
    "text":    "#cfd8dc",
    "dim":     "#546e7a",
    "bright":  "#eceff1",
}

_win = sys.platform == "win32"
FM   = ("Courier New", 10)
FMs  = ("Courier New", 9)
FU   = ("Segoe UI", 10)        if _win else ("Helvetica", 10)
FUB  = ("Segoe UI", 10, "bold") if _win else ("Helvetica", 10, "bold")
FT   = ("Segoe UI", 12, "bold") if _win else ("Helvetica", 12, "bold")
FS   = ("Segoe UI", 9)         if _win else ("Helvetica", 9)


# ═════════════════════════════════════════════════════════════════════════════
#  Helpers
# ═════════════════════════════════════════════════════════════════════════════
def classify(line: str) -> str:
    lo = line.lower()
    if any(k in lo for k in ("fatal","error","err:","failed","fail")):         return "error"
    if any(k in lo for k in ("warn","caution")):                               return "warn"
    if any(k in lo for k in ("ok","saved","connected","ready","done","ip:")):  return "ok"
    if any(k in lo for k in ("[gcs]","gcs task","tcp server")):                return "gcs"
    if any(k in lo for k in ("auto on","auto off")):                           return "auto"
    if any(k in lo for k in ("pid","servo","imu","pitch","roll","yaw")):       return "flight"
    if any(k in lo for k in ("wifi","ssid","eeprom","credentials","mdns")):    return "wifi"
    if any(k in lo for k in ("calibrat","gyro","priming","offset")):           return "calib"
    if any(k in lo for k in ("booting","boot","===","flight controller")):     return "system"
    return "normal"

TAG_COL = {
    "error":  T["red"],     "warn":   T["yellow"],  "ok":     T["green"],
    "gcs":    T["accent"],  "auto":   T["orange"],  "flight": T["acc2"],
    "wifi":   T["magenta"], "calib":  T["grey"],    "system": T["bright"],
    "normal": T["text"],
}

KNOWN_VIDS = {0x10C4, 0x1A86, 0x0403, 0x239A, 0x303A}

def list_ports():
    return [(p.device, p.description or p.device)
            for p in serial.tools.list_ports.comports()]

def find_esp32_port():
    for p in serial.tools.list_ports.comports():
        if p.vid in KNOWN_VIDS:
            return p.device
        if p.description and any(k in p.description.lower()
                for k in ("ch340","cp210","ftdi","uart","usb serial","esp")):
            return p.device
    return None

def mk_btn(parent, text, cmd, bg=None, width=None):
    kw = dict(text=text, command=cmd,
              bg=bg or T["acc2"], fg=T["bright"],
              font=FUB, relief="flat", bd=0,
              padx=10, pady=6, cursor="hand2",
              activebackground=T["accent"], activeforeground="#000")
    if width:
        kw["width"] = width
    return tk.Button(parent, **kw)

def mk_lbl(parent, text=None, textvariable=None, font=None, fg=None, bg=None, **kw):
    cfg = dict(font=font or FU, fg=fg or T["text"], bg=bg or T["panel"])
    cfg.update(kw)
    if textvariable is not None:
        return tk.Label(parent, textvariable=textvariable, **cfg)
    return tk.Label(parent, text=text or "", **cfg)

def mk_sep(parent, pady=4):
    tk.Frame(parent, bg=T["border"], height=1).pack(fill="x", padx=14, pady=pady)


# ═════════════════════════════════════════════════════════════════════════════
#  SerialWorker  — background thread; fans decoded lines to watchers
# ═════════════════════════════════════════════════════════════════════════════
class SerialWorker(threading.Thread):
    def __init__(self, port, baud, rx_q):
        super().__init__(daemon=True)
        self.port      = port
        self.baud      = baud
        self.rx_q      = rx_q
        self._ser      = None
        self._stop     = threading.Event()
        self.connected = False
        self._wlock    = threading.Lock()
        self._watchers = []

    def add_watcher(self):
        q = queue.Queue()
        with self._wlock:
            self._watchers.append(q)
        return q

    def remove_watcher(self, q):
        with self._wlock:
            self._watchers = [w for w in self._watchers if w is not q]

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=0.1, write_timeout=2)
            self.connected = True
            self.rx_q.put(("STATUS", f"Connected  {self.port} @ {self.baud}"))
            buf = b""
            while not self._stop.is_set():
                try:
                    chunk = self._ser.read(self._ser.in_waiting or 1)
                    if chunk:
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            decoded = line.decode("utf-8", errors="replace").rstrip("\r")
                            if decoded:
                                self.rx_q.put(("LINE", decoded))
                                with self._wlock:
                                    for w in self._watchers:
                                        w.put(decoded)
                except serial.SerialException as e:
                    self.rx_q.put(("ERROR", str(e)))
                    break
        except serial.SerialException as e:
            self.rx_q.put(("ERROR", f"Cannot open {self.port}: {e}"))
        finally:
            self.connected = False
            if self._ser and self._ser.is_open:
                self._ser.close()
            self.rx_q.put(("STATUS", "Disconnected"))

    def send(self, text: str):
        """Send text + newline."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write((text + "\n").encode())
                self._ser.flush()
            except Exception:
                pass

    def send_raw(self, data: bytes):
        """Send raw bytes."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(data)
                self._ser.flush()
            except Exception:
                pass

    def stop(self):
        self._stop.set()


# ═════════════════════════════════════════════════════════════════════════════
#  Main Application
# ═════════════════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Flight Controller Serial Monitor")
        self.geometry("1080x720")
        self.minsize(820, 560)
        self.configure(bg=T["bg"])

        self._worker     = None
        self._rx_q       = queue.Queue()
        self._log_lines  = []
        self._auto_scroll = tk.BooleanVar(value=True)

        # WiFi state  (all managed on the main/GUI thread)
        self._menu_active   = False   # boot menu window is open
        self._change_active = False   # user clicked "Change WiFi", waiting for Send
        self._cd_job        = None    # after() job id for countdown
        self._cd_n          = 0       # remaining seconds

        self._build_ui()
        self._refresh_ports()
        self._poll_rx()

    # ─────────────────────────────────────────────────────────────────────────
    #  BUILD UI
    # ─────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self._build_titlebar()
        self._build_toolbar()

        pane = tk.PanedWindow(self, orient="horizontal",
                              bg=T["bg"], sashwidth=5, sashrelief="flat")
        pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        left  = tk.Frame(pane, bg=T["bg"])
        right = tk.Frame(pane, bg=T["bg"], width=330)
        pane.add(left,  minsize=420)
        pane.add(right, minsize=300)

        self._build_log_panel(left)
        self._build_right_tabs(right)
        self._build_statusbar()

    # ── Title bar ─────────────────────────────
    def _build_titlebar(self):
        bar = tk.Frame(self, bg=T["bg"])
        bar.pack(fill="x", padx=8, pady=(8, 0))

        ic = tk.Frame(bar, bg=T["accent"], width=34, height=34)
        ic.pack(side="left", padx=(0, 10))
        ic.pack_propagate(False)
        tk.Label(ic, text="✈", font=("Segoe UI", 16),
                 bg=T["accent"], fg="#000").place(relx=.5, rely=.5, anchor="center")

        tk.Label(bar, text="FLIGHT CONTROLLER",
                 font=("Courier New", 13, "bold"),
                 bg=T["bg"], fg=T["bright"]).pack(side="left")
        tk.Label(bar, text=" SERIAL MONITOR",
                 font=("Courier New", 13),
                 bg=T["bg"], fg=T["accent"]).pack(side="left")
        tk.Label(bar, text="  v3.0",
                 font=FS, bg=T["bg"], fg=T["dim"]).pack(side="left")

    # ── Toolbar ───────────────────────────────
    def _build_toolbar(self):
        bar = tk.Frame(self, bg=T["panel"], height=46)
        bar.pack(fill="x", padx=8, pady=(6, 4))
        bar.pack_propagate(False)

        mk_lbl(bar, "PORT:", fg=T["dim"], bg=T["panel"]).pack(side="left", padx=(10, 4))
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(bar, textvariable=self._port_var,
                                      width=20, font=FMs, state="readonly")
        self._port_cb.pack(side="left", padx=(0, 4), pady=8)
        mk_btn(bar, "↺", self._refresh_ports, bg=T["panel2"], width=2).pack(side="left", pady=8)

        mk_lbl(bar, "BAUD:", fg=T["dim"], bg=T["panel"]).pack(side="left", padx=(10, 4))
        self._baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self._baud_var,
                     values=["9600","19200","38400","57600","115200","230400"],
                     width=8, font=FMs, state="readonly"
                     ).pack(side="left", padx=(0, 10), pady=8)

        self._conn_btn = mk_btn(bar, "  CONNECT  ", self._toggle_connect, bg=T["green"])
        self._conn_btn.pack(side="left", padx=(0, 4), pady=8)

        mk_btn(bar, "CLEAR LOG",  self._clear_log,  bg=T["panel2"]).pack(side="left", padx=(0,4), pady=8)
        mk_btn(bar, "EXPORT LOG", self._export_log, bg=T["panel2"]).pack(side="left", padx=(0,4), pady=8)

        tk.Checkbutton(bar, text="Auto-scroll",
                       variable=self._auto_scroll,
                       font=FS, bg=T["panel"], fg=T["text"],
                       selectcolor=T["panel2"],
                       activebackground=T["panel"],
                       relief="flat", bd=0, cursor="hand2"
                       ).pack(side="right", padx=10)

        self._dot_cv = tk.Canvas(bar, width=14, height=14,
                                 bg=T["panel"], highlightthickness=0)
        self._dot_cv.pack(side="right", padx=(4, 0))
        self._dot = self._dot_cv.create_oval(2, 2, 12, 12, fill=T["red"], outline="")

    # ── Log panel (left side) ─────────────────
    def _build_log_panel(self, parent):
        # Header with filter
        hdr = tk.Frame(parent, bg=T["panel"], height=26)
        hdr.pack(fill="x", pady=(0, 2))
        hdr.pack_propagate(False)
        mk_lbl(hdr, "  ◉  SERIAL LOG",
               font=("Courier New", 9, "bold"),
               fg=T["accent"], bg=T["panel"]).pack(side="left", padx=4)
        mk_lbl(hdr, "Filter:", fg=T["dim"], bg=T["panel"]).pack(side="right", padx=(0, 4))
        self._filter_var = tk.StringVar()
        self._filter_var.trace_add("write", lambda *_: self._redraw_log())
        tk.Entry(hdr, textvariable=self._filter_var,
                 font=FMs, width=14,
                 bg=T["panel2"], fg=T["text"],
                 insertbackground=T["accent"],
                 relief="flat", bd=2
                 ).pack(side="right", padx=(0, 6), pady=3)

        # Scrolled log
        frame = tk.Frame(parent, bg=T["border"], bd=1)
        frame.pack(fill="both", expand=True)
        self._log = scrolledtext.ScrolledText(
            frame, bg=T["bg"], fg=T["text"], font=FM,
            insertbackground=T["accent"],
            selectbackground=T["acc2"],
            relief="flat", bd=0, state="disabled", wrap="word")
        self._log.pack(fill="both", expand=True, padx=1, pady=1)
        for tag, col in TAG_COL.items():
            self._log.tag_config(tag, foreground=col)
        self._log.tag_config("ts", foreground=T["dim"])

        # Command bar
        cmd_f = tk.Frame(parent, bg=T["panel2"], height=34)
        cmd_f.pack(fill="x", pady=(2, 0))
        cmd_f.pack_propagate(False)
        mk_lbl(cmd_f, " CMD›",
               font=("Courier New", 10, "bold"),
               fg=T["accent"], bg=T["panel2"]).pack(side="left")
        self._cmd_var = tk.StringVar()
        ce = tk.Entry(cmd_f, textvariable=self._cmd_var,
                      font=FM, bg=T["bg"], fg=T["text"],
                      insertbackground=T["accent"], relief="flat", bd=0)
        ce.pack(side="left", fill="x", expand=True, padx=4, pady=5)
        ce.bind("<Return>", lambda _: self._send_raw_cmd())
        mk_btn(cmd_f, "SEND", self._send_raw_cmd, bg=T["acc2"]).pack(side="right", padx=4, pady=4)

    # ── Right-side tabs ───────────────────────
    def _build_right_tabs(self, parent):
        style = ttk.Style()
        style.theme_use("default")
        style.configure("TNotebook", background=T["bg"], borderwidth=0)
        style.configure("TNotebook.Tab",
                        background=T["panel"], foreground=T["dim"],
                        font=FUB, padding=[10, 4])
        style.map("TNotebook.Tab",
                  background=[("selected", T["panel2"])],
                  foreground=[("selected", T["accent"])])

        nb = ttk.Notebook(parent)
        nb.pack(fill="both", expand=True)

        t_wifi  = tk.Frame(nb, bg=T["panel"])
        t_flash = tk.Frame(nb, bg=T["bg"])
        t_info  = tk.Frame(nb, bg=T["panel"])
        nb.add(t_wifi,  text=" WiFi Setup ")
        nb.add(t_flash, text=" ⚡ Flash Firmware ")
        nb.add(t_info,  text=" Info ")

        self._build_wifi_tab(t_wifi)
        self._build_flash_tab(t_flash)
        self._build_info_tab(t_info)

    # ─────────────────────────────────────────────────────────────────────────
    #  WIFI TAB
    # ─────────────────────────────────────────────────────────────────────────
    def _build_wifi_tab(self, parent):

        # ── Credential fields ─────────────────
        mk_lbl(parent, "WiFi Credentials",
               font=FT, fg=T["accent"]).pack(anchor="w", padx=14, pady=(10, 2))
        mk_lbl(parent,
               "📢Press Force Open before booting starts if buttons are not enabled.\n"
               "   Type SSID & password at any time.\n"
               "   They are sent only when you press Send.",
               font=FS, fg=T["dim"], justify="left"
               ).pack(anchor="w", padx=14, pady=(0, 6))
        mk_sep(parent)

        def cred_field(label_text):
            row = tk.Frame(parent, bg=T["panel"])
            row.pack(fill="x", padx=14, pady=5)
            mk_lbl(row, label_text, font=FUB, width=10, anchor="w").pack(side="left")
            v = tk.StringVar()
            e = tk.Entry(row, textvariable=v, font=FMs,
                         bg=T["panel2"], fg=T["text"],
                         insertbackground=T["accent"],
                         relief="flat", bd=2)
            e.pack(side="left", fill="x", expand=True, ipady=5)
            return v

        self._ssid_var = cred_field("SSID")
        self._pass_var = cred_field("Password")

        mk_sep(parent, pady=(8, 4))

        # ── Countdown + manual trigger row ────
        cd_row = tk.Frame(parent, bg=T["panel"])
        cd_row.pack(fill="x", padx=14, pady=(6, 2))

        self._cd_var = tk.StringVar(value="Waiting for ESP32 boot menu…")
        mk_lbl(cd_row, textvariable=self._cd_var,
               font=("Courier New", 9, "bold"),
               fg=T["yellow"]).pack(side="left", fill="x", expand=True)

        # Manual trigger: activates buttons immediately (use when ESP32 is already
        # in the boot window and you missed the auto-detect)
        mk_btn(cd_row, "⚡ Force Open", self._force_open_menu,
               bg=T["panel2"]).pack(side="right")

        # ── Option buttons ────────────────────
        opt_row = tk.Frame(parent, bg=T["panel"])
        opt_row.pack(fill="x", padx=14, pady=6)

        self._btn_view   = mk_btn(opt_row, "1  View SSID",   self._do_view,   bg=T["panel2"])
        self._btn_change = mk_btn(opt_row, "2  Change WiFi", self._do_change, bg=T["acc2"])
        self._btn_view.pack(side="left", padx=(0, 8))
        self._btn_change.pack(side="left")

        # Start disabled — enabled only when boot menu is detected
        self._btn_view.config(state="disabled")
        self._btn_change.config(state="disabled")

        mk_sep(parent, pady=(6, 4))

        # ── Send credentials button ───────────
        self._send_hint_var = tk.StringVar(value="")
        mk_lbl(parent, textvariable=self._send_hint_var,
               font=FS, fg=T["dim"], wraplength=280, justify="left"
               ).pack(anchor="w", padx=14, pady=(0, 4))

        self._btn_send = mk_btn(parent, "✔  Send Credentials to ESP32",
                                self._do_send, bg=T["green"])
        self._btn_send.pack(anchor="w", padx=14, pady=(0, 8))
        self._btn_send.config(state="disabled")

        mk_sep(parent)

        # ── Status log ────────────────────────
        mk_lbl(parent, "Status", font=FUB, fg=T["dim"]).pack(anchor="w", padx=14, pady=(4, 0))
        self._wlog = tk.Text(parent, height=7,
                             bg=T["bg"], fg=T["text"], font=FMs,
                             relief="flat", bd=1,
                             state="disabled", wrap="word")
        self._wlog.pack(fill="x", padx=14, pady=4)
        self._wlog.tag_config("ok",   foreground=T["green"])
        self._wlog.tag_config("err",  foreground=T["red"])
        self._wlog.tag_config("info", foreground=T["accent"])
        self._wlog.tag_config("warn", foreground=T["yellow"])

        mk_btn(parent, "CLEAR STATUS", self._clear_wlog,
               bg=T["panel2"]).pack(anchor="w", padx=14, pady=(0, 10))

    # ── Flash tab ──────────────────────────────────────────────────────
    def _build_flash_tab(self, parent):
        """Embed the FlashTab widget. Loads flash_tab.py from same folder."""
        try:
            import importlib.util, os
            script_dir = os.path.dirname(os.path.abspath(__file__))
            spec = importlib.util.spec_from_file_location(
                "flash_tab",
                os.path.join(script_dir, "flash_tab.py"))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            ft = mod.FlashTab(parent)
            ft.tab.pack(fill="both", expand=True)
        except Exception as e:
            tk.Label(parent,
                     text=f"Flash tab unavailable:\n{e}\n\n"
                          "Ensure flash_tab.py is in the same folder.",
                     bg=T["bg"], fg=T["red"],
                     font=("Consolas", 10), justify="left"
                     ).pack(padx=20, pady=20, anchor="w")

    # ── Info tab ──────────────────────────────
    def _build_info_tab(self, parent):
        rows = [
            ("FLIGHT CONTROLLER MONITOR", T["accent"],  FT),
            ("v3.0",                      T["dim"],     FS),
            ("",                          T["text"],    FU),
            ("COLOUR LEGEND",             T["bright"],  FUB),
            ("  ● Errors / Fatal",        T["red"],     FU),
            ("  ● Warnings",              T["yellow"],  FU),
            ("  ● OK / Connected",        T["green"],   FU),
            ("  ● GCS / TCP",             T["accent"],  FU),
            ("  ● Auto pilot",            T["orange"],  FU),
            ("  ● PID / IMU",             T["acc2"],    FU),
            ("  ● WiFi / EEPROM",         T["magenta"], FU),
            ("  ● Calibration",           T["grey"],    FU),
            ("",                          T["text"],    FU),
            ("QUICK GCS COMMANDS",        T["bright"],  FUB),
            ("  getStatus",               T["text"],    FM),
            ("  getPitchPID / getRollPID",T["text"],    FM),
            ("  setPitchP2.5",            T["text"],    FM),
            ("  save",                    T["text"],    FM),
            ("  calibrate",               T["text"],    FM),
            ("  swapAxes",                T["text"],    FM),
            ("  invertPitch / invertRoll",T["text"],    FM),
            ("  startCubeStream",         T["text"],    FM),
            ("  stopCubeStream",          T["text"],    FM),
        ]
        for text, color, font in rows:
            tk.Label(parent, text=text, font=font,
                     bg=T["panel"], fg=color,
                     anchor="w", justify="left"
                     ).pack(anchor="w", padx=14, pady=1)

    # ── Status bar ────────────────────────────
    def _build_statusbar(self):
        bar = tk.Frame(self, bg=T["panel"], height=22)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        self._status_var = tk.StringVar(value="Not connected")
        mk_lbl(bar, textvariable=self._status_var,
               font=FS, fg=T["dim"], bg=T["panel"],
               anchor="w").pack(side="left", padx=8)

        self._lcount_var = tk.StringVar(value="Lines: 0")
        mk_lbl(bar, textvariable=self._lcount_var,
               font=FS, fg=T["dim"], bg=T["panel"]).pack(side="right", padx=8)

        self._time_var = tk.StringVar()
        mk_lbl(bar, textvariable=self._time_var,
               font=FS, fg=T["dim"], bg=T["panel"]).pack(side="right", padx=8)
        self._tick_clock()

    def _tick_clock(self):
        self._time_var.set(datetime.datetime.now().strftime("%H:%M:%S"))
        self.after(1000, self._tick_clock)

    # ─────────────────────────────────────────────────────────────────────────
    #  CONNECT / DISCONNECT
    # ─────────────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports  = list_ports()
        labels = [f"{d}  —  {n}" for d, n in ports]
        self._port_cb["values"] = labels
        if labels:
            auto = find_esp32_port()
            if auto:
                for i, (d, _) in enumerate(ports):
                    if d == auto:
                        self._port_cb.current(i)
                        break
            else:
                self._port_cb.current(0)

    def _get_port(self):
        v = self._port_var.get()
        return v.split("  —  ")[0].strip() if v else None

    def _toggle_connect(self):
        if self._worker and self._worker.connected:
            self._disconnect()
        else:
            self._do_connect()

    def _do_connect(self):
        port = self._get_port()
        if not port:
            messagebox.showerror("No Port", "Select a serial port first.")
            return
        try:
            baud = int(self._baud_var.get())
        except ValueError:
            baud = 115200
        self._worker = SerialWorker(port, baud, self._rx_q)
        # Register watcher queue BEFORE start() so no lines are ever missed
        self._boot_wq = self._worker.add_watcher()
        self._worker.start()
        self._conn_btn.config(text="  DISCONNECT  ", bg=T["red"])
        # Start boot-menu watcher, passing the pre-registered queue
        threading.Thread(target=self._boot_menu_watcher_thread,
                         args=(self._boot_wq,), daemon=True).start()

    def _disconnect(self):
        if self._worker:
            self._worker.stop()
            self._worker = None
        self._wifi_reset()
        self._conn_btn.config(text="  CONNECT  ", bg=T["green"])
        self._dot_cv.itemconfig(self._dot, fill=T["red"])
        self._status_var.set("Not connected")

    # ─────────────────────────────────────────────────────────────────────────
    #  RX QUEUE POLL  (runs on main thread every 40 ms)
    # ─────────────────────────────────────────────────────────────────────────
    def _poll_rx(self):
        try:
            while True:
                kind, data = self._rx_q.get_nowait()
                if kind == "LINE":
                    self._append_log(data)
                elif kind == "STATUS":
                    self._status_var.set(data)
                    up = "Connected" in data
                    self._dot_cv.itemconfig(self._dot, fill=T["green"] if up else T["red"])
                    if not up:
                        self._conn_btn.config(text="  CONNECT  ", bg=T["green"])
                        self._wifi_reset()
                elif kind == "ERROR":
                    self._append_log(f"[ERROR] {data}", "error")
                    self._status_var.set(f"Error: {data}")
        except queue.Empty:
            pass
        self.after(40, self._poll_rx)

    # ─────────────────────────────────────────────────────────────────────────
    #  LOG HELPERS
    # ─────────────────────────────────────────────────────────────────────────
    def _append_log(self, text: str, force_tag: str = None):
        filt = self._filter_var.get().lower()
        if filt and filt not in text.lower():
            return
        tag = force_tag or classify(text)
        ts  = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_lines.append((ts, text))

        self._log.config(state="normal")
        self._log.insert("end", f"[{ts}]  ", "ts")
        self._log.insert("end", text + "\n", tag)
        self._log.config(state="disabled")
        if self._auto_scroll.get():
            self._log.see("end")

        n = int(self._lcount_var.get().split(": ")[1]) + 1
        self._lcount_var.set(f"Lines: {n}")

    def _redraw_log(self):
        filt = self._filter_var.get().lower()
        self._log.config(state="normal")
        self._log.delete("1.0", "end")
        for ts, text in self._log_lines:
            if filt and filt not in text.lower():
                continue
            self._log.insert("end", f"[{ts}]  ", "ts")
            self._log.insert("end", text + "\n", classify(text))
        self._log.config(state="disabled")
        if self._auto_scroll.get():
            self._log.see("end")

    def _clear_log(self):
        self._log.config(state="normal")
        self._log.delete("1.0", "end")
        self._log.config(state="disabled")
        self._log_lines.clear()
        self._lcount_var.set("Lines: 0")

    def _export_log(self):
        if not self._log_lines:
            messagebox.showinfo("Export", "No log lines to export.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text", "*.txt"), ("All", "*.*")],
            initialfile=f"flight_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            for ts, text in self._log_lines:
                f.write(f"[{ts}]  {text}\n")
        messagebox.showinfo("Export", f"Saved to:\n{path}")

    def _send_raw_cmd(self):
        cmd = self._cmd_var.get().strip()
        if not cmd:
            return
        if not (self._worker and self._worker.connected):
            messagebox.showwarning("Not Connected", "Connect first.")
            return
        self._worker.send(cmd)
        self._append_log(f">>> {cmd}", "ok")
        self._cmd_var.set("")

    # ─────────────────────────────────────────────────────────────────────────
    #  WIFI TAB  —  STATUS LOG
    # ─────────────────────────────────────────────────────────────────────────
    def _wlog_write(self, text: str, tag: str = "info"):
        self._wlog.config(state="normal")
        self._wlog.insert("end", text + "\n", tag)
        self._wlog.config(state="disabled")
        self._wlog.see("end")

    def _clear_wlog(self):
        self._wlog.config(state="normal")
        self._wlog.delete("1.0", "end")
        self._wlog.config(state="disabled")

    # ─────────────────────────────────────────────────────────────────────────
    #  WIFI TAB  —  STATE MACHINE
    #
    #  IDLE          → buttons disabled, countdown shows "Waiting…"
    #  MENU_OPEN     → option buttons enabled, 10-s countdown ticking
    #  CHANGE_READY  → option buttons disabled, Send button enabled
    #  IDLE (reset)  → after send completes / countdown expires / disconnect
    # ─────────────────────────────────────────────────────────────────────────
    def _wifi_reset(self):
        """Return to IDLE state."""
        self._menu_active   = False
        self._change_active = False
        if self._cd_job:
            self.after_cancel(self._cd_job)
            self._cd_job = None
        self._cd_n = 0
        self._cd_var.set("Waiting for ESP32 boot menu…")
        self._btn_view.config(  state="disabled", bg=T["panel2"])
        self._btn_change.config(state="disabled", bg=T["acc2"])
        self._btn_send.config(  state="disabled", bg=T["green"])
        self._send_hint_var.set("")

    def _wifi_open_menu(self):
        """
        Called (on main thread) when the boot menu banner is detected.
        Transitions to MENU_OPEN state.
        """
        # Reset any previous state cleanly
        if self._cd_job:
            self.after_cancel(self._cd_job)
            self._cd_job = None

        self._menu_active   = True
        self._change_active = False

        # Enable option buttons
        self._btn_view.config(  state="normal", bg=T["panel2"])
        self._btn_change.config(state="normal", bg=T["acc2"])
        self._btn_send.config(  state="disabled")
        self._send_hint_var.set("")

        self._wlog_write("✓ Boot menu detected!", "ok")
        self._wlog_write("Click an option within 10 s.", "info")

        # Start 10-second countdown
        self._cd_n  = 10
        self._cd_job = self.after(0, self._cd_tick)

    def _cd_tick(self):
        """Countdown tick — runs on main thread via after()."""
        if not self._menu_active:
            return   # aborted by a button click
        n = self._cd_n
        if n <= 0:
            self._cd_var.set("⏱  Time's up — booting normally")
            self._btn_view.config(  state="disabled")
            self._btn_change.config(state="disabled")
            self._menu_active = False
            if not self._change_active:
                self._wlog_write("No option chosen — ESP32 booting normally.", "warn")
            return
        self._cd_var.set(f"⏱  Choose within  {n}  second{'s' if n != 1 else ''}…")
        self._cd_n  -= 1
        self._cd_job = self.after(1000, self._cd_tick)

    # ── Force-open (manual fallback) ─────────
    def _force_open_menu(self):
        """Manually activate the option buttons — use when auto-detect was missed."""
        self._wifi_open_menu()
        self._wlog_write("Manual trigger — make sure ESP32 is in boot window!", "warn")

    # ── Button: 1  View SSID ─────────────────
    def _do_view(self):
        if not self._menu_active:
            return
        # Stop countdown, disable option buttons
        self._menu_active = False
        if self._cd_job:
            self.after_cancel(self._cd_job)
            self._cd_job = None
        self._cd_var.set("")
        self._btn_view.config(  state="disabled")
        self._btn_change.config(state="disabled")
        # Send '2' → firmware prints current SSID + password then boots
        if self._worker:
            self._worker.send_raw(b"2\n")
        self._wlog_write("Sent: View SSID & password.", "info")
        self._wlog_write("Result will appear in the main log.", "info")

    # ── Button: 2  Change WiFi ────────────────
    def _do_change(self):
        if not self._menu_active:
            return
        # Stop countdown, disable option buttons
        self._menu_active   = False
        self._change_active = True
        if self._cd_job:
            self.after_cancel(self._cd_job)
            self._cd_job = None
        self._cd_var.set("")
        self._btn_view.config(  state="disabled")
        self._btn_change.config(state="disabled")

        # Send '1' → firmware enters case '1', immediately calls serialReadLine
        # waiting for SSID.  User now has unlimited time to press Send.
        # Small delay so the firmware has exited Serial.println() and entered
        # the countdown loop before our byte arrives in the RX buffer.
        if self._worker:
            time.sleep(0.05)
            self._worker.send_raw(b"1\n")

        ssid = self._ssid_var.get().strip()
        self._wlog_write("Sent: Change WiFi selected.", "info")
        if ssid:
            self._wlog_write(f"SSID ready: {ssid}", "ok")
        else:
            self._wlog_write("⚠  SSID field is empty — fill it in above.", "warn")
        self._wlog_write("Edit credentials if needed, then press Send.", "info")

        self._send_hint_var.set(
            "Firmware is waiting. Fill SSID & password above, then press Send.")
        self._btn_send.config(state="normal")

    # ── Button: Send Credentials ──────────────
    def _do_send(self):
        ssid = self._ssid_var.get().strip()
        pwd  = self._pass_var.get()     # blank = open network — valid

        if not ssid:
            messagebox.showerror("Missing SSID", "SSID cannot be empty.")
            return
        if not (self._worker and self._worker.connected):
            messagebox.showwarning("Not Connected", "ESP32 is not connected.")
            return

        # Disable send button immediately so user can't double-click
        self._btn_send.config(state="disabled")
        self._send_hint_var.set("Sending…")
        self._change_active = False

        threading.Thread(target=self._send_creds_thread,
                         args=(ssid, pwd), daemon=True).start()

    def _send_creds_thread(self, ssid: str, pwd: str):
        """Background thread: sends SSID → password → 'y' watching for prompts."""
        worker = self._worker
        if not worker:
            return

        # Helper: schedule wlog on main thread (lambda with default args avoids closure bug)
        def wlog(msg, tag="info"):
            self.after(0, lambda m=msg, t=tag: self._wlog_write(m, t))

        def hint(msg):
            self.after(0, lambda m=msg: self._send_hint_var.set(m))

        wq = worker.add_watcher()

        def wait_for(keywords, timeout):
            deadline = time.time() + timeout
            while time.time() < deadline:
                rem = max(0.0, deadline - time.time())
                try:
                    line = wq.get(timeout=min(0.15, rem))
                    if any(k.lower() in line.lower() for k in keywords):
                        return True, line
                except queue.Empty:
                    pass
            return False, ""

        try:
            # Firmware is inside case '1', about to call serialReadLine for SSID.
            # Wait for the SSID prompt to confirm firmware is ready, then send.
            wait_for(["Enter new SSID"], 5.0)
            time.sleep(0.1)   # tiny margin for serialReadLine to start blocking

            wlog(f"Sending SSID: {ssid}")
            worker.send(ssid)

            # Wait for password prompt
            found, _ = wait_for(["Enter new Password", "Password"], 8.0)
            if not found:
                wlog("⚠  No password prompt received — sending anyway.", "warn")
            time.sleep(0.1)

            wlog("Sending password…")
            worker.send(pwd)

            # Wait for confirmation prompt
            found, _ = wait_for(["Save?", "(y/n)", "Confirm:"], 8.0)
            if not found:
                wlog("⚠  No confirm prompt — sending 'y' anyway.", "warn")
            time.sleep(0.1)

            wlog("Confirming save…")
            worker.send("y")

            # Wait for final result
            found, line = wait_for(
                ["credentials saved", "saved. continuing",
                 "unchanged", "cancelled"], 10.0)

            if found and "saved" in line.lower():
                wlog("✓ Credentials saved to ESP32!", "ok")
                wlog(f"  SSID: {ssid}", "ok")
                wlog("  New credentials active on next boot.", "ok")
                hint("✓ Done!")
            elif found:
                wlog(f"ESP32: {line}", "warn")
                hint("")
            else:
                wlog("Timed out waiting for ESP32 confirmation.", "warn")
                wlog("Check the main log for the ESP32 response.", "warn")
                hint("")

        except Exception as e:
            wlog(f"Error: {e}", "err")
            hint("")
        finally:
            worker.remove_watcher(wq)

    # ─────────────────────────────────────────────────────────────────────────
    #  BOOT MENU WATCHER  (one background thread per connection)
    #
    #  Watches every decoded line via a watcher queue.
    #  When it sees the firmware banner it calls _wifi_open_menu() on the
    #  main thread via self.after(0, ...).
    # ─────────────────────────────────────────────────────────────────────────
    def _boot_menu_watcher_thread(self, wq):
        """
        Watches the serial stream for the ESP32 boot menu banner.
        wq is a watcher queue registered BEFORE worker.start() so no lines
        are ever missed, even if the ESP32 boots faster than this thread starts.
        """
        worker = self._worker
        if not worker:
            return
        try:
            while worker.connected:
                try:
                    line = wq.get(timeout=0.5)
                    lo   = line.lower()
                    if ("flight controller config" in lo or
                            "waiting for option" in lo or
                            "1 = change wifi" in lo):
                        self.after(0, self._wifi_open_menu)
                except queue.Empty:
                    pass
        finally:
            worker.remove_watcher(wq)

    # ─────────────────────────────────────────────────────────────────────────
    #  DIAGNOSTICS
    # ─────────────────────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    #  CLOSE
    # ─────────────────────────────────────────────────────────────────────────
    def on_close(self):
        if self._worker:
            self._worker.stop()
        self.destroy()


# ═════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    if sys.platform == "win32":
        try:
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
        os.system("")

    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()