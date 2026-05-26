# gcs/servo_tab.py — RC Input tab with bidirectional centre bars
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from gcs.theme import TH

BAR_W = 320
BAR_H = 32
CX    = BAR_W // 2


class _BiBar:
    """Bidirectional bar: centre = 50%, left = negative, right = positive."""

    TRACK  = TH["panel2"]
    CLINE  = "#ffffff"
    TICK   = "#333355"
    TXT    = "#e0e0ff"

    def __init__(self, canvas: tk.Canvas, fill_pos: str, fill_neg: str):
        self.cv       = canvas
        self.fill_pos = fill_pos
        self.fill_neg = fill_neg

        canvas.create_rectangle(0, 0, BAR_W, BAR_H,
                                 fill=self.TRACK, outline="")
        for frac in (0.25, 0.75):
            x = int(BAR_W * frac)
            canvas.create_line(x, 4, x, BAR_H - 4,
                               fill=self.TICK, width=1)

        self._fill  = canvas.create_rectangle(CX, 4, CX, BAR_H - 4,
                                               fill=fill_pos, outline="")
        self._cline = canvas.create_line(CX, 0, CX, BAR_H,
                                          fill=self.CLINE, width=2)
        self._text  = canvas.create_text(BAR_W - 6, BAR_H // 2,
                                          text="  0%", fill=self.TXT,
                                          font=("Consolas", 9), anchor="e")

    def update(self, pct: int):
        pct    = max(0, min(100, pct))
        offset = int((pct - 50) / 50.0 * CX)
        if offset >= 0:
            self.cv.coords(self._fill, CX, 4, CX + offset, BAR_H - 4)
            self.cv.itemconfig(self._fill, fill=self.fill_pos)
        else:
            self.cv.coords(self._fill, CX + offset, 4, CX, BAR_H - 4)
            self.cv.itemconfig(self._fill, fill=self.fill_neg)
        signed = pct - 50
        sign   = "+" if signed >= 0 else ""
        self.cv.itemconfig(self._text, text=f"{sign}{signed:3d}%")


class ServoTab:
    """🎮  RC Input — live stick positions with bidirectional bars."""

    _CHANNELS = [
        ("CH1  Roll", "#2ecc71", "#e74c3c"),
        ("CH2  Pitch",  "#3498db", "#e67e22"),
        ("CH3  Yaw",   "#9b59b6", "#9b59b6"),
    ]

    def __init__(self, parent, connection):
        self.connection                = connection
        self.tab                       = tk.Frame(parent, bg=TH["bg"])
        self.pwm_stream_thread_started = False
        self._thread                   = None
        self._stop_event               = threading.Event()
        self._bibars: list[_BiBar]     = []
        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        root = self.tab

        # Header
        hdr = tk.Frame(root, bg=TH["bg"])
        hdr.pack(fill="x", padx=14, pady=(14, 4))
        tk.Label(hdr, text="RC Receiver  —  Stick Positions",
                 bg=TH["bg"], fg=TH["accent"],
                 font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(hdr,
                 text="Centre line = neutral (50 %).   "
                      "Positive deflection fills right.   "
                      "Negative deflection fills left.",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")
        tk.Frame(root, bg=TH["border"], height=1).pack(
            fill="x", padx=14, pady=6)

        # Bars grid
        grid = tk.Frame(root, bg=TH["bg"])
        grid.pack(padx=14, pady=4, anchor="w")

        # Column header
        tk.Label(grid, text="", width=12, bg=TH["bg"]).grid(
            row=0, column=0)
        tk.Label(grid,
                 text=f"{'← negative':<22}│{'positive →':>22}",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Consolas", 8)
                 ).grid(row=0, column=1, sticky="w")

        for i, (label, fp, fn) in enumerate(self._CHANNELS):
            tk.Label(grid, text=label,
                     bg=TH["bg"], fg=TH["text"],
                     font=("Consolas", 10), width=12, anchor="w"
                     ).grid(row=i + 1, column=0, padx=(0, 8), pady=8)

            cv = tk.Canvas(grid, width=BAR_W, height=BAR_H,
                           bg=TH["panel2"],
                           highlightthickness=1,
                           highlightbackground=TH["border"])
            cv.grid(row=i + 1, column=1, pady=8)
            self._bibars.append(_BiBar(cv, fill_pos=fp, fill_neg=fn))

        tk.Frame(root, bg=TH["border"], height=1).pack(
            fill="x", padx=14, pady=6)

        # Autopilot + signal row
        info = tk.Frame(root, bg=TH["bg"])
        info.pack(fill="x", padx=14, pady=4)

        tk.Label(info, text="Autopilot  (CH4):",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Consolas", 10)).pack(side="left", padx=(0, 8))

        self._ap_cv = tk.Canvas(info, width=120, height=28,
                                 bg=TH["panel2"],
                                 highlightthickness=1,
                                 highlightbackground=TH["border"])
        self._ap_cv.pack(side="left")
        self._ap_fill = self._ap_cv.create_rectangle(
            0, 0, 0, 28, fill=TH["panel2"], outline="")
        self._ap_text = self._ap_cv.create_text(
            60, 14, text="—", fill=TH["dim"],
            font=("Consolas", 11, "bold"))

        tk.Label(info, text="   Signal:",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Consolas", 10)).pack(side="left", padx=(16, 6))
        self._sig_lbl = tk.Label(info, text="● No signal",
                                  bg=TH["bg"], fg=TH["red"],
                                  font=("Consolas", 11, "bold"))
        self._sig_lbl.pack(side="left")

        # Stream button
        bf = tk.Frame(root, bg=TH["bg"])
        bf.pack(padx=14, pady=(16, 10), anchor="w")

        self._stream_btn = tk.Button(
            bf, text="▶  Start PWM Stream",
            command=self._toggle_stream,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", bd=0, padx=14, pady=6,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000")
        self._stream_btn.pack()

    # ------------------------------------------------------------------
    def _toggle_stream(self):
        if self.pwm_stream_thread_started:
            self.stop_pwm_stream_and_send()
        else:
            self.start_pwm_stream()

    def start_pwm_stream(self):
        self.pwm_stream_thread_started = True
        self._stream_btn.config(text="⏹  Stop PWM Stream",
                                bg=TH["red"],
                                activebackground="#ff8080")
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._stream_worker, daemon=True)
        self._thread.start()

    def stop_pwm_stream(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None
        self.pwm_stream_thread_started = False
        self._stream_btn.config(text="▶  Start PWM Stream",
                                bg=TH["acc2"],
                                activebackground=TH["accent"])

    def stop_pwm_stream_and_send(self):
        try:
            self.connection.send_command("stopPWMStream", timeout=1.0)
        except Exception:
            pass
        self.stop_pwm_stream()

    # ------------------------------------------------------------------
    def _stream_worker(self):
        resp = self.connection.send_command("startPWMStream", timeout=2.0)
        if resp != "PWM_STREAM_START":
            self.tab.after(0, lambda: messagebox.showerror(
                "Stream Error",
                f"Expected PWM_STREAM_START, got {resp!r}"))
            self.tab.after(0, self.stop_pwm_stream)
            return

        buf = ""
        while not self._stop_event.is_set():
            chunk = self.connection.recv_data(timeout=0.05)
            if not chunk:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                if "No signal" in line:
                    self.tab.after(0, self._flag_ap_off)
                else:
                    try:
                        pcts = [max(0, min(100, int(x)))
                                for x in line.split(",")]
                        while len(pcts) < 4:
                            pcts.append(0)
                        self.tab.after(0, self._update_bars, pcts)
                    except ValueError:
                        continue

    # ------------------------------------------------------------------
    def _update_bars(self, pcts: list):
        self._sig_lbl.config(text="● Signal OK", fg=TH["green"])
        for i, bar in enumerate(self._bibars):
            bar.update(pcts[i])
        ap = pcts[3] if len(pcts) > 3 else 0
        if ap >= 90:
            self._ap_cv.coords(self._ap_fill, 0, 0, 120, 28)
            self._ap_cv.itemconfig(self._ap_fill, fill=TH["green"])
            self._ap_cv.itemconfig(self._ap_text,
                                    text="AUTO  ON", fill="#000")
        elif ap <= 10:
            self._ap_cv.coords(self._ap_fill, 0, 0, 60, 28)
            self._ap_cv.itemconfig(self._ap_fill, fill=TH["red"])
            self._ap_cv.itemconfig(self._ap_text,
                                    text="AUTO OFF", fill=TH["bright"])
        else:
            self._ap_cv.coords(self._ap_fill, 0, 0, 0, 28)
            self._ap_cv.itemconfig(self._ap_text,
                                    text="  —  ", fill=TH["dim"])

    def _flag_ap_off(self):
        """Firmware sends 'No signal' when CH4=0 (auto switch OFF).
        Keep stick bars as-is, just update the autopilot indicator."""
        self._sig_lbl.config(text="● RC Active", fg=TH["green"])
        self._ap_cv.coords(self._ap_fill, 0, 0, 60, 28)
        self._ap_cv.itemconfig(self._ap_fill, fill=TH["red"])
        self._ap_cv.itemconfig(self._ap_text,
                                text="AUTO OFF", fill=TH["bright"])
