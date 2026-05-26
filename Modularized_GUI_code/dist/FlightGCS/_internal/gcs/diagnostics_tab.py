# gcs/diagnostics_tab.py — Diagnostics tab
import tkinter as tk
from tkinter import ttk, messagebox
import threading

from gcs.theme import TH, mk_sep, mk_section_header


class DiagnosticsTab:
    """📊  Diagnostics — live angles, axis config, calibration, deadbands."""

    POLL_MS = 200

    def __init__(self, parent, connection):
        self.connection = connection
        self.tab        = tk.Frame(parent, bg=TH["bg"])
        self._polling   = False
        self._poll_job  = None

        self._pitch_var    = tk.StringVar(value="--")
        self._roll_var     = tk.StringVar(value="--")
        self._yaw_var      = tk.StringVar(value="--")
        self._auto_var     = tk.StringVar(value="--")
        self._swap_var     = tk.StringVar(value="--")
        self._psign_var    = tk.StringVar(value="--")
        self._rsign_var    = tk.StringVar(value="--")
        self._pid_db_var   = tk.DoubleVar(value=0.8)
        self._servo_db_var = tk.DoubleVar(value=1.0)

        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        root = self.tab

        # Header
        hdr = tk.Frame(root, bg=TH["bg"])
        hdr.pack(fill="x", padx=14, pady=(14, 4))
        tk.Label(hdr, text="Diagnostics & Calibration",
                 bg=TH["bg"], fg=TH["accent"],
                 font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(hdr,
                 text="Live sensor angles, axis orientation, "
                      "gyro calibration and deadband tuning.",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")
        mk_sep(root)

        # ── Scrollable canvas ─────────────────────────────────────────
        self._canvas = tk.Canvas(root, bg=TH["bg"], highlightthickness=0)
        sb = ttk.Scrollbar(root, orient="vertical",
                           command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(self._canvas, bg=TH["bg"])
        self._win_id = self._canvas.create_window(
            (0, 0), window=inner, anchor="nw")

        # Resize inner frame to match canvas width
        self._canvas.bind("<Configure>",
            lambda e: self._canvas.itemconfig(
                self._win_id, width=e.width))

        # Update scrollregion whenever inner frame changes size
        inner.bind("<Configure>",
            lambda e: self._canvas.configure(
                scrollregion=self._canvas.bbox("all")))

        # ── Trackpad / mouse wheel scroll ─────────────────────────────
        # Bind to canvas AND every child widget recursively so
        # two-finger trackpad drag scrolls from anywhere on the tab
        def _scroll(event):
            # Windows mouse wheel
            if event.num == 4 or event.delta > 0:
                self._canvas.yview_scroll(-1, "units")
            elif event.num == 5 or event.delta < 0:
                self._canvas.yview_scroll(1, "units")

        def _bind_scroll(widget):
            widget.bind("<MouseWheel>", _scroll)   # Windows / macOS
            widget.bind("<Button-4>",   _scroll)   # Linux scroll up
            widget.bind("<Button-5>",   _scroll)   # Linux scroll down
            for child in widget.winfo_children():
                _bind_scroll(child)

        # Re-bind after all widgets are built
        def _bind_all_after():
            _bind_scroll(self._canvas)
            _bind_scroll(inner)
        root.after(100, _bind_all_after)

        self._inner      = inner
        self._bind_scroll_fn = _bind_scroll

        self._build_live(inner)
        self._build_axis(inner)
        self._build_calibration(inner)
        self._build_deadbands(inner)
        self._build_save(inner)

    # ── Live angles ────────────────────────────────────────────────────
    def _build_live(self, parent):
        sec = mk_section_header(parent, "Live Sensor Angles")
        inner = tk.Frame(sec, bg=TH["panel"])
        inner.pack(fill="x", padx=10, pady=(0, 8))

        angle_row = tk.Frame(inner, bg=TH["panel"])
        angle_row.pack(fill="x", pady=4)

        for label, var, col in [
            ("PITCH", self._pitch_var, TH["accent"]),
            ("ROLL",  self._roll_var,  TH["green"]),
            ("YAW",   self._yaw_var,   TH["yellow"]),
        ]:
            cell = tk.Frame(angle_row, bg=TH["panel2"],
                            width=130, height=66)
            cell.pack(side="left", padx=6, pady=4)
            cell.pack_propagate(False)
            tk.Label(cell, text=label,
                     bg=TH["panel2"], fg=TH["dim"],
                     font=("Segoe UI", 9, "bold")
                     ).place(relx=.5, y=8, anchor="n")
            tk.Label(cell, textvariable=var,
                     bg=TH["panel2"], fg=col,
                     font=("Courier New", 15, "bold")
                     ).place(relx=.5, rely=.65, anchor="center")

        # State pills
        pill_row = tk.Frame(inner, bg=TH["panel"])
        pill_row.pack(fill="x", pady=(6, 0))

        for txt, var in [("Mode",       self._auto_var),
                          ("Axis Swap",  self._swap_var),
                          ("Pitch Sign", self._psign_var),
                          ("Roll Sign",  self._rsign_var)]:
            f = tk.Frame(pill_row, bg=TH["panel2"], padx=10, pady=3)
            f.pack(side="left", padx=4, pady=2)
            tk.Label(f, text=txt + ":",
                     bg=TH["panel2"], fg=TH["dim"],
                     font=("Segoe UI", 9)).pack(side="left")
            tk.Label(f, textvariable=var,
                     bg=TH["panel2"], fg=TH["text"],
                     font=("Consolas", 9, "bold"), width=7
                     ).pack(side="left", padx=(4, 0))

        # Poll + Read buttons on same row
        btn_row = tk.Frame(inner, bg=TH["panel"])
        btn_row.pack(anchor="w", pady=8)

        self._poll_btn = tk.Button(btn_row,
            text="▶  Start Live Feed",
            command=self._toggle_poll,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", padx=12, pady=5,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000")
        self._poll_btn.pack(side="left", padx=(0, 8))



    # ── Axis orientation ───────────────────────────────────────────────
    def _build_axis(self, parent):
        sec = mk_section_header(parent, "Axis Orientation")
        inner = tk.Frame(sec, bg=TH["panel"])
        inner.pack(fill="x", padx=10, pady=(0, 8))

        tk.Label(inner,
                 text="If the wrong servo responds or correction is inverted,\n"
                      "use these buttons. Changes apply immediately.",
                 bg=TH["panel"], fg=TH["dim"],
                 font=("Segoe UI", 9), justify="left"
                 ).pack(anchor="w", pady=(0, 8))

        row = tk.Frame(inner, bg=TH["panel"])
        row.pack(anchor="w")
        for text, cmd in [
            ("⇄  Swap Axes",    self._swap_axes),
            ("↕  Invert Pitch", self._invert_pitch),
            ("↕  Invert Roll",  self._invert_roll),
        ]:
            tk.Button(row, text=text, command=cmd,
                      bg=TH["panel2"], fg=TH["text"],
                      font=("Segoe UI", 10, "bold"),
                      relief="flat", bd=0, padx=12, pady=5,
                      cursor="hand2",
                      activebackground=TH["yellow"],
                      activeforeground="#000"
                      ).pack(side="left", padx=(0, 8))

    # ── Calibration ────────────────────────────────────────────────────
    def _build_calibration(self, parent):
        sec = mk_section_header(parent, "Gyro Calibration")
        inner = tk.Frame(sec, bg=TH["panel"])
        inner.pack(fill="x", padx=10, pady=(0, 8))

        tk.Label(inner,
                 text="Place the platform on a flat, still surface.\n"
                      "Press Calibrate — takes ~1 second.",
                 bg=TH["panel"], fg=TH["dim"],
                 font=("Segoe UI", 9), justify="left"
                 ).pack(anchor="w", pady=(0, 8))

        row = tk.Frame(inner, bg=TH["panel"])
        row.pack(anchor="w")

        self._cal_btn = tk.Button(row,
            text="⟳  Calibrate Gyro",
            command=self._calibrate,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", bd=0, padx=12, pady=5,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000")
        self._cal_btn.pack(side="left")

        self._cal_result = tk.Label(row, text="",
                                     bg=TH["panel"], fg=TH["green"],
                                     font=("Consolas", 9))
        self._cal_result.pack(side="left", padx=12)

    # ── Deadbands ──────────────────────────────────────────────────────
    def _build_deadbands(self, parent):
        sec = mk_section_header(parent, "Deadband Tuning")
        inner = tk.Frame(sec, bg=TH["panel"])
        inner.pack(fill="x", padx=10, pady=(0, 10))

        entries = [
            ("PID Deadband",
             self._pid_db_var,
             "Errors smaller than this (°) are ignored — reduces hunting at rest"),
            ("Servo Deadband",
             self._servo_db_var,
             "Servo moves only if change exceeds this (°) — reduces chatter"),
        ]

        # Value labels are created INSIDE the loop directly in slider_row
        # so there is no parent mismatch or re-parenting issue
        self._pid_db_lbl   = None   # assigned inside loop below
        self._servo_db_lbl = None

        for i, (label, var, hint) in enumerate(entries):

            # ── Row container ─────────────────────────────────────────
            rf = tk.Frame(inner, bg=TH["panel"])
            rf.pack(fill="x", pady=(6, 2))

            # ── Title ─────────────────────────────────────────────────
            tk.Label(rf, text=label,
                     bg=TH["panel"], fg=TH["text"],
                     font=("Segoe UI", 10, "bold"), anchor="w"
                     ).pack(anchor="w")

            # ── Hint ──────────────────────────────────────────────────
            tk.Label(rf, text=hint,
                     bg=TH["panel"], fg=TH["dim"],
                     font=("Segoe UI", 8), anchor="w"
                     ).pack(fill="x")

            # ── Slider row ────────────────────────────────────────────
            slider_row = tk.Frame(rf, bg=TH["panel"])
            slider_row.pack(fill="x", pady=(4, 0))

            # Value label created HERE as child of slider_row
            val_lbl = tk.Label(slider_row,
                               text="0.80°" if i == 0 else "1.00°",
                               bg=TH["panel"], fg=TH["accent"],
                               font=("Consolas", 11, "bold"),
                               width=6, anchor="e")
            val_lbl.pack(side="right")   # pack right FIRST so slider fills remaining space

            ttk.Scale(slider_row, from_=0.0, to=5.0, variable=var,
                      command=lambda v, lb=val_lbl: lb.config(
                          text=f"{float(v):.2f}°")
                      ).pack(side="left", fill="x", expand=True)

            # Store reference for _update_display
            if i == 0:
                self._pid_db_lbl   = val_lbl
            else:
                self._servo_db_lbl = val_lbl

            # Separator between entries
            if i < len(entries) - 1:
                tk.Frame(inner, bg=TH["border"], height=1).pack(
                    fill="x", pady=(8, 0))

        # ── Bottom button row: Apply + Read Current Values ────────────
        btn_row = tk.Frame(inner, bg=TH["panel"])
        btn_row.pack(anchor="w", pady=(12, 4))

        tk.Button(btn_row,
            text="Apply Deadbands",
            command=self._apply_deadbands,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", bd=0, padx=12, pady=5,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000"
            ).pack(side="left", padx=(0, 8))

        tk.Button(btn_row,
            text="↓  Read Current Values",
            command=self._read_once,
            bg=TH["panel2"], fg=TH["text"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", bd=0, padx=12, pady=5,
            cursor="hand2",
            activebackground=TH["border"],
            activeforeground=TH["text"]
            ).pack(side="left")

    # ── Save all ───────────────────────────────────────────────────────
    def _build_save(self, parent):
        f = tk.Frame(parent, bg=TH["bg"])
        f.pack(fill="x", padx=14, pady=(4, 16))
        tk.Button(f,
            text="💾  Save All Settings to EEPROM",
            command=self._save_all,
            bg=TH["green"], fg="#000",
            font=("Segoe UI", 11, "bold"),
            relief="flat", bd=0, padx=16, pady=8,
            cursor="hand2",
            activebackground="#00ff88",
            activeforeground="#000"
            ).pack(anchor="w")

    # ── Poll logic ─────────────────────────────────────────────────────
    def _toggle_poll(self):
        if self._polling:
            self._polling = False
            self._poll_btn.config(text="▶  Start Live Feed",
                                   bg=TH["acc2"])
            if self._poll_job:
                self.tab.after_cancel(self._poll_job)
                self._poll_job = None
        else:
            self._polling = True
            self._poll_btn.config(text="⏹  Stop Live Feed",
                                   bg=TH["red"])
            self._poll_once()

    def _poll_once(self):
        if not self._polling:
            return
        threading.Thread(target=self._fetch_status, daemon=True).start()
        self._poll_job = self.tab.after(self.POLL_MS, self._poll_once)

    def _read_once(self):
        """Read current values from ESP32 once without starting live feed."""
        threading.Thread(target=self._fetch_status, daemon=True).start()

    def _fetch_status(self):
        r = self.connection.send_command("getStatus", timeout=1.0)
        if not r:
            return
        try:
            p        = r.split(",")
            pitch    = float(p[0]); roll  = float(p[1]); yaw = float(p[2])
            auto_on  = int(p[3]);   swap  = int(p[4])
            psign    = int(p[5]);   rsign = int(p[6])
            pid_db   = float(p[7]); servo_db = float(p[8])
        except Exception:
            return
        self.tab.after(0, lambda: self._update_display(
            pitch, roll, yaw, auto_on, swap,
            psign, rsign, pid_db, servo_db))

    def _update_display(self, pitch, roll, yaw,
                        auto_on, swap, psign, rsign, pid_db, servo_db):
        self._pitch_var.set(f"{pitch:+.2f}°")
        self._roll_var.set(f"{roll:+.2f}°")
        self._yaw_var.set(f"{yaw:+.2f}°")
        self._auto_var.set("AUTO" if auto_on else "MANUAL")
        self._swap_var.set("ON"   if swap    else "OFF")
        self._psign_var.set(f"{psign:+d}")
        self._rsign_var.set(f"{rsign:+d}")

        # Always update deadband sliders and labels from ESP32 values
        self._pid_db_var.set(pid_db)
        self._pid_db_lbl.config(text=f"{pid_db:.2f}°")
        self._servo_db_var.set(servo_db)
        self._servo_db_lbl.config(text=f"{servo_db:.2f}°")

        # Bind scroll to any newly added widgets
        self.tab.after(50, lambda: self._bind_scroll_fn(self._inner))

    # ── Actions ────────────────────────────────────────────────────────
    def _swap_axes(self):
        r = self.connection.send_command("swapAxes")
        (messagebox.showinfo if "OK" in r else messagebox.showerror)(
            "Swap Axes", r)

    def _invert_pitch(self):
        r = self.connection.send_command("invertPitch")
        (messagebox.showinfo if "OK" in r else messagebox.showerror)(
            "Invert Pitch", r)

    def _invert_roll(self):
        r = self.connection.send_command("invertRoll")
        (messagebox.showinfo if "OK" in r else messagebox.showerror)(
            "Invert Roll", r)

    def _calibrate(self):
        self._cal_btn.config(state="disabled", text="Calibrating…")
        self._cal_result.config(text="Keep still…", fg=TH["yellow"])
        threading.Thread(target=self._run_calibrate, daemon=True).start()

    def _run_calibrate(self):
        r1 = self.connection.send_command("calibrate", timeout=0.5)
        r2 = self.connection.recv_data(timeout=3.0)
        self.tab.after(0, lambda: self._cal_done(r1, r2))

    def _cal_done(self, r1, r2):
        self._cal_btn.config(state="normal", text="⟳  Calibrate Gyro")
        combined = (r1 + " " + r2).strip()
        if "CAL_DONE" in combined:
            offsets = combined.replace("CAL_DONE,", "").strip()
            self._cal_result.config(
                text=f"✓ Done  {offsets}", fg=TH["green"])
        else:
            self._cal_result.config(
                text=f"✗ {combined!r}", fg=TH["red"])

    def _apply_deadbands(self):
        r1 = self.connection.send_command(
            f"setPidDB{self._pid_db_var.get():.2f}")
        r2 = self.connection.send_command(
            f"setServoDb{self._servo_db_var.get():.2f}")
        if "OK" in r1 and "OK" in r2:
            messagebox.showinfo("Applied", "Deadbands sent to ESP32.")
        else:
            messagebox.showwarning("Warning",
                f"PID: {r1!r}   Servo: {r2!r}")

    def _save_all(self):
        self._apply_deadbands()
        r = self.connection.send_command("save", timeout=4.0)
        if "OK" in r:
            messagebox.showinfo("Saved", "All settings saved to EEPROM.")
        else:
            messagebox.showerror("Error", f"Save failed: {r!r}")

    def on_hide(self):
        if self._polling:
            self._toggle_poll()