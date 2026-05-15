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

        self._pitch_var  = tk.StringVar(value="--")
        self._roll_var   = tk.StringVar(value="--")
        self._yaw_var    = tk.StringVar(value="--")
        self._auto_var   = tk.StringVar(value="--")
        self._swap_var   = tk.StringVar(value="--")
        self._psign_var  = tk.StringVar(value="--")
        self._rsign_var  = tk.StringVar(value="--")
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
                 text="Live sensor angles, axis orientation, gyro calibration and deadband tuning.",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")
        mk_sep(root)

        # Canvas so we can scroll if needed
        canvas = tk.Canvas(root, bg=TH["bg"], highlightthickness=0)
        sb = ttk.Scrollbar(root, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg=TH["bg"])
        win = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_resize(e):
            canvas.itemconfig(win, width=e.width)
        def _on_frame(e):
            canvas.configure(scrollregion=canvas.bbox("all"))
        canvas.bind("<Configure>", _on_resize)
        inner.bind("<Configure>", _on_frame)

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

        # Angle readouts
        angle_row = tk.Frame(inner, bg=TH["panel"])
        angle_row.pack(fill="x", pady=4)

        for label, var, col in [
            ("PITCH", self._pitch_var, TH["accent"]),
            ("ROLL",  self._roll_var,  TH["green"]),
            ("YAW",   self._yaw_var,   TH["yellow"]),
        ]:
            cell = tk.Frame(angle_row, bg=TH["panel2"],
                            width=180, height=104)
            cell.pack(side="left", padx=6, pady=4)
            cell.pack_propagate(False)
            tk.Label(cell, text=label,
                     bg=TH["panel2"], fg=TH["dim"],
                     font=("Segoe UI", 9, "bold")
                     ).place(relx=.5, y=10, anchor="n")
            tk.Label(cell, textvariable=var,
                     bg=TH["panel2"], fg=col,
                     font=("Courier New", 16, "bold")
                     ).place(relx=.5, rely=.65, anchor="center")

        # State pills
        pill_row = tk.Frame(inner, bg=TH["panel"])
        pill_row.pack(fill="x", pady=(4, 0))

        def pill(text_lbl, var):
            f = tk.Frame(pill_row, bg=TH["panel2"], padx=10, pady=3)
            f.pack(side="left", padx=4, pady=2)
            tk.Label(f, text=text_lbl + ":",
                     bg=TH["panel2"], fg=TH["dim"],
                     font=("Segoe UI", 9)).pack(side="left")
            tk.Label(f, textvariable=var,
                     bg=TH["panel2"], fg=TH["text"],
                     font=("Consolas", 9, "bold"), width=6
                     ).pack(side="left", padx=(4, 0))

        pill("Mode",       self._auto_var)
        pill("Axis Swap",  self._swap_var)
        pill("Pitch Sign", self._psign_var)
        pill("Roll Sign",  self._rsign_var)

        # Poll button
        self._poll_btn = tk.Button(inner,
            text="▶  Start Live Feed",
            command=self._toggle_poll,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", padx=12, pady=5,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000")
        self._poll_btn.pack(pady=8, anchor="w")

    # ── Axis orientation ────────────────────────────────────────────────
    def _build_axis(self, parent):
        sec = mk_section_header(parent, "Axis Orientation")
        inner = tk.Frame(sec, bg=TH["panel"])
        inner.pack(fill="x", padx=10, pady=(0, 8))

        tk.Label(inner,
                 text="If the wrong servo responds, or correction goes in the wrong\n"
                      "direction, use these buttons. Changes take effect immediately.",
                 bg=TH["panel"], fg=TH["dim"],
                 font=("Segoe UI", 9), justify="left"
                 ).pack(anchor="w", pady=(0, 8))

        btn_row = tk.Frame(inner, bg=TH["panel"])
        btn_row.pack(anchor="w")

        for text, cmd in [
            ("⇄  Swap Axes",    self._swap_axes),
            ("↕  Invert Pitch", self._invert_pitch),
            ("↕  Invert Roll",  self._invert_roll),
        ]:
            tk.Button(btn_row, text=text, command=cmd,
                      bg=TH["panel2"], fg=TH["text"],
                      font=("Segoe UI", 10, "bold"),
                      relief="flat", bd=0, padx=12, pady=5,
                      cursor="hand2",
                      activebackground=TH["yellow"],
                      activeforeground="#000"
                      ).pack(side="left", padx=(0, 8))

    # ── Calibration ─────────────────────────────────────────────────────
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

        btn_row = tk.Frame(inner, bg=TH["panel"])
        btn_row.pack(anchor="w")

        self._cal_btn = tk.Button(btn_row,
            text="⟳  Calibrate Gyro",
            command=self._calibrate,
            bg=TH["acc2"], fg=TH["bright"],
            font=("Segoe UI", 10, "bold"),
            relief="flat", bd=0, padx=12, pady=5,
            cursor="hand2",
            activebackground=TH["accent"],
            activeforeground="#000")
        self._cal_btn.pack(side="left")

        self._cal_result = tk.Label(btn_row, text="",
                                     bg=TH["panel"], fg=TH["green"],
                                     font=("Consolas", 9))
        self._cal_result.pack(side="left", padx=12)

    # ── Deadbands ────────────────────────────────────────────────────────
    def _build_deadbands(self, parent):
        sec = mk_section_header(parent, "Deadband Tuning")
        inner = tk.Frame(sec, bg=TH["panel"])
        inner.pack(fill="x", padx=10, pady=(0, 8))

        self._pid_db_lbl   = tk.Label(inner, text="0.80°",
                                       bg=TH["panel"], fg=TH["text"],
                                       font=("Consolas", 10))
        self._servo_db_lbl = tk.Label(inner, text="1.00°",
                                       bg=TH["panel"], fg=TH["text"],
                                       font=("Consolas", 10))

        for row, (label, var, lbl_ref, hint) in enumerate([
            ("PID Deadband",
             self._pid_db_var, self._pid_db_lbl,
             "Errors smaller than this (°) are ignored by the PID — reduces hunting at rest"),
            ("Servo Deadband",
             self._servo_db_var, self._servo_db_lbl,
             "Servo only moves if the commanded change exceeds this (°) — reduces chatter"),
        ]):
            rf = tk.Frame(inner, bg=TH["panel"])
            rf.pack(fill="x", pady=6)

            top = tk.Frame(rf, bg=TH["panel"])
            top.pack(fill="x")
            tk.Label(top, text=label,
                     bg=TH["panel"], fg=TH["text"],
                     font=("Segoe UI", 10, "bold")
                     ).pack(side="left")
            lbl_ref.pack(side="right")

            tk.Label(rf, text=hint,
                     bg=TH["panel"], fg=TH["dim"],
                     font=("Segoe UI", 8), anchor="w"
                     ).pack(fill="x")

            ttk.Scale(rf, from_=0.0, to=5.0, variable=var, length=0,
                      command=lambda v, lb=lbl_ref: lb.config(
                          text=f"{float(v):.2f}°")
                      ).pack(fill="x", pady=(4, 0))

        tk.Button(inner, text="Apply Deadbands",
                  command=self._apply_deadbands,
                  bg=TH["acc2"], fg=TH["bright"],
                  font=("Segoe UI", 10, "bold"),
                  relief="flat", bd=0, padx=12, pady=5,
                  cursor="hand2",
                  activebackground=TH["accent"],
                  activeforeground="#000"
                  ).pack(anchor="w", pady=8)

    # ── Save all ──────────────────────────────────────────────────────────
    def _build_save(self, parent):
        f = tk.Frame(parent, bg=TH["bg"])
        f.pack(fill="x", padx=14, pady=(4, 14))
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

    # ── Poll logic ────────────────────────────────────────────────────────
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

    def _fetch_status(self):
        r = self.connection.send_command("getStatus", timeout=1.0)
        if not r:
            return
        try:
            p = r.split(",")
            pitch, roll, yaw = float(p[0]), float(p[1]), float(p[2])
            auto_on = int(p[3]); swap = int(p[4])
            psign = int(p[5]); rsign = int(p[6])
            pid_db = float(p[7]); servo_db = float(p[8])
        except Exception:
            return
        self.tab.after(0, lambda: self._update_display(
            pitch, roll, yaw, auto_on, swap, psign, rsign, pid_db, servo_db))

    def _update_display(self, pitch, roll, yaw,
                        auto_on, swap, psign, rsign, pid_db, servo_db):
        self._pitch_var.set(f"{pitch:+.2f}°")
        self._roll_var.set(f"{roll:+.2f}°")
        self._yaw_var.set(f"{yaw:+.2f}°")
        self._auto_var.set("AUTO" if auto_on else "MANUAL")
        self._swap_var.set("ON" if swap else "OFF")
        self._psign_var.set(f"{psign:+d}")
        self._rsign_var.set(f"{rsign:+d}")
        if abs(self._pid_db_var.get() - pid_db) > 0.05:
            self._pid_db_var.set(pid_db)
            self._pid_db_lbl.config(text=f"{pid_db:.2f}°")
        if abs(self._servo_db_var.get() - servo_db) > 0.05:
            self._servo_db_var.set(servo_db)
            self._servo_db_lbl.config(text=f"{servo_db:.2f}°")

    # ── Actions ────────────────────────────────────────────────────────────
    def _swap_axes(self):
        r = self.connection.send_command("swapAxes")
        (messagebox.showinfo if "OK" in r else messagebox.showerror)(
            "Swap Axes", f"{'Done. ' if 'OK' in r else ''}{r}")

    def _invert_pitch(self):
        r = self.connection.send_command("invertPitch")
        (messagebox.showinfo if "OK" in r else messagebox.showerror)(
            "Invert Pitch", f"{'Done. ' if 'OK' in r else ''}{r}")

    def _invert_roll(self):
        r = self.connection.send_command("invertRoll")
        (messagebox.showinfo if "OK" in r else messagebox.showerror)(
            "Invert Roll", f"{'Done. ' if 'OK' in r else ''}{r}")

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
                text=f"✗ Unexpected: {combined!r}", fg=TH["red"])

    def _apply_deadbands(self):
        pid_db   = self._pid_db_var.get()
        servo_db = self._servo_db_var.get()
        r1 = self.connection.send_command(f"setPidDB{pid_db:.2f}")
        r2 = self.connection.send_command(f"setServoDb{servo_db:.2f}")
        if "OK" in r1 and "OK" in r2:
            messagebox.showinfo("Applied", "Deadbands sent to ESP32.")
        else:
            messagebox.showwarning("Warning",
                f"r1 = {r1!r}   r2 = {r2!r}")

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
