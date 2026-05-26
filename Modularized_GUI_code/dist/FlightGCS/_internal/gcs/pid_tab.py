# gcs/pid_tab.py — PID gains tab (shared by Pitch and Roll)
import tkinter as tk
from tkinter import ttk, messagebox
from gcs.theme import TH, mk_sep


class PIDTab:
    """PID gains tab — reused for Pitch and Roll axes."""

    _PARAMS = [
        ("Kp", "Proportional Gain", 0.0, 10.0, 2.0,
         "Main correction strength. Higher = faster response, may oscillate."),
        ("Ki", "Integral Gain",     0.0,  5.0, 0.1,
         "Removes steady-state error over time. Keep low to avoid wind-up."),
        ("Kd", "Derivative Gain",   0.0,  5.0, 0.5,
         "Dampens overshoot by acting on the rate of change of the error."),
    ]

    def __init__(self, parent, connection, axis: str):
        self.connection = connection
        self.axis       = axis
        self.tab        = tk.Frame(parent, bg=TH["bg"])
        self._vars      = {}
        self._labels    = {}
        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        root = self.tab

        # Header
        hdr = tk.Frame(root, bg=TH["bg"])
        hdr.pack(fill="x", padx=14, pady=(14, 4))
        tk.Label(hdr, text=f"{self.axis} Axis  —  PID Controller",
                 bg=TH["bg"], fg=TH["accent"],
                 font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(hdr,
                 text=f"Tune the PID gains for the {self.axis.lower()} stabilisation loop.",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")
        mk_sep(root)

        # Sliders
        sf = tk.Frame(root, bg=TH["panel"])
        sf.pack(fill="x", padx=14, pady=4)

        for i, (name, label, lo, hi, default, hint) in enumerate(self._PARAMS):
            key = f"{self.axis}_{name}"
            row = tk.Frame(sf, bg=TH["panel"])
            row.pack(fill="x", padx=12, pady=10)

            top = tk.Frame(row, bg=TH["panel"])
            top.pack(fill="x")
            tk.Label(top, text=f"{name}  —  {label}",
                     bg=TH["panel"], fg=TH["text"],
                     font=("Segoe UI", 10, "bold"), anchor="w"
                     ).pack(side="left")
            lv = ttk.Label(top, text=f"{default:.3f}", width=7, anchor="e")
            lv.pack(side="right")
            self._labels[key] = lv

            tk.Label(row, text=hint,
                     bg=TH["panel"], fg=TH["dim"],
                     font=("Segoe UI", 8), anchor="w"
                     ).pack(fill="x")

            var = tk.DoubleVar(value=default)
            self._vars[key] = var
            ttk.Scale(row, from_=lo, to=hi, variable=var, length=0,
                      command=lambda v, k=key: self._on_slider(k, v)
                      ).pack(fill="x", pady=(4, 0))

            if i < len(self._PARAMS) - 1:
                tk.Frame(sf, bg=TH["border"], height=1).pack(
                    fill="x", padx=12, pady=2)

        mk_sep(root)

        # Buttons
        bf = tk.Frame(root, bg=TH["bg"])
        bf.pack(fill="x", padx=14, pady=6)

        for text, cmd, bg, abg in [
            ("Read from ESP32", self._read,  TH["panel2"], TH["panel"]),
            ("Apply to ESP32",  self._apply, TH["acc2"],   TH["accent"]),
            ("Save to EEPROM",  self._save,  TH["panel2"], TH["panel"]),
        ]:
            tk.Button(bf, text=text, command=cmd,
                      bg=bg, fg=TH["bright"],
                      font=("Segoe UI", 10, "bold"),
                      relief="flat", bd=0, padx=12, pady=6,
                      cursor="hand2",
                      activebackground=abg,
                      activeforeground=TH["bright"]
                      ).pack(side="left", padx=(0, 8))

    # ------------------------------------------------------------------
    def _on_slider(self, key, value):
        self._labels[key].config(text=f"{float(value):.3f}")

    def _read(self):
        r = self.connection.send_command(f"get{self.axis}PID")
        if not r:
            messagebox.showerror("Error", "No response.")
            return
        try:
            kp, ki, kd = map(float, r.split(","))
            for param, val in zip(["Kp", "Ki", "Kd"], [kp, ki, kd]):
                k = f"{self.axis}_{param}"
                self._vars[k].set(val)
                self._labels[k].config(text=f"{val:.3f}")
        except Exception as e:
            messagebox.showerror("Parse Error", f"{e}\nRaw: {r!r}")

    def _apply(self):
        kp = self._vars[f"{self.axis}_Kp"].get()
        ki = self._vars[f"{self.axis}_Ki"].get()
        kd = self._vars[f"{self.axis}_Kd"].get()
        for cmd in [f"set{self.axis}P{kp:.3f}",
                    f"set{self.axis}I{ki:.3f}",
                    f"set{self.axis}D{kd:.3f}"]:
            if "OK" not in self.connection.send_command(cmd):
                messagebox.showwarning("Warning", f"Unexpected response to {cmd!r}")
                return
        messagebox.showinfo("Applied", f"{self.axis} PID sent to ESP32.")

    def _save(self):
        self._apply()
        r = self.connection.send_command("save", timeout=4.0)
        if "OK" in r:
            messagebox.showinfo("Saved", f"{self.axis} PID saved to EEPROM.")
        else:
            messagebox.showerror("Error", f"Save failed: {r!r}")
