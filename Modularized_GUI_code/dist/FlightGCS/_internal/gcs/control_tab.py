# gcs/control_tab.py — Filters tab
import tkinter as tk
from tkinter import ttk, messagebox
from gcs.theme import TH, mk_sep, mk_section_header


class ControlTab:
    """⚙  Filters — ACCEL / GYRO / COMP complementary filter coefficients."""

    _FILTERS = [
        ("ACCEL_FILTER", "Accelerometer Weight", 0.0, 1.0, 0.3,
         "How much the accelerometer corrects the fusion (0 = none, 1 = full)"),
        ("GYRO_FILTER",  "Gyroscope Low-Pass",   0.0, 1.0, 0.08,
         "Low-pass weight on raw gyro readings (0 = no filter, 1 = full smoothing)"),
        ("COMP_FILTER",  "Complementary Blend",  0.0, 1.0, 0.7,
         "Blend between gyro integration and accelerometer angle (0 = gyro, 1 = accel)"),
    ]

    def __init__(self, parent, connection):
        self.connection = connection
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
        tk.Label(hdr, text="Sensor Filter Coefficients",
                 bg=TH["bg"], fg=TH["accent"],
                 font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(hdr,
                 text="Adjust the complementary filter weights for the IMU fusion algorithm.",
                 bg=TH["bg"], fg=TH["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")
        mk_sep(root)

        # Sliders panel
        sf = tk.Frame(root, bg=TH["panel"])
        sf.pack(fill="x", padx=14, pady=4)

        for i, (key, label, lo, hi, default, hint) in enumerate(self._FILTERS):
            row = tk.Frame(sf, bg=TH["panel"])
            row.pack(fill="x", padx=12, pady=10)

            # Label row
            top = tk.Frame(row, bg=TH["panel"])
            top.pack(fill="x")
            tk.Label(top, text=label,
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

            if i < len(self._FILTERS) - 1:
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
    def _on_slider(self, name, value):
        self._labels[name].config(text=f"{float(value):.3f}")

    def set_connected(self, ok: bool):
        pass  # status handled by main connection bar

    def _read(self):
        r = self.connection.send_command("get")
        if not r:
            messagebox.showerror("Error", "No response from ESP32.")
            return
        try:
            a, g, c = map(float, r.split(","))
            for name, val in zip(
                    ["ACCEL_FILTER", "GYRO_FILTER", "COMP_FILTER"], [a, g, c]):
                self._vars[name].set(val)
                self._labels[name].config(text=f"{val:.3f}")
        except Exception as e:
            messagebox.showerror("Parse Error", f"{e}\nRaw: {r!r}")

    def _apply(self):
        a = self._vars["ACCEL_FILTER"].get()
        g = self._vars["GYRO_FILTER"].get()
        c = self._vars["COMP_FILTER"].get()
        for cmd in [f"setA{a:.3f}", f"setG{g:.3f}", f"setC{c:.3f}"]:
            if "OK" not in self.connection.send_command(cmd):
                messagebox.showwarning("Warning", f"Unexpected response to {cmd!r}")
                return
        messagebox.showinfo("Applied", "Filter values sent to ESP32.")

    def _save(self):
        self._apply()
        r = self.connection.send_command("save", timeout=4.0)
        if "OK" in r:
            messagebox.showinfo("Saved", "Filter values saved to EEPROM.")
        else:
            messagebox.showerror("Error", f"Save failed: {r!r}")
