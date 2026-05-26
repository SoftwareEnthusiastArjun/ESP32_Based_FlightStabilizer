"""
flash_tab.py — ESP32 Firmware Flash Tab
Flashes a pre-compiled .bin file to the ESP32 using esptool.

Dependency:  pip install esptool pyserial
"""

import shutil
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import subprocess
import sys
import os
import queue
import serial.tools.list_ports


# ── Colour palette (matches serial monitor theme) ────────────────────────────
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
    "text":    "#cfd8dc",
    "dim":     "#546e7a",
    "bright":  "#eceff1",
}

KNOWN_VIDS = {0x10C4, 0x1A86, 0x0403, 0x239A, 0x303A}


def find_esp32_port():
    for p in serial.tools.list_ports.comports():
        if p.vid in KNOWN_VIDS:
            return p.device
        if p.description and any(k in p.description.lower()
                for k in ("ch340", "cp210", "ftdi", "uart", "usb serial", "esp")):
            return p.device
    return None


def list_all_ports():
    return [(p.device, p.description or p.device)
            for p in serial.tools.list_ports.comports()]


def mk_btn(parent, text, cmd, bg=None, fg=None, state="normal", **kw):
    return tk.Button(parent, text=text, command=cmd,
                     bg=bg or T["acc2"],
                     fg=fg or T["bright"],
                     font=("Segoe UI", 10, "bold"),
                     relief="flat", bd=0, padx=12, pady=6,
                     cursor="hand2",
                     activebackground=T["accent"],
                     activeforeground="#000",
                     state=state, **kw)


class FlashTab:
    """
    ESP32 firmware flash tab.
    Flashes a pre-compiled .bin file to the ESP32 using esptool.
    Auto-detects COM port. Shows live flash progress in the log.
    """

    # ESP32 flash settings — correct for most ESP32 DevKit boards
    FLASH_MODE    = "dio"
    FLASH_FREQ    = "40m"
    FLASH_SIZE    = "4MB"
    FLASH_ADDR    = "0x10000"    # application start address
    BAUD_RATE     = "921600"     # fast flash baud rate


    def __init__(self, parent):
        self.tab       = tk.Frame(parent, bg=T["bg"])
        self._log_q    = queue.Queue()
        self._flashing = False
        self._file_path = tk.StringVar(value="No file selected")
        self._port_var  = tk.StringVar()
        self._baud_var  = tk.StringVar(value=self.BAUD_RATE)
        self._build()
        self._refresh_ports()
        self._poll_log()

    # ── Build UI ──────────────────────────────────────────────────────
    def _build(self):
        root = self.tab

        # ── Header ────────────────────────────────────────────────────
        hdr = tk.Frame(root, bg=T["bg"])
        hdr.pack(fill="x", padx=14, pady=(14, 4))
        tk.Label(hdr, text="Firmware Flasher",
                 bg=T["bg"], fg=T["accent"],
                 font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(hdr,
                 text="Flash firmware directly to your ESP32. "
                      "No PlatformIO or Arduino IDE needed.",
                 bg=T["bg"], fg=T["dim"],
                 font=("Segoe UI", 9)).pack(anchor="w")

        tk.Frame(root, bg=T["border"], height=1).pack(
            fill="x", padx=14, pady=6)

        # ── Port selection ─────────────────────────────────────────────
        port_frame = tk.Frame(root, bg=T["panel"])
        port_frame.pack(fill="x", padx=14, pady=4)

        tk.Label(port_frame, text="Serial Port",
                 bg=T["panel"], fg=T["text"],
                 font=("Segoe UI", 10, "bold")
                 ).grid(row=0, column=0, sticky="w", padx=10, pady=(8, 2))

        port_row = tk.Frame(port_frame, bg=T["panel"])
        port_row.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 8))
        port_frame.columnconfigure(0, weight=1)

        self._port_cb = ttk.Combobox(port_row, textvariable=self._port_var,
                                      width=22, font=("Consolas", 10),
                                      state="readonly")
        self._port_cb.pack(side="left", padx=(0, 6))

        mk_btn(port_row, "↺  Refresh", self._refresh_ports,
               bg=T["panel2"]).pack(side="left", padx=(0, 6))

        # Status dot
        self._port_dot = tk.Canvas(port_row, width=12, height=12,
                                    bg=T["panel"], highlightthickness=0)
        self._port_dot.pack(side="left", padx=(4, 0))
        self._dot = self._port_dot.create_oval(
            2, 2, 10, 10, fill=T["dim"], outline="")

        self._port_status_lbl = tk.Label(port_row, text="No port selected",
                                          bg=T["panel"], fg=T["dim"],
                                          font=("Segoe UI", 9))
        self._port_status_lbl.pack(side="left", padx=6)

        tk.Frame(root, bg=T["border"], height=1).pack(
            fill="x", padx=14, pady=4)

        # ── File selection ─────────────────────────────────────────────
        file_frame = tk.Frame(root, bg=T["panel"])
        file_frame.pack(fill="x", padx=14, pady=4)

        tk.Label(file_frame, text="Firmware File  (.bin)",
                 bg=T["panel"], fg=T["text"],
                 font=("Segoe UI", 10, "bold")
                 ).pack(anchor="w", padx=10, pady=(8, 4))

        file_row = tk.Frame(file_frame, bg=T["panel"])
        file_row.pack(fill="x", padx=10, pady=(0, 8))

        self._file_lbl = tk.Label(file_row,
                                   textvariable=self._file_path,
                                   bg=T["panel2"], fg=T["dim"],
                                   font=("Consolas", 9), anchor="w",
                                   padx=8, pady=4)
        self._file_lbl.pack(side="left", fill="x", expand=True,
                             padx=(0, 8))

        mk_btn(file_row, "Browse…", self._browse_file,
               bg=T["panel2"]).pack(side="left")

        tk.Frame(root, bg=T["border"], height=1).pack(
            fill="x", padx=14, pady=4)

        # ── Advanced settings (collapsible) ───────────────────────────
        adv_hdr = tk.Frame(root, bg=T["bg"])
        adv_hdr.pack(fill="x", padx=14, pady=2)
        tk.Button(adv_hdr, text="⚙  Advanced Settings ▾",
                  command=self._toggle_advanced,
                  bg=T["bg"], fg=T["dim"],
                  font=("Segoe UI", 9),
                  relief="flat", bd=0, cursor="hand2",
                  activebackground=T["bg"],
                  activeforeground=T["text"]
                  ).pack(anchor="w")

        self._adv_frame = tk.Frame(root, bg=T["panel"])
        # Not packed by default — shown when toggled
        self._adv_visible = False

        adv_inner = tk.Frame(self._adv_frame, bg=T["panel"])
        adv_inner.pack(fill="x", padx=14, pady=8)

        for label, var, default in [
            ("Flash Baud Rate", self._baud_var, self.BAUD_RATE),
            ("Flash Mode",
             tk.StringVar(value=self.FLASH_MODE), self.FLASH_MODE),
            ("Flash Frequency",
             tk.StringVar(value=self.FLASH_FREQ), self.FLASH_FREQ),
            ("Flash Address",
             tk.StringVar(value=self.FLASH_ADDR), self.FLASH_ADDR),
        ]:
            row = tk.Frame(adv_inner, bg=T["panel"])
            row.pack(fill="x", pady=2)
            tk.Label(row, text=label + ":", width=18, anchor="w",
                     bg=T["panel"], fg=T["dim"],
                     font=("Segoe UI", 9)).pack(side="left")
            tk.Entry(row, textvariable=var, width=14,
                     font=("Consolas", 9),
                     bg=T["panel2"], fg=T["text"],
                     insertbackground=T["accent"],
                     relief="flat", bd=2).pack(side="left", ipady=2)

        # ── Flash button + status ──────────────────────────────────────
        action_frame = tk.Frame(root, bg=T["bg"])
        action_frame.pack(fill="x", padx=14, pady=10)

        self._flash_btn = mk_btn(action_frame,
                                  "⚡  Flash Firmware to ESP32",
                                  self._start_flash,
                                  bg=T["acc2"])
        self._flash_btn.pack(side="left", padx=(0, 12))

        self._status_var = tk.StringVar(value="Ready")
        self._status_lbl = tk.Label(action_frame,
                                     textvariable=self._status_var,
                                     bg=T["bg"], fg=T["dim"],
                                     font=("Segoe UI", 10, "bold"))
        self._status_lbl.pack(side="left")

        # Progress bar
        self._progress = ttk.Progressbar(root, mode="indeterminate",
                                          length=0)
        self._progress.pack(fill="x", padx=14, pady=(0, 4))

        tk.Frame(root, bg=T["border"], height=1).pack(
            fill="x", padx=14, pady=4)

        # ── Log window ────────────────────────────────────────────────
        tk.Label(root, text="Flash Log",
                 bg=T["bg"], fg=T["dim"],
                 font=("Segoe UI", 9, "bold")
                 ).pack(anchor="w", padx=14, pady=(4, 0))

        log_frame = tk.Frame(root, bg=T["border"], bd=1)
        log_frame.pack(fill="both", expand=True, padx=14, pady=4)

        self._log = tk.Text(log_frame,
                             bg=T["bg"], fg=T["text"],
                             font=("Consolas", 9),
                             relief="flat", bd=0,
                             state="disabled", wrap="word")
        self._log.pack(fill="both", expand=True, padx=1, pady=1)

        scroll = ttk.Scrollbar(log_frame, command=self._log.yview)
        scroll.pack(side="right", fill="y")
        self._log.config(yscrollcommand=scroll.set)

        self._log.tag_config("ok",   foreground=T["green"])
        self._log.tag_config("err",  foreground=T["red"])
        self._log.tag_config("warn", foreground=T["yellow"])
        self._log.tag_config("info", foreground=T["accent"])
        self._log.tag_config("dim",  foreground=T["dim"])

        # Clear log button
        mk_btn(root, "Clear Log", self._clear_log,
               bg=T["panel2"]).pack(anchor="w", padx=14, pady=(0, 8))

        # Dependency check hint
        tk.Label(root,
                 text="Requirement:  pip install esptool pyserial",
                 bg=T["bg"], fg=T["dim"],
                 font=("Segoe UI", 8)
                 ).pack(anchor="w", padx=14, pady=(0, 6))

    # ── Port helpers ──────────────────────────────────────────────────
    def _refresh_ports(self):
        ports  = list_all_ports()
        labels = [f"{d}  —  {n}" for d, n in ports]
        self._port_cb["values"] = labels
        auto = find_esp32_port()
        if auto:
            for i, (d, _) in enumerate(ports):
                if d == auto:
                    self._port_cb.current(i)
                    self._port_dot.itemconfig(self._dot, fill=T["green"])
                    self._port_status_lbl.config(
                        text=f"ESP32 detected: {auto}",
                        fg=T["green"])
                    break
        elif labels:
            self._port_cb.current(0)
            self._port_dot.itemconfig(self._dot, fill=T["yellow"])
            self._port_status_lbl.config(
                text="Port found — may not be ESP32", fg=T["yellow"])
        else:
            self._port_var.set("")
            self._port_dot.itemconfig(self._dot, fill=T["red"])
            self._port_status_lbl.config(
                text="No serial ports found", fg=T["red"])

    def _get_port(self):
        v = self._port_var.get()
        return v.split("  —  ")[0].strip() if v else None

    # ── File helpers ──────────────────────────────────────────────
    def _browse_file(self):
        path = filedialog.askopenfilename(
            title="Select firmware .bin file",
            filetypes=[("Binary firmware", "*.bin"),
                       ("All files", "*.*")])
        if path:
            self._file_path.set(path)
            self._file_lbl.config(fg=T["text"])

    def _toggle_advanced(self):
        if self._adv_visible:
            self._adv_frame.pack_forget()
            self._adv_visible = False
        else:
            self._adv_frame.pack(fill="x", padx=14, pady=2)
            self._adv_visible = True

    # ── Log helpers ───────────────────────────────────────────────────
    def _log_write(self, text, tag=""):
        self._log.config(state="normal")
        self._log.insert("end", text + "\n", tag)
        self._log.config(state="disabled")
        self._log.see("end")

    def _clear_log(self):
        self._log.config(state="normal")
        self._log.delete("1.0", "end")
        self._log.config(state="disabled")

    def _poll_log(self):
        try:
            while True:
                tag, msg = self._log_q.get_nowait()
                self._log_write(msg, tag)
        except queue.Empty:
            pass
        self.tab.after(50, self._poll_log)

    def _q_log(self, msg, tag=""):
        self._log_q.put((tag, msg))

    def _set_status(self, msg, colour):
        self.tab.after(0, lambda: self._status_var.set(msg))
        self.tab.after(0, lambda: self._status_lbl.config(fg=colour))

    # ── Flash logic ───────────────────────────────────────────────────
    def _start_flash(self):
        if self._flashing:
            return

        port = self._get_port()
        if not port:
            messagebox.showerror("No Port",
                "Select or refresh a serial port first.")
            return

        filepath = self._file_path.get()
        if not filepath or filepath == "No file selected":
            messagebox.showerror("No File",
                "Select a firmware file first.")
            return

        if not os.path.isfile(filepath):
            messagebox.showerror("File Not Found",
                f"Cannot find:\n{filepath}")
            return

        self._flashing = True
        self._flash_btn.config(state="disabled",
                                text="Flashing…")
        self._progress.start(10)
        self._clear_log()

        threading.Thread(target=self._flash_thread,
                         args=(port, filepath),
                         daemon=True).start()

    def _flash_thread(self, port, filepath):
        def done(success):
            self.tab.after(0, self._on_flash_done, success)

        try:
            bin_path = filepath
            self._q_log("━━━  Flashing firmware  ━━━", "info")
            success = self._flash_bin(port, bin_path)
            done(success)

        except Exception as e:
            self._q_log(f"Unexpected error: {e}", "err")
            done(False)


    def _flash_bin(self, port, bin_path):
        """Flash .bin to ESP32 using esptool. Returns True on success."""

        # Check esptool is available
        esptool_cmd = shutil.which("esptool") or shutil.which("esptool.py")
        if not esptool_cmd:
            # Try via python -m esptool
            try:
                subprocess.run([sys.executable, "-m", "esptool", "--version"],
                               capture_output=True, check=True)
                esptool_cmd = None   # use python -m esptool
            except (subprocess.CalledProcessError, FileNotFoundError):
                self._q_log(
                    "esptool not found. Installing now...", "warn")
                self._run_subprocess(
                    [sys.executable, "-m", "pip", "install", "esptool"])
                esptool_cmd = None

        if esptool_cmd:
            base_cmd = [esptool_cmd]
        else:
            base_cmd = [sys.executable, "-m", "esptool"]

        cmd = base_cmd + [
            "--chip",   "esp32",
            "--port",   port,
            "--baud",   self._baud_var.get(),
            "--before", "default_reset",
            "--after",  "hard_reset",
            "write_flash",
            "-z",
            "--flash_mode",  self.FLASH_MODE,
            "--flash_freq",  self.FLASH_FREQ,
            "--flash_size",  self.FLASH_SIZE,
            self.FLASH_ADDR, bin_path,
        ]

        self._q_log(f"Port:  {port}", "dim")
        self._q_log(f"File:  {bin_path}", "dim")
        self._q_log(f"Baud:  {self._baud_var.get()}", "dim")
        self._q_log("", "")
        self._q_log(f"Running: {' '.join(cmd)}", "dim")
        self._q_log("", "")

        success = self._run_subprocess(
            cmd,
            success_keyword="Hash of data verified",
            progress_keyword="%")

        return success

    def _run_subprocess(self, cmd, success_keyword=None,
                        progress_keyword=None,
                        find_output=None, output_dir=None):
        """
        Run a subprocess, stream output to the log window line by line.
        Returns True / found-file-path on success, False / None on failure.
        """
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            found_path = None

            for line in proc.stdout:
                line = line.rstrip()
                if not line:
                    continue

                # Colour code output lines
                lo = line.lower()
                if any(k in lo for k in ("error", "failed", "exception")):
                    tag = "err"
                elif any(k in lo for k in ("warning", "warn")):
                    tag = "warn"
                elif any(k in lo for k in ("writing", "flashing",
                                            "connecting", "chip is",
                                            "uploading", "verif")):
                    tag = "info"
                elif progress_keyword and progress_keyword in line:
                    tag = "ok"
                else:
                    tag = ""

                self._q_log(line, tag)

                if find_output and output_dir and find_output in line:
                    # Try to find the output file
                    for f in os.listdir(output_dir):
                        if f.endswith(find_output):
                            found_path = os.path.join(output_dir, f)

            proc.wait()
            success = proc.returncode == 0

            if success_keyword:
                # Also check if success keyword appeared
                pass  # already logged above

            if success:
                self._q_log("", "")
                self._q_log("✓ Command completed successfully.", "ok")
            else:
                self._q_log("", "")
                self._q_log(
                    f"✗ Command failed (exit code {proc.returncode}).",
                    "err")

            if find_output:
                return found_path if success else None
            return success

        except FileNotFoundError:
            self._q_log(
                f"✗ Command not found: {cmd[0]}", "err")
            return False if find_output is None else None

    # ── Flash done ────────────────────────────────────────────────────
    def _on_flash_done(self, success):
        self._flashing = False
        self._progress.stop()
        self._flash_btn.config(state="normal",
                                text="⚡  Flash Firmware to ESP32")

        if success:
            self._set_status("✓  Flash successful!", T["green"])
            self._q_log("", "")
            self._q_log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "ok")
            self._q_log("  ✓  Firmware flashed successfully!   ", "ok")
            self._q_log("  The ESP32 is now rebooting.          ", "ok")
            self._q_log("  Open the Serial Monitor tab to see   ", "ok")
            self._q_log("  the device output at 115200 baud.    ", "ok")
            self._q_log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "ok")
            messagebox.showinfo(
                "Flash Complete",
                "Firmware flashed successfully!\n\n"
                "The ESP32 has been reset and is running the new firmware.\n\n"
                "Switch to the Serial Monitor tab and connect at 115200 baud "
                "to see the device output.")
        else:
            self._set_status("✗  Flash failed", T["red"])
            self._q_log("", "")
            self._q_log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "err")
            self._q_log("  ✗  Flash failed. Check the log above.", "err")
            self._q_log("  Common fixes:", "err")
            self._q_log("  • Hold BOOT button on ESP32 during flash", "err")
            self._q_log("  • Try a lower baud rate (460800 or 115200)", "err")
            self._q_log("  • Check the USB cable (use a data cable)", "err")
            self._q_log("  • Close any other app using the COM port", "err")
            self._q_log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "err")