# theme.py — shared dark avionics theme for the ESP32 GCS
# Imported by main.py and every tab module.
# Never import from main.py in tab files — import from here instead.

from tkinter import ttk

TH = {
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
    "text":    "#cfd8dc",
    "dim":     "#546e7a",
    "bright":  "#eceff1",
}


def apply_ttk_theme():
    """Apply the dark ttk theme to the whole application."""
    style = ttk.Style()
    try:
        style.theme_use("clam")
    except Exception:
        pass

    panel, panel2  = TH["panel"], TH["panel2"]
    border, text   = TH["border"], TH["text"]
    dim, accent    = TH["dim"], TH["accent"]

    style.configure(".",
        background=panel, foreground=text,
        font=("Segoe UI", 10),
        troughcolor=panel2, bordercolor=border,
        focuscolor=accent, selectbackground=TH["acc2"],
        fieldbackground=panel2, insertcolor=accent)

    for w in ("TFrame", "TLabelframe"):
        style.configure(w, background=panel)
    style.configure("TLabel",      background=panel, foreground=text)
    style.configure("TLabelframe", background=panel, foreground=accent,
                    bordercolor=border)
    style.configure("TLabelframe.Label",
                    background=panel, foreground=accent,
                    font=("Segoe UI", 10, "bold"))

    style.configure("TButton",
        background=TH["acc2"], foreground=TH["bright"],
        font=("Segoe UI", 10, "bold"),
        borderwidth=0, focusthickness=0, padding=(10, 5))
    style.map("TButton",
        background=[("active", accent), ("disabled", border)],
        foreground=[("active", "#000"), ("disabled", dim)])

    style.configure("TNotebook",
        background=TH["bg"], borderwidth=0, tabmargins=0)
    style.configure("TNotebook.Tab",
        background=panel, foreground=dim,
        font=("Segoe UI", 10, "bold"), padding=[14, 6], borderwidth=0)
    style.map("TNotebook.Tab",
        background=[("selected", panel2)],
        foreground=[("selected", accent)])

    style.configure("TScale",
        background=panel, troughcolor=panel2,
        sliderlength=14, sliderrelief="flat")
    style.map("TScale", background=[("active", accent)])

    style.configure("TScrollbar",
        background=panel2, troughcolor=panel,
        borderwidth=0, arrowsize=12)

    style.configure("TEntry",
        fieldbackground=panel2, foreground=text,
        bordercolor=border, insertcolor=accent)

    style.configure("Horizontal.TProgressbar",
        background=accent, troughcolor=panel2,
        borderwidth=0, thickness=6)


def mk_btn(parent, text, cmd,
           bg=None, fg=None, abg=None, afg="#000", **kw):
    """Create a styled flat tk.Button."""
    import tkinter as tk
    return tk.Button(
        parent, text=text, command=cmd,
        bg=bg   or TH["acc2"],
        fg=fg   or TH["bright"],
        font=kw.pop("font", ("Segoe UI", 10, "bold")),
        relief="flat", bd=0, cursor="hand2",
        activebackground=abg or TH["accent"],
        activeforeground=afg,
        **kw
    )


def mk_sep(parent, padx=14, pady=6):
    """Horizontal separator line."""
    import tkinter as tk
    tk.Frame(parent, bg=TH["border"], height=1).pack(
        fill="x", padx=padx, pady=pady)


def mk_section_header(parent, title):
    """Styled section header label inside a panel frame."""
    import tkinter as tk
    f = tk.Frame(parent, bg=TH["panel"])
    f.pack(fill="x", padx=14, pady=4)
    tk.Label(f, text=title,
             bg=TH["panel"], fg=TH["accent"],
             font=("Segoe UI", 10, "bold")
             ).pack(anchor="w", padx=10, pady=(8, 2))
    tk.Frame(f, bg=TH["border"], height=1).pack(
        fill="x", padx=10, pady=(0, 6))
    return f
