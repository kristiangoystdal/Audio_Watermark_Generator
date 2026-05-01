import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math

from scripts.paths import *
from scripts.user_config import *
from scripts.build import *


# ---------------------------------------------------------
# Constants
# ---------------------------------------------------------
APP_TITLE = "Audio Watermark Flash Tool"
WINDOW_SIZE = "960x620"
WINDOW_MIN_SIZE = (860, 560)

SECTION_WIDTH = 460
ENTRY_WIDTH = 28
SMALL_ENTRY_WIDTH = 12

FS_HZ = 960000
MIN_BIT_US = 3000
FREQ_MIN = 2000
FREQ_MAX = 24000
BIT_SAMPLE_TOLERANCE_PERCENT = 1


# ---------------------------------------------------------
# Tooltip
# ---------------------------------------------------------
class ToolTip:
    def __init__(self, widget, text, colors):
        self.widget = widget
        self.text = text
        self.colors = colors
        self.tip_window = None
        widget.bind("<Enter>", self._show)
        widget.bind("<Leave>", self._hide)

    def _show(self, _=None):
        if self.tip_window or not self.text:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        lbl = tk.Label(
            tw,
            text=self.text,
            background=self.colors["surface_2"],
            foreground=self.colors["muted"],
            relief="flat",
            borderwidth=0,
            font=("SF Pro Display", 10),
            wraplength=300,
            justify="left",
            padx=10,
            pady=6,
        )
        lbl.pack()

    def _hide(self, _=None):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None


# ---------------------------------------------------------
# App
# ---------------------------------------------------------
class FlashToolApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title(APP_TITLE)
        self.geometry(WINDOW_SIZE)
        self.resizable(False, False)
        self.minsize(*WINDOW_MIN_SIZE)

        self.vars = {}
        self.widgets = {}

        self._setup_style()
        self._build_ui()
        self._bind_events()
        self._init_dynamic_state()

        self.after(100, self.bring_to_front)
        self.after(50, self._init_tab_render)

    # -----------------------------------------------------
    # Styling
    # -----------------------------------------------------

    def _setup_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        # --------------------------------------------------
        # Modern dark palette
        # --------------------------------------------------

        self.colors = {
            "bg": "#181a1f",
            "surface": "#181a1f",
            "surface_2": "#2a2e36",
            "surface_3": "#313641",
            "border": "#3a404c",
            "text": "#ffffff",
            "muted": "#a7afbd",
            "accent": "#4f8cff",
            "accent_hover": "#6aa0ff",
            "accent_pressed": "#3d79e6",
            "input_bg": "#1f2329",
            "disabled_bg": "#2a2d33",
            "disabled_text": "#7d8592",
            "success": "#00ffcc",
        }

        c = self.colors

        # Root background
        self.configure(bg=c["bg"])

        # Generic backgrounds
        style.configure(".", background=c["bg"], foreground=c["text"])

        style.configure(
            "TFrame",
            background=c["bg"],
        )

        style.configure(
            "Card.TFrame",
            background=c["surface"],
            relief="flat",
            borderwidth=0,
        )

        style.configure(
            "TLabel",
            background=c["bg"],
            foreground=c["text"],
            font=("SF Pro Display", 12),
        )

        style.configure(
            "Subtitle.TLabel",
            background=c["bg"],
            foreground=c["muted"],
            font=("SF Pro Display", 12),
        )

        style.configure(
            "Title.TLabel",
            background=c["bg"],
            foreground=c["text"],
            font=("SF Pro Display", 24, "bold"),
        )

        style.configure(
            "Muted.TLabel",
            background=c["surface"],
            foreground=c["muted"],
            font=("SF Pro Display", 11),
        )

        style.configure(
            "SectionTitle.TLabel",
            background=c["surface"],
            foreground=c["text"],
            font=("SF Pro Display", 12, "bold"),
        )

        # --------------------------------------------------
        # Notebook
        # --------------------------------------------------
        style.configure(
            "TNotebook",
            background=c["bg"],
            borderwidth=0,
            tabmargins=(0, 0, 0, 0),
        )

        style.configure(
            "TNotebook.Tab",
            background=c["surface_2"],
            foreground=c["muted"],
            padding=(22, 8),
            font=("SF Pro Display", 11, "bold"),
            borderwidth=0,
        )

        style.map(
            "TNotebook.Tab",
            background=[
                ("selected", c["surface"]),
                ("active", c["surface_3"]),
            ],
            foreground=[
                ("selected", c["text"]),
                ("active", c["text"]),
            ],
        )

        # --------------------------------------------------
        # Labelframe as modern card
        # --------------------------------------------------
        style.configure(
            "Section.TLabelframe",
            background=c["surface"],
            bordercolor=c["border"],
            borderwidth=1,
            relief="solid",
            padding=7,
        )

        style.configure(
            "Section.TLabelframe.Label",
            background=c["surface"],
            foreground=c["text"],
            font=("SF Pro Display", 11, "bold"),
        )

        # --------------------------------------------------
        # Entry / Spinbox / Combobox
        # --------------------------------------------------
        style.configure(
            "TEntry",
            fieldbackground=c["input_bg"],
            background=c["input_bg"],
            foreground=c["text"],
            bordercolor=c["border"],
            lightcolor=c["border"],
            darkcolor=c["border"],
            insertcolor=c["text"],
            padding=5,
            relief="flat",
        )

        style.map(
            "TEntry",
            bordercolor=[
                ("focus", c["accent"]),
            ],
            lightcolor=[
                ("focus", c["accent"]),
            ],
            darkcolor=[
                ("focus", c["accent"]),
            ],
        )

        style.configure(
            "TSpinbox",
            fieldbackground=c["input_bg"],
            background=c["input_bg"],
            foreground=c["text"],
            bordercolor=c["border"],
            lightcolor=c["border"],
            darkcolor=c["border"],
            arrowsize=12,
            padding=4,
            relief="solid",
        )

        style.map(
            "TSpinbox",
            bordercolor=[("focus", c["accent"])],
            lightcolor=[("focus", c["accent"])],
            darkcolor=[("focus", c["accent"])],
        )

        # --------------------------------------------------
        # Checkbuttons / Radiobuttons
        # --------------------------------------------------
        style.configure(
            "TCheckbutton",
            background=c["surface"],
            foreground=c["text"],
            font=("SF Pro Display", 11),
        )

        style.map(
            "TCheckbutton",
            background=[("active", c["surface"])],
            foreground=[("disabled", c["disabled_text"])],
        )

        style.configure(
            "TRadiobutton",
            background=c["surface"],
            foreground=c["text"],
            font=("SF Pro Display", 11),
        )

        style.map(
            "TRadiobutton",
            background=[("active", c["surface"])],
            foreground=[("disabled", c["disabled_text"])],
        )

        # --------------------------------------------------
        # Primary button
        # --------------------------------------------------
        style.configure(
            "Primary.TButton",
            background=c["accent"],
            foreground="white",
            borderwidth=0,
            focusthickness=0,
            focuscolor=c["accent"],
            padding=(28, 8),
            font=("SF Pro Display", 12, "bold"),
            relief="flat",
        )

        style.map(
            "Primary.TButton",
            background=[
                ("pressed", c["accent_pressed"]),
                ("active", c["accent_hover"]),
                ("disabled", c["disabled_bg"]),
            ],
            foreground=[
                ("disabled", c["disabled_text"]),
            ],
        )

        # Secondary button if needed later
        style.configure(
            "Secondary.TButton",
            background=c["surface_2"],
            foreground=c["text"],
            borderwidth=0,
            padding=(18, 10),
            font=("SF Pro Display", 11, "bold"),
            relief="flat",
        )

        style.map(
            "Secondary.TButton",
            background=[
                ("pressed", c["surface_3"]),
                ("active", c["surface_3"]),
            ],
        )

        # --------------------------------------------------
        # Progressbar
        # --------------------------------------------------
        style.configure(
            "Horizontal.TProgressbar",
            background=c["accent"],
            troughcolor=c["surface_2"],
            bordercolor=c["border"],
            lightcolor=c["accent"],
            darkcolor=c["accent"],
            thickness=10,
        )

        # --------------------------------------------------
        # Separator
        # --------------------------------------------------
        style.configure(
            "TSeparator",
            background=c["border"],
        )

        self.style = style

    # -----------------------------------------------------
    # Main UI
    # -----------------------------------------------------
    def _build_ui(self):
        outer = ttk.Frame(self, padding=12)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text=APP_TITLE, style="Title.TLabel").pack(pady=(0, 8))

        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill="both", expand=True)

        self.base_tab = ttk.Frame(self.notebook, padding=8)
        self.receiver_tab = ttk.Frame(self.notebook, padding=8)
        self.standalone_tab = ttk.Frame(self.notebook, padding=8)

        self.notebook.add(self.base_tab, text="Base Station")
        self.notebook.add(self.receiver_tab, text="Receiver")
        self.notebook.add(self.standalone_tab, text="Standalone")

        self.notebook.select(self.base_tab)

        self._build_base_tab()
        self._build_receiver_tab()
        self._build_standalone_tab()
        self._build_bottom_bar(outer)

    # -----------------------------------------------------
    # Helpers
    # -----------------------------------------------------

    def on_tab_changed(self, event=None):
        current = self.notebook.select()

        if current == str(self.base_tab):
            self.vars["operation_mode"].set(1)
            self.after(10, lambda: self.widgets["base_user_string"].focus_set())
            self.notebook.select(self.base_tab)

        elif current == str(self.receiver_tab):
            self.vars["operation_mode"].set(0)
            self.after(10, lambda: self.widgets["receiver_user_string"].focus_set())
            self.notebook.select(self.receiver_tab)

        elif current == str(self.standalone_tab):
            self.vars["operation_mode"].set(2)
            self.after(10, lambda: self.widgets["standalone_user_string"].focus_set())
            self.notebook.select(self.standalone_tab)

    def _init_tab_render(self):
        self.notebook.select(self.receiver_tab)
        self.update_idletasks()
        self.notebook.select(self.base_tab)

    def bring_to_front(self):
        self.deiconify()
        self.lift()
        self.focus_force()

        # Optional: remove always-on-top after focus
        self.attributes("-topmost", True)
        self.after(200, lambda: self.attributes("-topmost", False))

    def section(
        self, parent, title, row, column=0, colspan=1, sticky="ew", pady=(0, 6)
    ):
        frame = ttk.LabelFrame(parent, text=title, style="Section.TLabelframe")
        frame.grid(
            row=row, column=column, columnspan=colspan, sticky=sticky, pady=pady, padx=4
        )
        return frame

    def entry_row(self, parent, row, label, var, width=ENTRY_WIDTH):
        ttk.Label(parent, text=f"{label}:").grid(
            row=row, column=0, sticky="e", padx=(0, 10), pady=4
        )
        entry = ttk.Entry(parent, textvariable=var, width=width)
        entry.grid(row=row, column=1, sticky="w", pady=4)
        return entry

    def checkbox(self, parent, row, column, text, var, padx=(0, 20)):
        cb = tk.Checkbutton(
            parent,
            text=text,
            variable=var,
            bg=self.colors["surface"],
            fg=self.colors["text"],
            activebackground=self.colors["surface"],
            activeforeground=self.colors["text"],
            selectcolor=self.colors["accent"],
            indicatoron=True,
            relief="flat",
            offrelief="flat",
            overrelief="flat",
            highlightthickness=0,
            borderwidth=0,
            font=("SF Pro Display", 11),
        )
        cb.grid(row=row, column=column, sticky="w", padx=padx, pady=2)
        return cb

    def muted_label(self, parent, text, row, wrap=420, column=0, pady=(0, 4)):
        lbl = ttk.Label(
            parent, text=text, style="Muted.TLabel", wraplength=wrap, justify="left"
        )
        lbl.grid(row=row, column=column, sticky="w", pady=pady)
        return lbl

    # -----------------------------------------------------
    # Variables
    # -----------------------------------------------------
    def _create_shared_vars(self):
        self.vars["user_string"] = tk.StringVar(
            value=read_user_config_value("USER_STRING")
        )
        self.vars["device_id"] = tk.StringVar(
            value=str(read_user_config_value("DEVICE_ID"))
        )
        self.vars["location"] = tk.StringVar(value=read_user_config_value("LOCATION"))

        self.vars["include_user_string"] = tk.IntVar(value=1)
        self.vars["include_temperature"] = tk.IntVar(value=1)
        self.vars["include_device_id"] = tk.IntVar(value=1)
        self.vars["include_time"] = tk.IntVar(value=1)
        self.vars["include_location"] = tk.IntVar(value=1)

        self.vars["default_interval"] = tk.IntVar(value=1)
        self.vars["interval"] = tk.IntVar(
            value=int(read_user_config_value("INTERVAL_BETWEEN_REPEATS_MINUTES"))
        )

        self.vars["use_start_minute"] = tk.IntVar(value=0)
        self.vars["start_minute"] = tk.IntVar(
            value=int(read_user_config_value("STARTING_MINUTE"))
        )

        self.vars["transmission"] = tk.StringVar(value="cable")

        self.vars["frequency_low"] = tk.IntVar(
            value=int(read_user_config_value("FSK_LOWER_FREQUENCY"))
        )
        self.vars["frequency_high"] = tk.IntVar(
            value=int(read_user_config_value("FSK_HIGHER_FREQUENCY"))
        )

        self.vars["attenuation"] = tk.IntVar(
            value=int(read_user_config_value("SIGNAL_ATTENUATION"))
        )

        self.vars["ecc_enabled"] = tk.IntVar(value=1)
        self.vars["ecc_level"] = tk.IntVar(
            value=int(read_user_config_value("RS_ERROR_CORRECTION_SYMBOLS"))
        )

        self.vars["operation_mode"] = tk.IntVar(value=0)

        self.vars["show_log"] = tk.IntVar(value=0)

    # -----------------------------------------------------
    # Base tab
    # -----------------------------------------------------
    def _build_base_tab(self):
        self._create_shared_vars()
        c = self.colors

        self.base_tab.grid_columnconfigure(0, weight=1, uniform="base_cols")
        self.base_tab.grid_columnconfigure(1, weight=1, uniform="base_cols")

        ttk.Label(
            self.base_tab,
            text="Configure common data and transmission interval, then build & flash.",
            style="Muted.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        left = ttk.Frame(self.base_tab)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        left.grid_columnconfigure(0, weight=1)

        right = ttk.Frame(self.base_tab)
        right.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        right.grid_columnconfigure(0, weight=1)

        device = self.section(left, "Device Information", 0)
        device.grid_columnconfigure(1, weight=1)
        self.widgets["base_user_string"] = self.entry_row(
            device, 0, "User String", self.vars["user_string"]
        )

        include = self.section(left, "Include in Watermark", 1)
        grid = tk.Frame(include, bg=self.colors["surface"])
        grid.grid(row=0, column=0, sticky="w")

        self.checkbox(grid, 0, 0, "User String", self.vars["include_user_string"])
        self.checkbox(grid, 0, 1, "Timestamp", self.vars["include_time"])

        interval = self.section(left, "Interval Settings", 2)
        row = tk.Frame(interval, bg=c["surface"])
        row.grid(row=0, column=0, sticky="ew")
        row.grid_columnconfigure(0, weight=1)

        self.checkbox(
            row,
            0,
            0,
            "Use Default Interval (1 minute)",
            self.vars["default_interval"],
        ).grid(row=0, column=0, sticky="w")

        tk.Label(
            row,
            text="Minutes:",
            bg=c["surface"],
            fg=c["text"],
        ).grid(row=0, column=1, padx=(16, 6))

        self.widgets["base_interval"] = tk.Spinbox(
            row,
            from_=1,
            to=1440,
            width=6,
            textvariable=self.vars["interval"],
            bg=c["surface"],
            fg=c["text"],
            buttonbackground=c["surface_2"],
            insertbackground=c["text"],
            highlightbackground=c["surface_2"],
            highlightcolor=c["accent"],
            highlightthickness=1,
            relief="solid",
            bd=1,
        )
        self.widgets["base_interval"].grid(row=0, column=2, sticky="w")

        delay = self.section(left, "Initial Delay", 3)
        row = tk.Frame(delay, bg=c["surface"])
        row.grid(row=0, column=0, sticky="ew")
        row.grid_columnconfigure(0, weight=1)

        self.checkbox(
            row,
            0,
            0,
            "Use Starting Minute",
            self.vars["use_start_minute"],
        )

        self.widgets["base_start_minute"] = ttk.Spinbox(
            row,
            from_=0,
            to=59,
            width=6,
            textvariable=self.vars["start_minute"],
        )
        self.widgets["base_start_minute"].grid(row=0, column=1, sticky="w")

    # -----------------------------------------------------
    # Receiver tab
    # -----------------------------------------------------
    def _build_receiver_tab(self):
        self.receiver_tab.grid_columnconfigure(0, weight=1, uniform="recv_cols")
        self.receiver_tab.grid_columnconfigure(1, weight=1, uniform="recv_cols")

        ttk.Label(
            self.receiver_tab,
            text="Configure watermark data, FSK frequencies, and transmission method.",
            style="Muted.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        left = ttk.Frame(self.receiver_tab)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        left.grid_columnconfigure(0, weight=1)

        right = ttk.Frame(self.receiver_tab)
        right.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        right.grid_columnconfigure(0, weight=1)

        # Device info
        device = self.section(left, "Device Information", 0)
        device.grid_columnconfigure(1, weight=1)
        self.widgets["receiver_user_string"] = self.entry_row(
            device, 0, "User String", self.vars["user_string"]
        )
        self.widgets["receiver_device_id"] = self.entry_row(
            device, 1, "Device ID", self.vars["device_id"]
        )
        self.widgets["receiver_location"] = self.entry_row(
            device, 2, "Location", self.vars["location"]
        )

        # Include settings
        include = self.section(left, "Include in Watermark", 1)
        grid = tk.Frame(include, bg=self.colors["surface"])
        grid.grid(row=0, column=0, sticky="w")
        self.checkbox(grid, 0, 0, "User String", self.vars["include_user_string"])
        self.checkbox(grid, 0, 1, "Device ID", self.vars["include_device_id"])
        self.checkbox(grid, 0, 2, "Location", self.vars["include_location"])
        self.checkbox(grid, 1, 0, "Temperature", self.vars["include_temperature"])
        self.checkbox(grid, 1, 1, "Timestamp", self.vars["include_time"])

        # Transmission settings
        tx = self.section(right, "Transmission Settings", 0)
        ttk.Radiobutton(
            tx,
            text="Use Cable Transmission",
            variable=self.vars["transmission"],
            value="cable",
        ).grid(row=0, column=0, sticky="w", pady=2)
        ttk.Radiobutton(
            tx,
            text="Use Speaker Transmission",
            variable=self.vars["transmission"],
            value="speaker",
        ).grid(row=1, column=0, sticky="w", pady=2)

        # FSK settings
        fsk = self.section(right, "FSK Parameters", 1)

        row = ttk.Frame(fsk)
        row.grid(row=0, column=0, sticky="w")

        ttk.Label(row, text="Low (Hz):").grid(row=0, column=0, sticky="w")
        self.widgets["receiver_frequency_low"] = ttk.Entry(
            row, width=8, textvariable=self.vars["frequency_low"]
        )
        self.widgets["receiver_frequency_low"].grid(
            row=0, column=1, padx=(8, 20), sticky="w"
        )

        ttk.Label(row, text="High (Hz):").grid(row=0, column=2, sticky="w")
        self.widgets["receiver_frequency_high"] = ttk.Entry(
            row, width=8, textvariable=self.vars["frequency_high"]
        )
        self.widgets["receiver_frequency_high"].grid(
            row=0, column=3, padx=(8, 0), sticky="w"
        )

        ToolTip(
            self.widgets["receiver_frequency_low"],
            "Lower FSK frequency (Hz). Must be < High frequency.\nValid range: 2000-24000 Hz.",
            self.colors,
        )
        ToolTip(
            self.widgets["receiver_frequency_high"],
            "Higher FSK frequency (Hz). Must be > Low frequency.\nValid range: 2000-24000 Hz.",
            self.colors,
        )

        self.widgets["receiver_freq_gap_label"] = ttk.Label(
            fsk, text="", style="Muted.TLabel"
        )
        self.widgets["receiver_freq_gap_label"].grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )

        # Attenuation settings
        attenuation = self.section(right, "Attenuation", 2)

        row = ttk.Frame(attenuation)
        row.grid(row=0, column=0, sticky="w")

        ttk.Label(row, text="Level:").grid(row=0, column=0, sticky="w")

        self.widgets["receiver_attenuation"] = ttk.Spinbox(
            row,
            from_=0,
            to=100,
            width=6,
            textvariable=self.vars["attenuation"],
        )
        self.widgets["receiver_attenuation"].grid(
            row=0, column=1, padx=(8, 4), sticky="w"
        )

        ttk.Label(row, text="%").grid(row=0, column=2, sticky="w", padx=(0, 12))

        self.widgets["receiver_attenuation_db_label"] = ttk.Label(
            row,
            text="",
            style="Muted.TLabel",
        )
        self.widgets["receiver_attenuation_db_label"].grid(row=0, column=3, sticky="w")

        # ECC Settings
        ecc = self.section(right, "ECC Settings", 3)

        row = ttk.Frame(ecc)
        row.grid(row=0, column=0, sticky="w")

        self.checkbox(
            row,
            0,
            0,
            "Enable ECC (Recommended)",
            self.vars["ecc_enabled"],
        ).grid(row=0, column=0, sticky="w")

        self.widgets["receiver_ecc_level"] = ttk.Spinbox(
            row,
            from_=0,
            to=100,
            width=6,
            textvariable=self.vars["ecc_level"],
        )
        self.widgets["receiver_ecc_level"].grid(
            row=0, column=1, padx=(8, 0), sticky="w"
        )
        ToolTip(
            self.widgets["receiver_ecc_level"],
            "Number of Reed-Solomon error-correction symbols (0-100).\nHigher = more robust, but longer transmissions.",
            self.colors,
        )

    # -----------------------------------------------------
    # Standalone tab
    # -----------------------------------------------------
    def _build_standalone_tab(self):
        self.standalone_tab.grid_columnconfigure(0, weight=1, uniform="recv_cols")
        self.standalone_tab.grid_columnconfigure(1, weight=1, uniform="recv_cols")

        ttk.Label(
            self.standalone_tab,
            text="All-in-one mode: configure device info, watermark fields, interval, FSK, and transmission.",
            style="Muted.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        left = ttk.Frame(self.standalone_tab)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        left.grid_columnconfigure(0, weight=1)

        right = ttk.Frame(self.standalone_tab)
        right.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        right.grid_columnconfigure(0, weight=1)

        c = self.colors

        # Device info
        device = self.section(left, "Device Information", 0)
        device.grid_columnconfigure(1, weight=1)
        self.widgets["standalone_user_string"] = self.entry_row(
            device, 0, "User String", self.vars["user_string"]
        )
        self.widgets["standalone_device_id"] = self.entry_row(
            device, 1, "Device ID", self.vars["device_id"]
        )
        self.widgets["standalone_location"] = self.entry_row(
            device, 2, "Location", self.vars["location"]
        )

        # Include settings
        include = self.section(left, "Include in Watermark", 1)
        grid = tk.Frame(include, bg=self.colors["surface"])
        grid.grid(row=0, column=0, sticky="w")
        self.checkbox(grid, 0, 0, "User String", self.vars["include_user_string"])
        self.checkbox(grid, 0, 1, "Device ID", self.vars["include_device_id"])
        self.checkbox(grid, 0, 2, "Location", self.vars["include_location"])
        self.checkbox(grid, 1, 0, "Temperature", self.vars["include_temperature"])
        self.checkbox(grid, 1, 1, "Timestamp", self.vars["include_time"])

        # Interval Settings  (from Base)
        interval = self.section(left, "Interval Settings", 2)
        irow = tk.Frame(interval, bg=c["surface"])
        irow.grid(row=0, column=0, sticky="ew")
        irow.grid_columnconfigure(0, weight=1)

        self.checkbox(
            irow, 0, 0, "Use Default Interval (1 minute)", self.vars["default_interval"]
        ).grid(row=0, column=0, sticky="w")
        tk.Label(irow, text="Minutes:", bg=c["surface"], fg=c["text"]).grid(
            row=0, column=1, padx=(16, 6)
        )

        self.widgets["standalone_interval"] = tk.Spinbox(
            irow,
            from_=1,
            to=1440,
            width=6,
            textvariable=self.vars["interval"],
            bg=c["surface"],
            fg=c["text"],
            buttonbackground=c["surface_2"],
            insertbackground=c["text"],
            highlightbackground=c["surface_2"],
            highlightcolor=c["accent"],
            highlightthickness=1,
            relief="solid",
            bd=1,
        )
        self.widgets["standalone_interval"].grid(row=0, column=2, sticky="w")

        # Initial Delay  (from Base)
        delay = self.section(left, "Initial Delay", 3)
        drow = tk.Frame(delay, bg=c["surface"])
        drow.grid(row=0, column=0, sticky="ew")
        drow.grid_columnconfigure(0, weight=1)

        self.checkbox(drow, 0, 0, "Use Starting Minute", self.vars["use_start_minute"])
        self.widgets["standalone_start_minute"] = ttk.Spinbox(
            drow, from_=0, to=59, width=6, textvariable=self.vars["start_minute"]
        )
        self.widgets["standalone_start_minute"].grid(row=0, column=1, sticky="w")

        # Transmission settings
        tx = self.section(right, "Transmission Settings", 0)
        ttk.Radiobutton(
            tx,
            text="Use Cable Transmission",
            variable=self.vars["transmission"],
            value="cable",
        ).grid(row=0, column=0, sticky="w", pady=2)
        ttk.Radiobutton(
            tx,
            text="Use Speaker Transmission",
            variable=self.vars["transmission"],
            value="speaker",
        ).grid(row=1, column=0, sticky="w", pady=2)

        # FSK settings
        fsk = self.section(right, "FSK Parameters", 1)
        row = ttk.Frame(fsk)
        row.grid(row=0, column=0, sticky="w")

        ttk.Label(row, text="Low (Hz):").grid(row=0, column=0, sticky="w")
        self.widgets["standalone_frequency_low"] = ttk.Entry(
            row, width=8, textvariable=self.vars["frequency_low"]
        )
        self.widgets["standalone_frequency_low"].grid(
            row=0, column=1, padx=(8, 20), sticky="w"
        )

        ttk.Label(row, text="High (Hz):").grid(row=0, column=2, sticky="w")
        self.widgets["standalone_frequency_high"] = ttk.Entry(
            row, width=8, textvariable=self.vars["frequency_high"]
        )
        self.widgets["standalone_frequency_high"].grid(
            row=0, column=3, padx=(8, 0), sticky="w"
        )

        ToolTip(
            self.widgets["standalone_frequency_low"],
            "Lower FSK frequency (Hz). Must be < High frequency.\nValid range: 2000–24000 Hz.",
            self.colors,
        )
        ToolTip(
            self.widgets["standalone_frequency_high"],
            "Higher FSK frequency (Hz). Must be > Low frequency.\nValid range: 2000–24000 Hz.",
            self.colors,
        )

        self.widgets["standalone_freq_gap_label"] = ttk.Label(
            fsk, text="", style="Muted.TLabel"
        )
        self.widgets["standalone_freq_gap_label"].grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )

        # Attenuation settings
        attenuation = self.section(right, "Attenuation", 2)
        row = ttk.Frame(attenuation)
        row.grid(row=0, column=0, sticky="w")

        ttk.Label(row, text="Level:").grid(row=0, column=0, sticky="w")
        self.widgets["standalone_attenuation"] = ttk.Spinbox(
            row, from_=0, to=100, width=6, textvariable=self.vars["attenuation"]
        )
        self.widgets["standalone_attenuation"].grid(
            row=0, column=1, padx=(8, 4), sticky="w"
        )
        ttk.Label(row, text="%").grid(row=0, column=2, sticky="w", padx=(0, 12))

        self.widgets["standalone_attenuation_db_label"] = ttk.Label(
            row, text="", style="Muted.TLabel"
        )
        self.widgets["standalone_attenuation_db_label"].grid(
            row=0, column=3, sticky="w"
        )

        # ECC Settings
        ecc = self.section(right, "ECC Settings", 3)
        row = ttk.Frame(ecc)
        row.grid(row=0, column=0, sticky="w")

        self.checkbox(
            row, 0, 0, "Enable ECC (Recommended)", self.vars["ecc_enabled"]
        ).grid(row=0, column=0, sticky="w")
        self.widgets["standalone_ecc_level"] = ttk.Spinbox(
            row, from_=0, to=100, width=6, textvariable=self.vars["ecc_level"]
        )
        self.widgets["standalone_ecc_level"].grid(
            row=0, column=1, padx=(8, 0), sticky="w"
        )
        ToolTip(
            self.widgets["standalone_ecc_level"],
            "Number of Reed-Solomon error-correction symbols (0-100).\nHigher = more robust, but longer transmissions.",
            self.colors,
        )

    # -----------------------------------------------------
    # Bottom bar
    # -----------------------------------------------------
    def _build_bottom_bar(self, parent):
        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=(8, 6))

        bottom = ttk.Frame(parent)
        bottom.pack(fill="x", padx=8)

        # Row for checkbox + button
        row = ttk.Frame(bottom)
        row.pack(fill="x")

        # Left side: debug toggle
        self.checkbox(
            row,
            0,
            0,
            "Show Build Log",
            self.vars["show_log"],
            padx=(0, 0),
        ).pack(side="left")

        # Right side: build button
        self.widgets["build_btn"] = ttk.Button(
            row,
            text="Build & Flash",
            style="Primary.TButton",
            command=self.start_build_thread,
        )
        self.widgets["build_btn"].pack(side="right")

        # Progress bar
        self.widgets["progress"] = ttk.Progressbar(bottom, mode="indeterminate")
        self.widgets["progress"].pack(fill="x", pady=(6, 0))

        # Status row with dot indicator
        status_row = ttk.Frame(bottom)
        status_row.pack(pady=(4, 0))

        c = self.colors
        self.widgets["status_dot"] = tk.Canvas(
            status_row, width=12, height=12, bg=c["bg"], highlightthickness=0
        )
        self.widgets["status_dot"].pack(side="left", padx=(0, 6))
        self.widgets["status_dot_id"] = self.widgets["status_dot"].create_oval(
            2, 2, 10, 10, fill=c["muted"], outline=""
        )

        self.widgets["status"] = ttk.Label(status_row, text="Idle")
        self.widgets["status"].pack(side="left")

    # -----------------------------------------------------
    # Events / dynamic state
    # -----------------------------------------------------
    def _bind_events(self):
        self.vars["default_interval"].trace_add(
            "write", lambda *_: self.toggle_interval_field()
        )
        self.vars["use_start_minute"].trace_add(
            "write", lambda *_: self.toggle_delay_field()
        )
        self.vars["attenuation"].trace_add(
            "write", lambda *_: self.update_attenuation_db_label()
        )
        self.vars["ecc_enabled"].trace_add(
            "write", lambda *_: self.toggle_ecc_level_field()
        )

        for prefix in ("receiver", "standalone"):
            self.widgets[f"{prefix}_frequency_low"].bind(
                "<FocusOut>", lambda e: self.update_low_frequency()
            )
            self.widgets[f"{prefix}_frequency_low"].bind(
                "<Return>", lambda e: self.update_low_frequency()
            )
            self.widgets[f"{prefix}_frequency_high"].bind(
                "<FocusOut>", lambda e: self.update_high_frequency()
            )
            self.widgets[f"{prefix}_frequency_high"].bind(
                "<Return>", lambda e: self.update_high_frequency()
            )

        self.bind_all("<Button-1>", self.defocus_all, add="+")
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

    def _init_dynamic_state(self):
        self.toggle_interval_field()
        self.toggle_delay_field()
        self.toggle_ecc_level_field()
        self.update_low_frequency()
        self.update_high_frequency()
        self.update_attenuation_db_label()
        self.on_tab_changed()

    def toggle_interval_field(self):
        state = "disabled" if self.vars["default_interval"].get() else "normal"
        for key in ("base_interval", "standalone_interval"):
            self.widgets[key].config(state=state)

    def toggle_delay_field(self):
        state = "normal" if self.vars["use_start_minute"].get() else "disabled"
        for key in ("base_start_minute", "standalone_start_minute"):
            self.widgets[key].config(state=state)

    def toggle_ecc_level_field(self):
        state = "normal" if self.vars["ecc_enabled"].get() else "disabled"
        for key in ("receiver_ecc_level", "standalone_ecc_level"):
            self.widgets[key].config(state=state)

    def defocus_all(self, event):
        if isinstance(
            event.widget,
            (
                tk.Entry,
                tk.Spinbox,
                ttk.Entry,
                ttk.Spinbox,
                tk.Checkbutton,
                tk.Radiobutton,
                ttk.Checkbutton,
                ttk.Radiobutton,
                ttk.Notebook,
            ),
        ):
            return
        self.focus_set()

    # -----------------------------------------------------
    # Frequency helpers
    # -----------------------------------------------------
    def calculate_attenuation_db(self, x: int) -> str:
        try:
            value = 0.2 * float(x) / 100.0
            if value <= 0:
                return "-∞ dB"
            db = 20.0 * math.log10(value)
            return f"{db:.2f} dB"
        except Exception:
            return "Invalid value"

    def update_attenuation_db_label(self):
        try:
            x = int(self.vars["attenuation"].get())
            text = f"dB value: {self.calculate_attenuation_db(x)}"
        except Exception:
            text = "dB value: Invalid value"
        self.widgets["receiver_attenuation_db_label"].config(text=text)
        self.widgets["standalone_attenuation_db_label"].config(text=text)

    def tolerance_samples(
        self, target_samples: int, tolerance_percent: int = BIT_SAMPLE_TOLERANCE_PERCENT
    ) -> int:
        return int(round(target_samples * tolerance_percent / 100.0))

    def total_samples_within_tolerance(
        self,
        total_samples: int,
        target_samples: int,
        tolerance_percent: int = BIT_SAMPLE_TOLERANCE_PERCENT,
    ) -> bool:
        tol = self.tolerance_samples(target_samples, tolerance_percent)
        return (target_samples - tol) <= total_samples <= (target_samples + tol)

    def rounded_min_bit_samples(self, fs: int) -> int:
        return ((fs * MIN_BIT_US) + 500000) // 1000000

    def quantized_freq_from_samples(self, fs: int, samples_per_period: int) -> int:
        return (
            0 if samples_per_period <= 0 else int(math.floor(fs / samples_per_period))
        )

    def period_count_from_samples(
        self, min_bit_samples: int, samples_per_period: int
    ) -> int:
        return int(round(min_bit_samples / samples_per_period))

    def freq_diff_u16(self, a: int, b: int) -> int:
        return abs(a - b)

    def min_required_diff_hz(self, lower_freq: int) -> int:
        return 300 + (400000 // lower_freq)

    def adjust_low_frequency_to_valid(self, low_freq: float):
        min_bit_samples = self.rounded_min_bit_samples(FS_HZ)

        low_n = max(1, int(math.floor(FS_HZ / low_freq)))

        while True:
            low_q = self.quantized_freq_from_samples(FS_HZ, low_n)
            low_p = self.period_count_from_samples(min_bit_samples, low_n)
            low_total = low_n * low_p

            low_timing_ok = self.total_samples_within_tolerance(
                low_total, min_bit_samples
            )

            if low_timing_ok:
                break

            candidate_low_n = low_n + 1
            candidate_low_q = self.quantized_freq_from_samples(FS_HZ, candidate_low_n)
            if candidate_low_q < FREQ_MIN:
                break
            low_n = candidate_low_n

        low_q = self.quantized_freq_from_samples(FS_HZ, low_n)
        low_p = self.period_count_from_samples(min_bit_samples, low_n)
        low_total = low_n * low_p
        return low_q, low_n, low_p, low_total

    def adjust_high_frequency_to_valid(self, high_freq: float):
        min_bit_samples = self.rounded_min_bit_samples(FS_HZ)

        high_n = max(1, int(math.floor(FS_HZ / high_freq)))

        while True:
            high_q = self.quantized_freq_from_samples(FS_HZ, high_n)
            high_p = self.period_count_from_samples(min_bit_samples, high_n)
            high_total = high_n * high_p

            high_timing_ok = self.total_samples_within_tolerance(
                high_total, min_bit_samples
            )

            if high_timing_ok:
                break

            if high_n <= 1:
                break

            candidate_high_n = high_n - 1
            candidate_high_q = self.quantized_freq_from_samples(FS_HZ, candidate_high_n)
            if candidate_high_q > FREQ_MAX:
                break
            high_n = candidate_high_n

        high_q = self.quantized_freq_from_samples(FS_HZ, high_n)
        high_p = self.period_count_from_samples(min_bit_samples, high_n)
        high_total = high_n * high_p
        return high_q, high_n, high_p, high_total

    def _update_freq_gap_label(self):
        try:
            low = int(self.vars["frequency_low"].get())
            high = int(self.vars["frequency_high"].get())
            gap = high - low
            if gap > 0:
                text = f"Gap: {gap:,} Hz  ({low:,} Hz \u2192 {high:,} Hz)"
            else:
                text = "Gap: — (High must be greater than Low)"
            self.widgets["receiver_freq_gap_label"].config(text=text)
            self.widgets["standalone_freq_gap_label"].config(text=text)
        except Exception:
            pass

    def update_low_frequency(self):
        try:
            low = float(self.vars["frequency_low"].get())
        except Exception:
            return

        if low < FREQ_MIN:
            low = FREQ_MIN

        new_low, _, _, _ = self.adjust_low_frequency_to_valid(low)
        self.vars["frequency_low"].set(int(new_low))
        self._update_freq_gap_label()

    def update_high_frequency(self):
        try:
            high = float(self.vars["frequency_high"].get())
        except Exception:
            return

        if high > FREQ_MAX:
            high = FREQ_MAX

        new_high, _, _, _ = self.adjust_high_frequency_to_valid(high)
        self.vars["frequency_high"].set(int(new_high))
        self._update_freq_gap_label()

    def _set_status(self, text, dot_color=None):
        self.widgets["status"].config(text=text)
        if dot_color and "status_dot" in self.widgets:
            self.widgets["status_dot"].itemconfig(
                self.widgets["status_dot_id"], fill=dot_color
            )

    # -----------------------------------------------------
    # Validation
    # -----------------------------------------------------
    def valid_text(self, value: str, min_len=1, max_len=None):
        value = value.strip()
        if len(value) < min_len:
            return False
        if max_len is not None and len(value.strip('"')) > max_len:
            return False
        return True

    def valid_int(self, value, min_val=None, max_val=None):
        try:
            x = int(str(value).strip())
        except Exception:
            return False
        if min_val is not None and x < min_val:
            return False
        if max_val is not None and x > max_val:
            return False
        return True

    def validate_all_fields(self):
        errors = []

        if not self.valid_text(self.vars["user_string"].get(), max_len=48):
            errors.append("User String must be 1–48 characters")

        if not self.valid_int(self.vars["device_id"].get(), 0, 99):
            errors.append("Device ID must be an integer between 0 and 99")

        if not self.valid_text(self.vars["location"].get(), max_len=18):
            errors.append("Location must be 1–18 characters")

        if not self.vars["default_interval"].get():
            if not self.valid_int(self.vars["interval"].get(), 1, 1440):
                errors.append("Interval must be an integer between 1 and 1440 minutes")

        if self.vars["use_start_minute"].get():
            if not self.valid_int(self.vars["start_minute"].get(), 0, 59):
                errors.append(
                    "Initial Delay must be an integer between 0 and 59 minutes"
                )

        low = self.vars["frequency_low"].get()
        high = self.vars["frequency_high"].get()

        if not self.valid_int(low, FREQ_MIN, FREQ_MAX):
            errors.append(
                f"Lower Frequency must be an integer between {FREQ_MIN} and {FREQ_MAX} Hz"
            )

        if not self.valid_int(high, FREQ_MIN, FREQ_MAX):
            errors.append(
                f"Higher Frequency must be an integer between {FREQ_MIN} and {FREQ_MAX} Hz"
            )

        if self.valid_int(low, FREQ_MIN, FREQ_MAX) and self.valid_int(
            high, FREQ_MIN, FREQ_MAX
        ):
            low = int(low)
            high = int(high)

            if low >= high:
                errors.append("Lower Frequency must be less than Higher Frequency")

        if not self.valid_int(self.vars["attenuation"].get(), 0, 100):
            errors.append("Attenuation must be an integer between 0 and 100")

        if self.notebook.index(self.notebook.select()) == 1:  # Receiver tab
            if (
                not self.vars["include_user_string"].get()
                and not self.vars["include_device_id"].get()
                and not self.vars["include_location"].get()
                and not self.vars["include_time"].get()
                and not self.vars["include_temperature"].get()
            ):
                errors.append("At least one field must be included in the watermark")
        else:  # Base tab
            if (
                not self.vars["include_user_string"].get()
                and not self.vars["include_time"].get()
            ):
                errors.append("At least User String or Timestamp must be included")

        if errors:
            messagebox.showwarning(
                "Invalid Input",
                "⚠️ Please correct the following fields:\n\n"
                + "\n".join(f"• {e}" for e in errors),
            )
            return False

        return True

    # -----------------------------------------------------
    # Build
    # -----------------------------------------------------
    def start_build_thread(self):
        if not self.validate_all_fields():
            return

        self.widgets["build_btn"].config(state="disabled")
        self._set_status("Building...", dot_color=self.colors["accent"])
        self.widgets["progress"].start(12)

        threading.Thread(target=self._run_build, daemon=True).start()

    def _run_build(self):
        try:
            ok = build_flash(
                self,
                self.vars["show_log"],
                self.widgets["build_btn"],
            )

            if not ok:
                self.after(0, lambda: self._finish_build(False, "❌ Build failed"))
                return

            self.after(
                0,
                lambda: self._finish_build(
                    True,
                    "✅ Build & Flash completed successfully!",
                    show_popup=True,
                ),
            )

        except Exception as e:
            self.after(
                0,
                lambda: self._finish_build(False, "❌ Build failed", error=str(e)),
            )

    def _finish_build(self, success, status_text, show_popup=False, error=None):
        self.widgets["progress"].stop()
        dot = self.colors["success"] if success else "#ff5555"
        self._set_status(status_text, dot_color=dot)
        self.widgets["build_btn"].config(state="normal")

        if show_popup:
            messagebox.showinfo(
                "Build & Flash", "Build and Flash completed successfully!"
            )

        if error:
            messagebox.showerror("Error", f"❌ {error}")

        self.after(
            4000, lambda: self._set_status("Idle", dot_color=self.colors["muted"])
        )


if __name__ == "__main__":
    app = FlashToolApp()
    app.mainloop()
