"""
Sensor Monitor V4.1 - Universal CO2 + ENV Monitor
Author: Zell
Date: 2026/07/09

Supports both ESP32-S3 CO2 sensor (ZELL_S3CO2) and ESP32-C3 ENV sensor (ZELL_C3ENV).
Displays: CO2, Temperature, Humidity, Pressure, Altitude (toggleable).
Data sources: BLE advertising + Serial (JSON).
V3.5: Non-linear CO2 Y-axis — stretches 400-1000 ppm range for detail.
V3.6: Day/Night theme toggle in Settings for eye-comfort dark mode.
V3.7: Night mode polish — button contrast, entry/history dark colors, per-device row tint.
V3.8: BLE packet v0x03 support (S3 CO2+Pressure from BMP580), Serial JSON Pressure_BMP580 field.
V3.9: Per-device pressure/altitude chart colors (S3/C3 differentiation),
      configurable chart line colors via Settings dropdown (12 colors).
V4.0: Double-click chart to open maximized in new window;
      Max/Min statistics for Temperature, Humidity, CO2.
V4.1: Pressure max/min display; per-device stats selection; custom device naming;
      settings persistence to JSON config file.

Required pip packages:
    pip install matplotlib bleak pyserial pillow requests openpyxl
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import asyncio
import json
import struct
import time
import csv
import math
import os
import urllib.request
import urllib.error
from datetime import datetime
from collections import deque

from PIL import Image, ImageTk

import serial
import serial.tools.list_ports
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.scale import FuncScale
from matplotlib.ticker import FixedLocator, FuncFormatter
import matplotlib.dates as mdates

try:
    from bleak import BleakScanner
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False

# --- Constants ---
APP_VERSION = {
    "name": "Sensor Monitor",
    "version": "V4.1",
    "date": "09.July.2026",
    "author": "Zell",
}
WINDOW_TITLE = "Sensor Monitor V4.1 — CO2 + ENV (S3/C3)"
WINDOW_SIZE = "1300x950"

# CO2 Y-axis adaptive non-linear scale
# Dynamically adjusts focus range based on current max CO2 value:
#   max <= 400:  focus 300-800,  Y range 300-1200
#   max <= 600:  focus 400-600,  Y range 300-1200
#   max <= 800:  focus 300-800,  Y range 300-1200
#   max <= 1000: focus 500-1000, Y range 300-1500
#   max <= 1200: focus 800-1200, Y range 300-1500
#   max > 1200:  focus 800-max,  Y range 300-2000
CO2_Y_MIN = 300
CO2_FOCUS_RATIO = 0.75  # focused zone gets 75% of display space


def _make_co2_scale(max_val):
    """Build forward/inverse functions for the current CO2 range."""
    if max_val <= 400:
        focus_lo, focus_hi, y_max = 300, 800, 1200
    elif max_val <= 600:
        focus_lo, focus_hi, y_max = 400, 600, 1200
    elif max_val <= 800:
        focus_lo, focus_hi, y_max = 300, 800, 1200
    elif max_val <= 1000:
        focus_lo, focus_hi, y_max = 500, 1000, 1500
    elif max_val <= 1200:
        focus_lo, focus_hi, y_max = 800, 1200, 1500
    else:
        focus_lo, focus_hi, y_max = 800, max(max_val, 1500), 2000

    y_min = CO2_Y_MIN
    below_range = focus_lo - y_min
    focus_range = focus_hi - focus_lo
    above_range = y_max - focus_hi

    # display space allocation
    if below_range > 0 and above_range > 0:
        below_ratio = (1.0 - CO2_FOCUS_RATIO) * below_range / (below_range + above_range)
        above_ratio = (1.0 - CO2_FOCUS_RATIO) - below_ratio
    elif below_range > 0:
        below_ratio = 1.0 - CO2_FOCUS_RATIO
        above_ratio = 0.0
    else:
        below_ratio = 0.0
        above_ratio = 1.0 - CO2_FOCUS_RATIO
    focus_ratio = CO2_FOCUS_RATIO

    def forward(ppm):
        ppm = np.asarray(ppm, dtype=float)
        result = np.zeros_like(ppm)
        m_below = ppm <= focus_lo
        m_focus = (ppm > focus_lo) & (ppm <= focus_hi)
        m_above = ppm > focus_hi
        if below_range > 0:
            result[m_below] = (ppm[m_below] - y_min) / below_range * below_ratio
        if focus_range > 0:
            result[m_focus] = below_ratio + (ppm[m_focus] - focus_lo) / focus_range * focus_ratio
        if above_range > 0:
            result[m_above] = below_ratio + focus_ratio + (ppm[m_above] - focus_hi) / above_range * above_ratio
        return result

    def inverse(disp):
        disp = np.asarray(disp, dtype=float)
        result = np.zeros_like(disp)
        m_below = disp <= below_ratio
        m_focus = (disp > below_ratio) & (disp <= below_ratio + focus_ratio)
        m_above = disp > below_ratio + focus_ratio
        if below_ratio > 0:
            result[m_below] = disp[m_below] / below_ratio * below_range + y_min
        else:
            result[m_below] = y_min
        if focus_ratio > 0:
            result[m_focus] = (disp[m_focus] - below_ratio) / focus_ratio * focus_range + focus_lo
        if above_ratio > 0:
            result[m_above] = (disp[m_above] - below_ratio - focus_ratio) / above_ratio * above_range + focus_hi
        else:
            result[m_above] = focus_hi
        return result

    return forward, inverse, y_min, y_max, focus_lo, focus_hi


SERIAL_BAUD = 115200
BLE_DEVICE_NAMES = ["ZELL_S3CO2", "ZELL_C3ENV"]
BLE_COMPANY_ID = 0x90FC
BLE_SERVICE_UUID = "0000fc90-0000-1000-8000-00805f9b34fb"
MAX_DATA_POINTS = 1000
MAX_LOG_LINES = 500
CHART_UPDATE_INTERVAL_MS = 2000
BLE_SCAN_INTERVAL_S = 3.0

SEA_LEVEL_PRESSURE_HPA_DEFAULT = 1013.25
# Default location: Hangzhou (30.25°N, 120.17°E)
LOCATION_LAT = 30.25
LOCATION_LON = 120.17
LOCATION_NAME = "Hangzhou"
SEA_LEVEL_PRESSURE_UPDATE_INTERVAL_S = 1800  # update every 30 minutes

APP_ICON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images", "iot-sensors-rgb-color-icon-vector.jpg")
CONFIG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_monitor_config.json")

# Chart line color palette (16 common colors, no white)
CHART_COLOR_PALETTE = {
    "Red": "#F44336",
    "Deep Orange": "#FF5722",
    "Orange": "#FF8C00",
    "Amber": "#FFC107",
    "Yellow": "#FFEB3B",
    "Green": "#4CAF50",
    "Dark Green": "#2E7D32",
    "Teal": "#009688",
    "Cyan": "#00BCD4",
    "Blue": "#2196F3",
    "Dark Blue": "#1565C0",
    "Purple": "#9C27B0",
    "Indigo": "#3F51B5",
    "Brown": "#795548",
    "Black": "#000000",
    "Gray": "#607D8B",
    "Pink": "#E91E63",
}
CHART_COLOR_NAMES = list(CHART_COLOR_PALETTE.keys()) + ["Custom..."]
CHART_COLOR_HEX_TO_NAME = {v: k for k, v in CHART_COLOR_PALETTE.items()}

# Day/Night theme definitions
THEME_DAY = {
    "name": "Day",
    "bg": "#f0f0f0",
    "fg": "#000000",
    "frame_bg": "#f0f0f0",
    "label_fg": "#333333",
    "entry_bg": "#ffffff",
    "entry_fg": "#000000",
    "btn_bg": "#e1e1e1",
    "btn_fg": "#000000",
    "btn_active_bg": "#c8c8c8",
    "log_bg": "black",
    "log_fg": "#00FF00",
    "chart_face": "#f8f8f8",
    "chart_text": "#000000",
    "chart_grid": "#cccccc",
    "chart_axes_bg": "#ffffff",
    "stat_fg": "#555555",
    "led_outline": "darkgray",
    "tree_bg": "#ffffff",
    "tree_fg": "#000000",
    "tree_heading_bg": "#e1e1e1",
    "tree_heading_fg": "#000000",
    "tree_selected_bg": "#cce5ff",
    "tree_s3_bg": "#C8E6C9",
    "tree_s3_fg": "#000000",
    "tree_c3_bg": "#E3F2FD",
    "tree_c3_fg": "#000000",
}
THEME_NIGHT = {
    "name": "Night",
    "bg": "#1e1e2e",
    "fg": "#cdd6f4",
    "frame_bg": "#1e1e2e",
    "label_fg": "#bac2de",
    "entry_bg": "#2a2040",
    "entry_fg": "#cdd6f4",
    "btn_bg": "#1a3a2a",
    "btn_fg": "#e0e0e0",
    "btn_active_bg": "#2d5a3f",
    "log_bg": "#11111b",
    "log_fg": "#a6e3a1",
    "chart_face": "#1e1e2e",
    "chart_text": "#cdd6f4",
    "chart_grid": "#45475a",
    "chart_axes_bg": "#181825",
    "stat_fg": "#9399b2",
    "led_outline": "#585b70",
    "tree_bg": "#11111b",
    "tree_fg": "#cdd6f4",
    "tree_heading_bg": "#2a4035",
    "tree_heading_fg": "#e0e0e0",
    "tree_selected_bg": "#3a5a4a",
    "tree_s3_bg": "#1a3328",
    "tree_s3_fg": "#a6e3a1",
    "tree_c3_bg": "#1a2533",
    "tree_c3_fg": "#89b4fa",
}

# Global mutable sea level pressure (updated at runtime from API)
_sea_level_pressure_hpa = SEA_LEVEL_PRESSURE_HPA_DEFAULT
_sea_level_pressure_source = "default"


def fetch_sea_level_pressure(lat=LOCATION_LAT, lon=LOCATION_LON):
    """Fetch current sea-level pressure from Open-Meteo API (free, no key needed)."""
    url = (
        f"https://api.open-meteo.com/v1/forecast?"
        f"latitude={lat}&longitude={lon}&current=pressure_msl&timezone=auto"
    )
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "SensorMonitor/3.7"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            pressure = data.get("current", {}).get("pressure_msl")
            if pressure and 900 < pressure < 1100:
                return float(pressure)
    except Exception:
        pass
    return None


def update_sea_level_pressure():
    """Update global sea level pressure from online source."""
    global _sea_level_pressure_hpa, _sea_level_pressure_source
    p = fetch_sea_level_pressure()
    if p is not None:
        _sea_level_pressure_hpa = p
        _sea_level_pressure_source = f"Open-Meteo ({LOCATION_NAME})"
        return True
    return False


def get_sea_level_pressure():
    """Get current sea level pressure value."""
    return _sea_level_pressure_hpa


def pressure_to_altitude(pressure_hpa):
    """Calculate altitude from pressure using barometric formula."""
    if pressure_hpa is None or pressure_hpa <= 0:
        return None
    return 44330.0 * (1.0 - math.pow(pressure_hpa / _sea_level_pressure_hpa, 0.1903))


def _load_config():
    try:
        with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _save_config(cfg):
    try:
        with open(CONFIG_FILE_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except OSError:
        pass


class SensorData:
    """Container for a single sensor reading."""
    def __init__(self, timestamp=None, source="", device="", temp=None, hum=None,
                 co2=None, pressure=None, altitude=None, rssi=None):
        self.timestamp = timestamp or datetime.now()
        self.source = source
        self.device = device  # "S3" or "C3"
        self.temp = temp
        self.hum = hum
        self.co2 = co2
        self.pressure = pressure
        self.altitude = altitude
        self.rssi = rssi
        self.alarm = ""


class SensorMonitorV3:
    def __init__(self, root):
        self.root = root
        self.root.title(WINDOW_TITLE)
        self.root.geometry(WINDOW_SIZE)
        self.root.minsize(1000, 750)

        # Set window icon
        self._app_icon = None
        self._app_icon_about = None
        try:
            icon_img = Image.open(APP_ICON_PATH)
            icon_photo = ImageTk.PhotoImage(icon_img.resize((32, 32), Image.LANCZOS))
            self._app_icon = icon_photo
            self.root.iconphoto(True, icon_photo)
        except Exception:
            pass

        # Data storage
        self.data_history = deque(maxlen=MAX_DATA_POINTS)
        self.timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.co2_values = deque(maxlen=MAX_DATA_POINTS)
        self.temp_values = deque(maxlen=MAX_DATA_POINTS)
        self.hum_values = deque(maxlen=MAX_DATA_POINTS)
        self.pressure_values = deque(maxlen=MAX_DATA_POINTS)
        self.altitude_values = deque(maxlen=MAX_DATA_POINTS)

        # Per-device data storage for dual-device chart display
        self.s3_timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.s3_temp_values = deque(maxlen=MAX_DATA_POINTS)
        self.s3_hum_values = deque(maxlen=MAX_DATA_POINTS)
        self.s3_pressure_timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.s3_pressure_values = deque(maxlen=MAX_DATA_POINTS)
        self.s3_altitude_values = deque(maxlen=MAX_DATA_POINTS)
        self.c3_timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.c3_temp_values = deque(maxlen=MAX_DATA_POINTS)
        self.c3_hum_values = deque(maxlen=MAX_DATA_POINTS)
        self.c3_pressure_timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.c3_pressure_values = deque(maxlen=MAX_DATA_POINTS)
        self.c3_altitude_values = deque(maxlen=MAX_DATA_POINTS)

        # State
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.ble_thread = None
        self.ble_running = False
        self.ble_loop = None
        self.last_chart_update = 0
        self.new_data_flag = False
        self.show_altitude = tk.BooleanVar(value=False)
        self.show_pressure = tk.BooleanVar(value=True)
        self.show_co2 = tk.BooleanVar(value=True)

        # BLE connection tracking
        self.start_time = time.time()
        self.ble_device_status = {}  # {device_name: {"connected": bool, "last_data_time": None, "data_count": 0}}
        self.total_data_received = 0
        self.console_print_enabled = True
        self.current_theme = THEME_DAY

        # Chart line colors (configurable in Settings)
        self.chart_colors = {
            "temp_s3": "#FF5722",
            "temp_c3": "#FF8C00",
            "temp_single": "#FF8C00",
            "hum_s3": "#1565C0",
            "hum_c3": "#4CAF50",
            "hum_single": "#2196F3",
            "press_s3": "#009688",
            "press_c3": "#4CAF50",
            "press_single": "#4CAF50",
            "alt_s3": "#795548",
            "alt_c3": "#FF8C00",
            "alt_single": "#795548",
        }

        # Current values
        self.current_temp = tk.StringVar(value="--.-")
        self.current_hum = tk.StringVar(value="--.-")
        self.current_co2 = tk.StringVar(value="---")
        self.current_pressure = tk.StringVar(value="----.-")
        self.current_altitude = tk.StringVar(value="---.-")
        self.current_co2_level = tk.StringVar(value="")
        self.ble_status = tk.StringVar(value="Idle")
        self.device_label = tk.StringVar(value="--")

        # Statistics values
        self.stat_temp_avg = tk.StringVar(value="--.-")
        self.stat_temp_max = tk.StringVar(value="--.-")
        self.stat_temp_min = tk.StringVar(value="--.-")
        self.stat_hum_avg = tk.StringVar(value="--.-")
        self.stat_hum_max = tk.StringVar(value="--.-")
        self.stat_hum_min = tk.StringVar(value="--.-")
        self.stat_co2_avg = tk.StringVar(value="---")
        self.stat_co2_max = tk.StringVar(value="---")
        self.stat_co2_min = tk.StringVar(value="---")
        self.stat_pressure_avg = tk.StringVar(value="----.-")
        self.stat_pressure_max = tk.StringVar(value="----.-")
        self.stat_pressure_min = tk.StringVar(value="----.-")
        self.stat_alt_min = tk.StringVar(value="---.-")
        self.stat_alt_max = tk.StringVar(value="---.-")
        self.stat_sea_level_p = tk.StringVar(value=f"{_sea_level_pressure_hpa:.1f}")

        # Device naming and stats device selection
        self.stats_device = "All"
        self.s3_name_suffix = ""
        self.c3_name_suffix = ""

        # Alarm thresholds
        self.alarm_enabled = True
        self.alarm_temp_high = 35.0
        self.alarm_temp_low = 10.0
        self.alarm_co2_high = 1000
        self.alarm_co2_low = 400
        self.alarm_hum_high = 80.0
        self.alarm_hum_low = 20.0

        # Load persisted config
        self._load_persisted_config()

        self._build_ui()
        self._start_ble_scanner()
        self._schedule_chart_update()
        self._schedule_console_status()
        self._start_pressure_updater()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _load_persisted_config(self):
        global _sea_level_pressure_hpa, _sea_level_pressure_source
        global LOCATION_LAT, LOCATION_LON, LOCATION_NAME
        cfg = _load_config()
        if not cfg:
            return
        self.s3_name_suffix = cfg.get("s3_name_suffix", "")[:12]
        self.c3_name_suffix = cfg.get("c3_name_suffix", "")[:12]
        self.stats_device = cfg.get("stats_device", "All")
        if self.stats_device not in ("All", "S3", "C3"):
            self.stats_device = "All"
        if "chart_colors" in cfg:
            for k, v in cfg["chart_colors"].items():
                if k in self.chart_colors:
                    self.chart_colors[k] = v
        if cfg.get("theme") == "Night":
            self.current_theme = THEME_NIGHT
        if "location_name" in cfg:
            LOCATION_NAME = cfg["location_name"]
        if "location_lat" in cfg:
            try:
                LOCATION_LAT = float(cfg["location_lat"])
            except (ValueError, TypeError):
                pass
        if "location_lon" in cfg:
            try:
                LOCATION_LON = float(cfg["location_lon"])
            except (ValueError, TypeError):
                pass
        if "sea_level_pressure" in cfg:
            try:
                p = float(cfg["sea_level_pressure"])
                if 900 < p < 1100:
                    _sea_level_pressure_hpa = p
                    _sea_level_pressure_source = f"Config ({LOCATION_NAME})"
                    self.stat_sea_level_p.set(f"{p:.1f}")
            except (ValueError, TypeError):
                pass
        if "console_print" in cfg:
            self.console_print_enabled = cfg["console_print"]
        if "alarm_enabled" in cfg:
            self.alarm_enabled = bool(cfg["alarm_enabled"])
        for key in ("alarm_temp_high", "alarm_temp_low", "alarm_hum_high", "alarm_hum_low"):
            if key in cfg:
                try:
                    setattr(self, key, float(cfg[key]))
                except (ValueError, TypeError):
                    pass
        for key in ("alarm_co2_high", "alarm_co2_low"):
            if key in cfg:
                try:
                    setattr(self, key, int(cfg[key]))
                except (ValueError, TypeError):
                    pass

    def _get_s3_display_name(self):
        if self.s3_name_suffix:
            return f"S3 {self.s3_name_suffix}"
        return "S3"

    def _get_c3_display_name(self):
        if self.c3_name_suffix:
            return f"C3 {self.c3_name_suffix}"
        return "C3"

    # =========================================================================
    # UI Construction
    # =========================================================================
    def _build_ui(self):
        self.main_pane = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        self.main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        top_frame = ttk.Frame(self.main_pane)
        self.main_pane.add(top_frame, weight=0)
        self._build_top_frame(top_frame)

        serial_frame = ttk.LabelFrame(self.main_pane, text="Serial Port")
        self.main_pane.add(serial_frame, weight=0)
        self._build_serial_frame(serial_frame)

        log_frame = ttk.LabelFrame(self.main_pane, text="UART Log")
        self.main_pane.add(log_frame, weight=1)
        self._build_log_frame(log_frame)

        chart_frame = ttk.Frame(self.main_pane)
        self.main_pane.add(chart_frame, weight=4)
        self._build_chart_frame(chart_frame)

        history_frame = ttk.LabelFrame(self.main_pane, text="History")
        self.main_pane.add(history_frame, weight=1)
        self._build_history_frame(history_frame)

    def _build_top_frame(self, parent):
        # Sensor values panel (left, expandable)
        values_frame = ttk.LabelFrame(parent, text="Current Sensor Values", padding=(5, 2))
        values_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Top row: BLE indicator (inline, horizontal)
        status_row = ttk.Frame(values_frame)
        status_row.pack(fill=tk.X, pady=(0, 1))
        ble_ind_frame = ttk.Frame(status_row)
        ble_ind_frame.pack(side=tk.LEFT)
        self.ble_led = tk.Canvas(ble_ind_frame, width=14, height=14, highlightthickness=0)
        self.ble_led.pack(side=tk.LEFT)
        self.ble_led_oval = self.ble_led.create_oval(2, 2, 12, 12, fill="gray", outline="darkgray")
        ttk.Label(ble_ind_frame, text="BLE:", font=("Consolas", 9)).pack(side=tk.LEFT, padx=(3, 0))
        ttk.Label(ble_ind_frame, textvariable=self.ble_status, font=("Consolas", 9)).pack(side=tk.LEFT, padx=(2, 8))
        ttk.Label(ble_ind_frame, text="Device:", font=("Consolas", 9)).pack(side=tk.LEFT)
        ttk.Label(ble_ind_frame, textvariable=self.device_label, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=(2, 0))

        # Alarm LED indicators
        alarm_ind_frame = ttk.Frame(status_row)
        alarm_ind_frame.pack(side=tk.LEFT, padx=(20, 0))
        ttk.Label(alarm_ind_frame, text="Alarm:", font=("Consolas", 9)).pack(side=tk.LEFT, padx=(0, 4))

        self.alarm_temp_led = tk.Canvas(alarm_ind_frame, width=14, height=14, highlightthickness=0)
        self.alarm_temp_led.pack(side=tk.LEFT)
        self.alarm_temp_oval = self.alarm_temp_led.create_oval(2, 2, 12, 12, fill="gray", outline="darkgray")
        ttk.Label(alarm_ind_frame, text="T", font=("Consolas", 8)).pack(side=tk.LEFT, padx=(1, 6))

        self.alarm_co2_led = tk.Canvas(alarm_ind_frame, width=14, height=14, highlightthickness=0)
        self.alarm_co2_led.pack(side=tk.LEFT)
        self.alarm_co2_oval = self.alarm_co2_led.create_oval(2, 2, 12, 12, fill="gray", outline="darkgray")
        ttk.Label(alarm_ind_frame, text="CO2", font=("Consolas", 8)).pack(side=tk.LEFT, padx=(1, 6))

        self.alarm_hum_led = tk.Canvas(alarm_ind_frame, width=14, height=14, highlightthickness=0)
        self.alarm_hum_led.pack(side=tk.LEFT)
        self.alarm_hum_oval = self.alarm_hum_led.create_oval(2, 2, 12, 12, fill="gray", outline="darkgray")
        ttk.Label(alarm_ind_frame, text="H", font=("Consolas", 8)).pack(side=tk.LEFT, padx=(1, 0))

        # Sensor value grid
        grid_frame = ttk.Frame(values_frame)
        grid_frame.pack(fill=tk.X)
        labels = ["Temperature", "Humidity", "CO2", "Pressure", "Altitude"]
        units = ["C", "%", "ppm", "hPa", "m"]
        colors = ["#2196F3", "#4CAF50", "#FF5722", "#009688", "#795548"]
        vars_list = [self.current_temp, self.current_hum, self.current_co2,
                     self.current_pressure, self.current_altitude]

        # Load sensor icons (20x20)
        icon_files = ["Temperature_icon.png", "Humidity_icon.webp", "co2 icon.jpg",
                      "air pressure_icon.jpg", "altitude_icon.png"]
        self._sensor_icons = []
        img_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
        for fname in icon_files:
            if fname:
                try:
                    img = Image.open(os.path.join(img_dir, fname))
                    img = img.resize((20, 20), Image.LANCZOS)
                    photo = ImageTk.PhotoImage(img)
                    self._sensor_icons.append(photo)
                except Exception:
                    self._sensor_icons.append(None)
            else:
                self._sensor_icons.append(None)

        for i, (name, unit, color, var) in enumerate(zip(labels, units, colors, vars_list)):
            grid_frame.columnconfigure(i, weight=1)
            header_frame = ttk.Frame(grid_frame)
            header_frame.grid(row=0, column=i, padx=4, sticky=tk.W)
            if self._sensor_icons[i]:
                ttk.Label(header_frame, image=self._sensor_icons[i]).pack(side=tk.LEFT, padx=(0, 3))
            ttk.Label(header_frame, text=f"{name} ({unit})", font=("Segoe UI", 9)).pack(side=tk.LEFT)
            lbl = ttk.Label(grid_frame, textvariable=var, font=("Consolas", 16, "bold"), foreground=color)
            lbl.grid(row=1, column=i, padx=4, sticky=tk.W)

        # CO2 level indicator next to CO2 value
        self.co2_level_label = ttk.Label(grid_frame, textvariable=self.current_co2_level,
                                          font=("Arial", 9, "bold"))
        self.co2_level_label.grid(row=1, column=2, padx=(80, 0), sticky=tk.W)

        # Statistics rows: Avg on row 2, Max/Min on row 3 for temp/hum/co2; pressure avg row 2; altitude min/max row 2
        # Row 2: Avg values
        stat_avg_labels = ["Avg:", "Avg:", "Avg:", "Avg:", "Min/Max:"]
        stat_avg_vars = [self.stat_temp_avg, self.stat_hum_avg, self.stat_co2_avg,
                         self.stat_pressure_avg, self.stat_alt_min]
        for i, (slbl, svar) in enumerate(zip(stat_avg_labels, stat_avg_vars)):
            frame = ttk.Frame(grid_frame)
            frame.grid(row=2, column=i, padx=4, sticky=tk.W)
            ttk.Label(frame, text=slbl, font=("Segoe UI", 8)).pack(side=tk.LEFT)
            ttk.Label(frame, textvariable=svar, font=("Consolas", 9), foreground="#555555").pack(side=tk.LEFT)
            if i == 4:
                ttk.Label(frame, text="/", font=("Consolas", 9), foreground="#555555").pack(side=tk.LEFT)
                ttk.Label(frame, textvariable=self.stat_alt_max, font=("Consolas", 9), foreground="#555555").pack(side=tk.LEFT)

        # Row 3: Max/Min for Temperature, Humidity, CO2, Pressure
        stat_maxmin_vars = [
            (self.stat_temp_max, self.stat_temp_min),
            (self.stat_hum_max, self.stat_hum_min),
            (self.stat_co2_max, self.stat_co2_min),
            (self.stat_pressure_max, self.stat_pressure_min),
        ]
        for i, (max_var, min_var) in enumerate(stat_maxmin_vars):
            frame = ttk.Frame(grid_frame)
            frame.grid(row=3, column=i, padx=4, sticky=tk.W)
            ttk.Label(frame, text="H:", font=("Segoe UI", 8), foreground="#C62828").pack(side=tk.LEFT)
            ttk.Label(frame, textvariable=max_var, font=("Consolas", 9), foreground="#C62828").pack(side=tk.LEFT)
            ttk.Label(frame, text=" L:", font=("Segoe UI", 8), foreground="#1565C0").pack(side=tk.LEFT)
            ttk.Label(frame, textvariable=min_var, font=("Consolas", 9), foreground="#1565C0").pack(side=tk.LEFT)

        # Sea-level pressure display under Pressure column
        slp_frame = ttk.Frame(grid_frame)
        slp_frame.grid(row=4, column=3, padx=4, sticky=tk.W)
        ttk.Label(slp_frame, text="P0:", font=("Segoe UI", 8)).pack(side=tk.LEFT)
        ttk.Label(slp_frame, textvariable=self.stat_sea_level_p, font=("Consolas", 9), foreground="#009688").pack(side=tk.LEFT)
        ttk.Label(slp_frame, text="hPa", font=("Segoe UI", 7), foreground="#888").pack(side=tk.LEFT, padx=(2, 0))

        # Controls panel (right side, wider with two-column grid)
        ctrl_frame = ttk.LabelFrame(parent, text="Controls", padding=5)
        ctrl_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))

        ctrl_grid = ttk.Frame(ctrl_frame)
        ctrl_grid.pack(fill=tk.BOTH, expand=True)
        ctrl_grid.columnconfigure(0, weight=1, minsize=90)
        ctrl_grid.columnconfigure(1, weight=1, minsize=90)

        ttk.Button(ctrl_grid, text="Export CSV", command=self._export_csv).grid(
            row=0, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(ctrl_grid, text="Clear", command=self._clear_data).grid(
            row=0, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Checkbutton(ctrl_grid, text="CO2", variable=self.show_co2,
                        command=self._on_chart_toggle).grid(
            row=1, column=0, padx=2, pady=2, sticky=tk.W)
        ttk.Checkbutton(ctrl_grid, text="Pressure", variable=self.show_pressure,
                        command=self._on_chart_toggle).grid(
            row=1, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Checkbutton(ctrl_grid, text="Altitude", variable=self.show_altitude,
                        command=self._on_chart_toggle).grid(
            row=2, column=0, padx=2, pady=2, sticky=tk.W)
        self.lbl_records = ttk.Label(ctrl_grid, text="Records: 0", font=("Consolas", 9))
        self.lbl_records.grid(row=2, column=1, padx=2, pady=2, sticky=tk.W)
        ttk.Button(ctrl_grid, text="Settings", command=self._show_settings).grid(
            row=3, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(ctrl_grid, text="About", command=self._show_about).grid(
            row=3, column=1, padx=2, pady=2, sticky=tk.EW)

    def _build_serial_frame(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=3)

        ttk.Label(frame, text="Port:").pack(side=tk.LEFT, padx=2)
        self.port_combo = ttk.Combobox(frame, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self._toggle_serial)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        ttk.Label(frame, text="CMD:").pack(side=tk.LEFT, padx=2)
        self.cmd_entry = ttk.Entry(frame, width=25)
        self.cmd_entry.pack(side=tk.LEFT, padx=2)
        self.cmd_entry.bind("<Return>", self._send_serial_cmd)
        ttk.Button(frame, text="Send", command=self._send_serial_cmd).pack(side=tk.LEFT, padx=2)

        self._refresh_ports()

    def _build_log_frame(self, parent):
        self.log_text = tk.Text(parent, height=5, bg="black", fg="#00FF00",
                                font=("Consolas", 9), wrap=tk.WORD, state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _build_chart_frame(self, parent):
        self.fig = Figure(figsize=(13, 6), dpi=80, facecolor=self.current_theme["chart_face"])
        self._create_chart_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self._on_chart_double_click)
        self._maximized_windows = []

    def _create_chart_axes(self):
        """Create or recreate chart axes based on CO2/pressure/altitude toggles."""
        self.fig.clear()
        self.ax_co2 = None
        self.ax_press = None
        self.ax_alt = None

        show_c = self.show_co2.get()
        show_p = self.show_pressure.get()
        show_a = self.show_altitude.get()
        extra = int(show_p) + int(show_a)

        if show_c:
            if extra == 0:
                self.ax_co2 = self.fig.add_subplot(2, 2, (1, 2))
                self.ax_temp = self.fig.add_subplot(2, 2, 3)
                self.ax_hum = self.fig.add_subplot(2, 2, 4)
            elif extra == 1:
                self.ax_co2 = self.fig.add_subplot(2, 3, (1, 3))
                self.ax_temp = self.fig.add_subplot(2, 3, 4)
                self.ax_hum = self.fig.add_subplot(2, 3, 5)
                if show_p:
                    self.ax_press = self.fig.add_subplot(2, 3, 6)
                else:
                    self.ax_alt = self.fig.add_subplot(2, 3, 6)
            else:
                self.ax_co2 = self.fig.add_subplot(3, 2, (1, 2))
                self.ax_temp = self.fig.add_subplot(3, 2, 3)
                self.ax_hum = self.fig.add_subplot(3, 2, 4)
                self.ax_press = self.fig.add_subplot(3, 2, 5)
                self.ax_alt = self.fig.add_subplot(3, 2, 6)
        else:
            if extra == 0:
                self.ax_temp = self.fig.add_subplot(1, 2, 1)
                self.ax_hum = self.fig.add_subplot(1, 2, 2)
            elif extra == 1:
                self.ax_temp = self.fig.add_subplot(2, 2, 1)
                self.ax_hum = self.fig.add_subplot(2, 2, 2)
                if show_p:
                    self.ax_press = self.fig.add_subplot(2, 2, (3, 4))
                else:
                    self.ax_alt = self.fig.add_subplot(2, 2, (3, 4))
            else:
                self.ax_temp = self.fig.add_subplot(2, 2, 1)
                self.ax_hum = self.fig.add_subplot(2, 2, 2)
                self.ax_press = self.fig.add_subplot(2, 2, 3)
                self.ax_alt = self.fig.add_subplot(2, 2, 4)

        self.fig.subplots_adjust(hspace=0.5, wspace=0.3, top=0.95, bottom=0.08)

    def _on_chart_toggle(self):
        self._create_chart_axes()
        self.new_data_flag = True

    def _on_chart_double_click(self, event):
        """Handle double-click on chart to open maximized in new window."""
        if event.dblclick and event.inaxes is not None:
            ax = event.inaxes
            chart_name = self._get_chart_name(ax)
            if chart_name:
                self._open_maximized_chart(chart_name)

    def _get_chart_name(self, ax):
        """Identify which chart an axes belongs to."""
        if ax == self.ax_co2:
            return "CO2"
        elif ax == self.ax_temp:
            return "Temperature"
        elif ax == self.ax_hum:
            return "Humidity"
        elif ax == self.ax_press:
            return "Pressure"
        elif ax == self.ax_alt:
            return "Altitude"
        return None

    def _open_maximized_chart(self, chart_name):
        """Open a chart in a new maximized window with live updates."""
        win = tk.Toplevel(self.root)
        win.title(f"{chart_name} — Maximized Chart")
        win.state("zoomed")

        theme = self.current_theme
        fig = Figure(dpi=100, facecolor=theme["chart_face"])
        ax = fig.add_subplot(1, 1, 1)
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        win_info = {"win": win, "fig": fig, "ax": ax, "canvas": canvas,
                    "chart_name": chart_name, "alive": True}
        self._maximized_windows.append(win_info)

        def _on_close_max():
            win_info["alive"] = False
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", _on_close_max)
        self._refresh_maximized_chart(win_info)
        self._schedule_maximized_update(win_info)

    def _schedule_maximized_update(self, win_info):
        """Schedule periodic refresh for maximized chart window."""
        if not win_info["alive"]:
            return
        try:
            win_info["win"].winfo_exists()
        except tk.TclError:
            win_info["alive"] = False
            return
        self._refresh_maximized_chart(win_info)
        win_info["win"].after(CHART_UPDATE_INTERVAL_MS, self._schedule_maximized_update, win_info)

    def _refresh_maximized_chart(self, win_info):
        """Redraw a single chart in maximized window."""
        if not win_info["alive"]:
            return
        ax = win_info["ax"]
        fig = win_info["fig"]
        canvas = win_info["canvas"]
        chart_name = win_info["chart_name"]
        theme = self.current_theme

        ax.clear()
        fig.set_facecolor(theme["chart_face"])
        ax.set_facecolor(theme["chart_axes_bg"])

        if chart_name == "CO2":
            self._draw_co2_chart(ax)
        elif chart_name == "Temperature":
            self._draw_temp_chart(ax)
        elif chart_name == "Humidity":
            self._draw_hum_chart(ax)
        elif chart_name == "Pressure":
            self._draw_pressure_chart(ax)
        elif chart_name == "Altitude":
            self._draw_altitude_chart(ax)

        ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
        ax.tick_params(axis="x", rotation=30, labelsize=9, colors=theme["chart_text"])
        ax.tick_params(axis="y", labelsize=10, colors=theme["chart_text"])
        ax.title.set_color(theme["chart_text"])
        ax.xaxis.label.set_color(theme["chart_text"])
        ax.yaxis.label.set_color(theme["chart_text"])
        ax.grid(True, alpha=0.3, color=theme["chart_grid"])
        for spine in ax.spines.values():
            spine.set_color(theme["chart_grid"])

        fig.tight_layout(pad=2.0)
        canvas.draw_idle()

    def _draw_co2_chart(self, ax):
        """Draw CO2 chart on given axes (used by both main and maximized)."""
        if not self.co2_values:
            ax.set_title("CO2 (ppm)", fontsize=12, fontweight="bold")
            ax.set_ylabel("ppm")
            return
        ts = list(self.timestamps)[-len(self.co2_values):]
        vals = list(self.co2_values)
        current_max = max(vals)
        fwd, inv, y_min, y_max, focus_lo, focus_hi = _make_co2_scale(current_max)

        ax.set_yscale("function", functions=(fwd, inv))
        ax.set_title(f"CO2 (ppm) [focus: {focus_lo}-{focus_hi}]", fontsize=12, fontweight="bold")
        ax.set_ylabel("ppm")

        ax.axhspan(y_min, 800, alpha=0.08, color="green")
        ax.axhspan(800, min(1200, y_max), alpha=0.08, color="yellow")
        if y_max > 1200:
            ax.axhspan(1200, y_max, alpha=0.08, color="red")

        for i in range(len(vals) - 1):
            seg_val = (vals[i] + vals[i + 1]) / 2.0
            color = self._co2_value_to_color(seg_val)
            ax.plot(ts[i:i+2], vals[i:i+2], color=color, linewidth=2, solid_capstyle="round")
        for i, v in enumerate(vals):
            ax.plot(ts[i], v, ".", color=self._co2_value_to_color(v), markersize=5)

        ax.axhline(y=800, color="orange", linestyle="--", alpha=0.5, linewidth=0.8)
        if y_max >= 1200:
            ax.axhline(y=1200, color="red", linestyle="--", alpha=0.5, linewidth=0.8)
        ax.set_ylim(y_min, y_max)

        ticks = sorted(set([y_min, 400, 500, 600, 700, 800, 900, 1000, 1200, 1500, 2000]) & set(range(y_min, y_max + 1)))
        ticks = [t for t in ticks if y_min <= t <= y_max]
        ax.yaxis.set_major_locator(FixedLocator(ticks))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v)}"))

    def _draw_temp_chart(self, ax):
        """Draw Temperature chart on given axes."""
        ax.set_title("Temperature (°C)", fontsize=12, fontweight="bold")
        ax.set_ylabel("°C")
        has_both = len(self.s3_temp_values) > 0 and len(self.c3_temp_values) > 0
        if has_both:
            ts_s3 = list(self.s3_timestamps)[-len(self.s3_temp_values):]
            ax.plot(ts_s3, list(self.s3_temp_values), self.chart_colors["temp_s3"],
                    linewidth=1.8, marker=".", markersize=4, label=self._get_s3_display_name())
            ts_c3 = list(self.c3_timestamps)[-len(self.c3_temp_values):]
            ax.plot(ts_c3, list(self.c3_temp_values), self.chart_colors["temp_c3"],
                    linewidth=1.8, marker="s", markersize=4, label=self._get_c3_display_name())
            ax.legend(loc="upper left", fontsize=9)
        elif self.temp_values:
            ts = list(self.timestamps)[-len(self.temp_values):]
            ax.plot(ts, list(self.temp_values), self.chart_colors["temp_single"],
                    linewidth=1.8, marker=".", markersize=4)

    def _draw_hum_chart(self, ax):
        """Draw Humidity chart on given axes."""
        ax.set_title("Humidity (%)", fontsize=12, fontweight="bold")
        ax.set_ylabel("%")
        has_both = len(self.s3_hum_values) > 0 and len(self.c3_hum_values) > 0
        if has_both:
            ts_s3 = list(self.s3_timestamps)[-len(self.s3_hum_values):]
            ax.plot(ts_s3, list(self.s3_hum_values), self.chart_colors["hum_s3"],
                    linewidth=1.8, marker=".", markersize=4, label=self._get_s3_display_name())
            ts_c3 = list(self.c3_timestamps)[-len(self.c3_hum_values):]
            ax.plot(ts_c3, list(self.c3_hum_values), self.chart_colors["hum_c3"],
                    linewidth=1.8, marker="s", markersize=4, label=self._get_c3_display_name())
            ax.legend(loc="upper left", fontsize=9)
        elif self.hum_values:
            ts = list(self.timestamps)[-len(self.hum_values):]
            ax.plot(ts, list(self.hum_values), self.chart_colors["hum_single"],
                    linewidth=1.8, marker=".", markersize=4)

    def _draw_pressure_chart(self, ax):
        """Draw Pressure chart on given axes."""
        ax.set_title("Pressure (hPa)", fontsize=12, fontweight="bold")
        ax.set_ylabel("hPa")
        ax.ticklabel_format(axis='y', useOffset=False, style='plain')
        has_both = len(self.s3_pressure_values) > 0 and len(self.c3_pressure_values) > 0
        if has_both:
            ts_s3 = list(self.s3_pressure_timestamps)[-len(self.s3_pressure_values):]
            ax.plot(ts_s3, list(self.s3_pressure_values), self.chart_colors["press_s3"],
                    linewidth=1.8, marker=".", markersize=4, label=self._get_s3_display_name())
            ts_c3 = list(self.c3_pressure_timestamps)[-len(self.c3_pressure_values):]
            ax.plot(ts_c3, list(self.c3_pressure_values), self.chart_colors["press_c3"],
                    linewidth=1.8, marker="s", markersize=4, label=self._get_c3_display_name())
            ax.legend(loc="upper left", fontsize=9)
        elif self.pressure_values:
            ts = list(self.timestamps)[-len(self.pressure_values):]
            ax.plot(ts, list(self.pressure_values), self.chart_colors["press_single"],
                    linewidth=1.8, marker=".", markersize=4)
        if self.pressure_values:
            vals = list(self.pressure_values)
            p_min, p_max = min(vals), max(vals)
            if p_max - p_min <= 0.1:
                ax.set_ylim(980, 1020)
        else:
            ax.set_ylim(980, 1020)

    def _draw_altitude_chart(self, ax):
        """Draw Altitude chart on given axes."""
        ax.set_title("Altitude (m)", fontsize=12, fontweight="bold")
        ax.set_ylabel("m")
        has_both = len(self.s3_altitude_values) > 0 and len(self.c3_altitude_values) > 0
        if has_both:
            ts_s3 = list(self.s3_pressure_timestamps)[-len(self.s3_altitude_values):]
            ax.plot(ts_s3, list(self.s3_altitude_values), self.chart_colors["alt_s3"],
                    linewidth=1.8, marker=".", markersize=4, label=self._get_s3_display_name())
            ts_c3 = list(self.c3_pressure_timestamps)[-len(self.c3_altitude_values):]
            ax.plot(ts_c3, list(self.c3_altitude_values), self.chart_colors["alt_c3"],
                    linewidth=1.8, marker="s", markersize=4, label=self._get_c3_display_name())
            ax.legend(loc="upper left", fontsize=9)
        elif self.altitude_values:
            ts = list(self.timestamps)[-len(self.altitude_values):]
            ax.plot(ts, list(self.altitude_values), self.chart_colors["alt_single"],
                    linewidth=1.8, marker=".", markersize=4)

    def _apply_theme(self, theme):
        """Apply day/night theme to all UI elements."""
        self.current_theme = theme
        bg = theme["bg"]
        fg = theme["fg"]

        # Use 'clam' theme on Windows so button colors are actually applied
        style = ttk.Style()
        if style.theme_use() in ("vista", "winnative", "xpnative"):
            style.theme_use("clam")

        # Root and all frames
        self.root.configure(bg=bg)
        style.configure(".", background=bg, foreground=fg)
        style.configure("TFrame", background=bg)
        style.configure("TLabelframe", background=bg, foreground=fg)
        style.configure("TLabelframe.Label", background=bg, foreground=fg)
        style.configure("TLabel", background=bg, foreground=theme["label_fg"])
        style.configure("TButton", background=theme["btn_bg"], foreground=theme["btn_fg"])
        style.map("TButton",
                  background=[("active", theme["btn_active_bg"]), ("!active", theme["btn_bg"])],
                  foreground=[("active", theme["btn_fg"]), ("!active", theme["btn_fg"])])
        style.configure("TCheckbutton", background=bg, foreground=fg)
        style.configure("TPanedwindow", background=bg)
        style.configure("TSeparator", background=theme["chart_grid"])
        style.configure("TCombobox", fieldbackground=theme["entry_bg"], foreground=theme["entry_fg"],
                        background=theme["btn_bg"])
        style.map("TCombobox", fieldbackground=[("readonly", theme["entry_bg"])])
        style.configure("TEntry", fieldbackground=theme["entry_bg"], foreground=theme["entry_fg"])

        # Treeview (history)
        style.configure("Treeview", background=theme["tree_bg"], foreground=theme["tree_fg"],
                        fieldbackground=theme["tree_bg"])
        style.configure("Treeview.Heading", background=theme["tree_heading_bg"],
                        foreground=theme["tree_heading_fg"])
        style.map("Treeview",
                  background=[("selected", theme["tree_selected_bg"])],
                  foreground=[("selected", theme["tree_fg"])])
        self.history_tree.tag_configure("S3", background=theme["tree_s3_bg"],
                                        foreground=theme["tree_s3_fg"])
        self.history_tree.tag_configure("C3", background=theme["tree_c3_bg"],
                                        foreground=theme["tree_c3_fg"])

        # Log text widget
        self.log_text.configure(bg=theme["log_bg"], fg=theme["log_fg"])

        # BLE LED outline
        self.ble_led.configure(bg=bg)
        self.ble_led.itemconfig(self.ble_led_oval, outline=theme["led_outline"])
        # Alarm LED outlines
        for led, oval in [(self.alarm_temp_led, self.alarm_temp_oval),
                          (self.alarm_co2_led, self.alarm_co2_oval),
                          (self.alarm_hum_led, self.alarm_hum_oval)]:
            led.configure(bg=bg)
            led.itemconfig(oval, outline=theme["led_outline"])

        # Chart colors
        self.fig.set_facecolor(theme["chart_face"])
        for ax in self.fig.get_axes():
            ax.set_facecolor(theme["chart_axes_bg"])
            ax.tick_params(colors=theme["chart_text"])
            ax.xaxis.label.set_color(theme["chart_text"])
            ax.yaxis.label.set_color(theme["chart_text"])
            ax.title.set_color(theme["chart_text"])
            for spine in ax.spines.values():
                spine.set_color(theme["chart_grid"])
        self.canvas.draw_idle()
        self.new_data_flag = True

    def _build_history_frame(self, parent):
        columns = ("Time", "Source", "Device", "Temp", "Humidity", "CO2", "Pressure", "Altitude", "RSSI")
        self.history_tree = ttk.Treeview(parent, columns=columns, show="headings", height=5)
        for col in columns:
            self.history_tree.heading(col, text=col)
            self.history_tree.column(col, width=85, anchor=tk.CENTER)
        self.history_tree.column("Time", width=140)
        self.history_tree.column("Source", width=60)
        self.history_tree.column("Device", width=50)

        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=self.history_tree.yview)
        self.history_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.history_tree.pack(fill=tk.BOTH, expand=True)

        # Row colors by device type
        self.history_tree.tag_configure("S3", background=self.current_theme["tree_s3_bg"],
                                        foreground=self.current_theme["tree_s3_fg"])
        self.history_tree.tag_configure("C3", background=self.current_theme["tree_c3_bg"],
                                        foreground=self.current_theme["tree_c3_fg"])

    # =========================================================================
    # Serial Port
    # =========================================================================
    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [p.device for p in ports]
        self.port_combo["values"] = port_list
        if port_list:
            self.port_combo.current(0)

    def _toggle_serial(self):
        if self.serial_connected:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("Serial", "No port selected.")
            return
        try:
            self.serial_port = serial.Serial(port, SERIAL_BAUD, timeout=1)
            self.serial_connected = True
            self.connect_btn.config(text="Disconnect")
            self._log(f"Connected to {port} @ {SERIAL_BAUD}")
            self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.serial_thread.start()
        except Exception as e:
            messagebox.showerror("Serial Error", str(e))

    def _disconnect_serial(self):
        self.serial_connected = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.connect_btn.config(text="Connect")
        self._log("Serial disconnected")

    def _serial_reader(self):
        while self.serial_connected and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.root.after(0, self._process_serial_line, line)
                else:
                    time.sleep(0.05)
            except Exception as e:
                if self.serial_connected:
                    self.root.after(0, self._log, f"Serial error: {e}")
                break

    def _process_serial_line(self, line):
        self._log(line)
        if line.startswith("{"):
            try:
                data = json.loads(line)
                self._parse_serial_json(data)
            except (json.JSONDecodeError, ValueError, TypeError) as e:
                pass  # not all lines are JSON

    def _parse_serial_json(self, data):
        """Parse JSON from either S3 (CO2) or C3 (ENV) format."""
        co2 = data.get("co2_ppm")
        pressure = None
        altitude = None

        # S3 format: display_temp, display_hum, co2_ppm
        temp_str = data.get("display_temp") or data.get("temp_scd40")
        hum_str = data.get("display_hum") or data.get("humidity")
        temp = float(temp_str) if temp_str else None
        hum = float(hum_str) if hum_str else None

        # Pressure: S3 BMP580 or C3 format
        press_str = data.get("Pressure_BMP580") or data.get("pressure") or data.get("pressure_hpa")
        if press_str is not None:
            try:
                pressure = float(press_str)
                altitude = pressure_to_altitude(pressure)
            except (ValueError, TypeError):
                pass

        alt_str = data.get("altitude") or data.get("alt")
        if alt_str is not None and altitude is None:
            try:
                altitude = float(alt_str)
            except (ValueError, TypeError):
                pass

        device = "S3" if co2 is not None else "C3"

        if co2 is not None or temp is not None or pressure is not None:
            self._add_data(SensorData(
                source="Serial",
                device=device,
                temp=temp,
                hum=hum,
                co2=int(co2) if co2 is not None else None,
                pressure=pressure,
                altitude=altitude
            ))

    def _send_serial_cmd(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return
        if self.serial_connected and self.serial_port:
            try:
                self.serial_port.write((cmd + "\n").encode("utf-8"))
                self._log(f">> {cmd}")
                self.cmd_entry.delete(0, tk.END)
            except Exception as e:
                self._log(f"Send error: {e}")
        else:
            messagebox.showwarning("Serial", "Not connected.")

    # =========================================================================
    # BLE Scanner
    # =========================================================================
    def _start_ble_scanner(self):
        if not BLE_AVAILABLE:
            self.ble_status.set("No bleak")
            self._log("BLE: bleak not installed. pip install bleak")
            return
        self.ble_running = True
        self.ble_thread = threading.Thread(target=self._ble_thread_func, daemon=True)
        self.ble_thread.start()

    def _ble_thread_func(self):
        self.ble_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ble_loop)
        try:
            self.ble_loop.run_until_complete(self._ble_scan_loop())
        except Exception as e:
            self.root.after(0, self._log, f"BLE thread error: {e}")
        finally:
            self.ble_loop.close()

    async def _ble_scan_loop(self):
        self.root.after(0, lambda: self.ble_status.set("Scanning"))
        while self.ble_running:
            try:
                devices = await BleakScanner.discover(timeout=BLE_SCAN_INTERVAL_S, return_adv=True)
                found = False
                for addr, (device, adv_data) in devices.items():
                    if self._is_target_device(device, adv_data):
                        found = True
                        self._process_ble_device(device, adv_data)

                if found:
                    self.root.after(0, self._set_ble_led, "green")
                    self.root.after(0, lambda: self.ble_status.set("Online"))
                else:
                    self.root.after(0, self._set_ble_led, "gray")
                    self.root.after(0, lambda: self.ble_status.set("Scanning"))
                    for info in self.ble_device_status.values():
                        info["connected"] = False

            except Exception as e:
                self.root.after(0, self._log, f"BLE error: {e}")
                self.root.after(0, self._set_ble_led, "red")
                await asyncio.sleep(5)
                continue
            await asyncio.sleep(0.5)

    def _is_target_device(self, device, adv_data):
        name = device.name or adv_data.local_name or ""
        for target_name in BLE_DEVICE_NAMES:
            if target_name in name:
                return True
        if adv_data.manufacturer_data:
            if BLE_COMPANY_ID in adv_data.manufacturer_data:
                return True
        return False

    def _process_ble_device(self, device, adv_data):
        rssi = adv_data.rssi if hasattr(adv_data, "rssi") else None
        temp = None
        hum = None
        co2 = None
        pressure = None
        altitude = None
        device_type = "?"
        source = "BLE"

        # Decode manufacturer data
        if adv_data.manufacturer_data and BLE_COMPANY_ID in adv_data.manufacturer_data:
            mfr_data = adv_data.manufacturer_data[BLE_COMPANY_ID]
            result = self._decode_manufacturer_data(mfr_data)
            if result:
                temp = result.get("temp")
                hum = result.get("hum")
                co2 = result.get("co2")
                pressure = result.get("pressure")
                altitude = result.get("altitude")
                device_type = result.get("device", "?")

        # Decode scan response service data
        if adv_data.service_data:
            for uuid, svc_data in adv_data.service_data.items():
                if "fc90" in str(uuid).lower():
                    result = self._decode_scan_response_text(svc_data)
                    if result:
                        if result.get("temp") is not None:
                            temp = result["temp"]
                        if result.get("hum") is not None:
                            hum = result["hum"]
                        if result.get("co2") is not None:
                            co2 = result["co2"]
                        if result.get("pressure") is not None:
                            pressure = result["pressure"]
                            altitude = pressure_to_altitude(pressure)

        if co2 is not None or temp is not None or pressure is not None:
            self.root.after(0, self._add_data, SensorData(
                source=source,
                device=device_type,
                temp=temp,
                hum=hum,
                co2=co2,
                pressure=pressure,
                altitude=altitude,
                rssi=rssi
            ))

    def _decode_manufacturer_data(self, data):
        """
        Decode payload after company ID (bleak strips 2-byte company ID):
          v0x01 (8 bytes): [ver][temp_L][temp_H][hum_L][hum_H][press_L][press_H][XOR]  — C3
          v0x02 (8 bytes): [ver][temp_L][temp_H][hum_L][hum_H][CO2_L][CO2_H][XOR]      — S3 (no pressure)
          v0x03 (10 bytes): [ver][temp_L][temp_H][hum_L][hum_H][CO2_L][CO2_H][press_L][press_H][XOR] — S3 + BMP580
        """
        if len(data) < 8:
            return None

        version = data[0]

        if version == 0x03:
            if len(data) < 10:
                return None
            xor_check = 0
            for i in range(9):
                xor_check ^= data[i]
            if xor_check != data[9]:
                return None

            temp_raw = struct.unpack_from("<h", data, 1)[0]
            hum_raw = struct.unpack_from("<h", data, 3)[0]
            co2_raw = struct.unpack_from("<H", data, 5)[0]
            press_raw = struct.unpack_from("<H", data, 7)[0]

            result = {
                "temp": temp_raw / 100.0 if temp_raw != 0x7FFF else None,
                "hum": hum_raw / 100.0 if hum_raw != 0x7FFF else None,
                "co2": co2_raw if co2_raw != 0xFFFF else None,
                "device": "S3",
            }
            if press_raw != 0xFFFF:
                pressure_hpa = (press_raw + 8000) / 10.0
                result["pressure"] = pressure_hpa
                result["altitude"] = pressure_to_altitude(pressure_hpa)
            return result

        # v0x01 / v0x02: 8-byte format
        xor_check = 0
        for i in range(7):
            xor_check ^= data[i]
        if xor_check != data[7]:
            return None

        temp_raw = struct.unpack_from("<h", data, 1)[0]
        hum_raw = struct.unpack_from("<h", data, 3)[0]
        field_raw = struct.unpack_from("<H", data, 5)[0]

        result = {
            "temp": temp_raw / 100.0 if temp_raw != 0x7FFF else None,
            "hum": hum_raw / 100.0 if hum_raw != 0x7FFF else None,
        }

        if version == 0x02:
            result["co2"] = field_raw if field_raw != 0xFFFF else None
            result["device"] = "S3"
        elif version == 0x01:
            if field_raw != 0xFFFF:
                pressure_hpa = (field_raw + 8000) / 10.0
                result["pressure"] = pressure_hpa
                result["altitude"] = pressure_to_altitude(pressure_hpa)
            result["device"] = "C3"

        return result

    def _decode_scan_response_text(self, data):
        """Decode 'T23.5 H65.2 C456' or 'T23.5 H65.2 P1013.2'"""
        try:
            text = data.decode("utf-8", errors="replace").strip()
            return self._parse_text_format(text)
        except Exception:
            return None

    def _parse_text_format(self, text):
        result = {}
        parts = text.split()
        for part in parts:
            try:
                if part.startswith("T") and len(part) > 1:
                    val = part[1:]
                    if val != "--":
                        result["temp"] = float(val)
                elif part.startswith("H") and len(part) > 1:
                    val = part[1:]
                    if val != "--":
                        result["hum"] = float(val)
                elif part.startswith("C") and len(part) > 1:
                    val = part[1:]
                    if val != "--":
                        result["co2"] = int(float(val))
                elif part.startswith("P") and len(part) > 1:
                    val = part[1:]
                    if val != "--":
                        result["pressure"] = float(val)
            except (ValueError, IndexError):
                continue
        return result if result else None

    # =========================================================================
    # Data Management
    # =========================================================================
    def _add_data(self, sd):
        self.data_history.append(sd)
        self.timestamps.append(sd.timestamp)

        if sd.co2 is not None:
            self.co2_values.append(sd.co2)
            self.current_co2.set(str(sd.co2))
            self._update_co2_level(sd.co2)
        if sd.temp is not None:
            self.temp_values.append(sd.temp)
            self.current_temp.set(f"{sd.temp:.1f}")
        if sd.hum is not None:
            self.hum_values.append(sd.hum)
            self.current_hum.set(f"{sd.hum:.1f}")
        if sd.pressure is not None:
            self.pressure_values.append(sd.pressure)
            self.current_pressure.set(f"{sd.pressure:.1f}")
        if sd.altitude is not None:
            self.altitude_values.append(sd.altitude)
            self.current_altitude.set(f"{sd.altitude:.1f}")

        # Per-device temp/hum/pressure/altitude storage for dual-device chart
        if sd.device == "S3":
            if sd.temp is not None:
                self.s3_timestamps.append(sd.timestamp)
                self.s3_temp_values.append(sd.temp)
            if sd.hum is not None:
                if not self.s3_timestamps or self.s3_timestamps[-1] != sd.timestamp:
                    self.s3_timestamps.append(sd.timestamp)
                self.s3_hum_values.append(sd.hum)
            if sd.pressure is not None:
                self.s3_pressure_timestamps.append(sd.timestamp)
                self.s3_pressure_values.append(sd.pressure)
            if sd.altitude is not None:
                if not self.s3_pressure_timestamps or self.s3_pressure_timestamps[-1] != sd.timestamp:
                    self.s3_pressure_timestamps.append(sd.timestamp)
                self.s3_altitude_values.append(sd.altitude)
        elif sd.device == "C3":
            if sd.temp is not None:
                self.c3_timestamps.append(sd.timestamp)
                self.c3_temp_values.append(sd.temp)
            if sd.hum is not None:
                if not self.c3_timestamps or self.c3_timestamps[-1] != sd.timestamp:
                    self.c3_timestamps.append(sd.timestamp)
                self.c3_hum_values.append(sd.hum)
            if sd.pressure is not None:
                self.c3_pressure_timestamps.append(sd.timestamp)
                self.c3_pressure_values.append(sd.pressure)
            if sd.altitude is not None:
                if not self.c3_pressure_timestamps or self.c3_pressure_timestamps[-1] != sd.timestamp:
                    self.c3_pressure_timestamps.append(sd.timestamp)
                self.c3_altitude_values.append(sd.altitude)

        self._update_statistics()
        self._check_alarms(sd)

        self.device_label.set(sd.device)

        # Track BLE device status for console output
        self.total_data_received += 1
        device_name = f"{sd.device}({sd.source})"
        if device_name not in self.ble_device_status:
            self.ble_device_status[device_name] = {"connected": True, "last_data_time": None, "data_count": 0}
        self.ble_device_status[device_name]["connected"] = True
        self.ble_device_status[device_name]["last_data_time"] = sd.timestamp
        self.ble_device_status[device_name]["data_count"] += 1

        # History table
        time_str = sd.timestamp.strftime("%Y-%m-%d %H:%M:%S")
        t_str = f"{sd.temp:.1f}" if sd.temp is not None else "--"
        h_str = f"{sd.hum:.1f}" if sd.hum is not None else "--"
        c_str = str(sd.co2) if sd.co2 is not None else "--"
        p_str = f"{sd.pressure:.1f}" if sd.pressure is not None else "--"
        a_str = f"{sd.altitude:.1f}" if sd.altitude is not None else "--"
        r_str = str(sd.rssi) if sd.rssi is not None else "--"

        row_tag = sd.device if sd.device in ("S3", "C3") else "C3"
        self.history_tree.insert("", 0, values=(
            time_str, sd.source, sd.device, t_str, h_str, c_str, p_str, a_str, r_str
        ), tags=(row_tag,))
        children = self.history_tree.get_children()
        if len(children) > MAX_DATA_POINTS:
            self.history_tree.delete(children[-1])

        self.lbl_records.config(text=f"Records: {len(self.data_history)}")
        self.new_data_flag = True

    def _update_co2_level(self, co2):
        if co2 < 800:
            self.current_co2_level.set("Good")
            self.co2_level_label.configure(foreground="green")
        elif co2 <= 1200:
            self.current_co2_level.set("Fair")
            self.co2_level_label.configure(foreground="orange")
        else:
            self.current_co2_level.set("Poor")
            self.co2_level_label.configure(foreground="red")

    def _update_statistics(self):
        if self.stats_device == "S3":
            temp_src = self.s3_temp_values
            hum_src = self.s3_hum_values
            pressure_src = self.s3_pressure_values
            altitude_src = self.s3_altitude_values
        elif self.stats_device == "C3":
            temp_src = self.c3_temp_values
            hum_src = self.c3_hum_values
            pressure_src = self.c3_pressure_values
            altitude_src = self.c3_altitude_values
        else:
            temp_src = self.temp_values
            hum_src = self.hum_values
            pressure_src = self.pressure_values
            altitude_src = self.altitude_values

        if temp_src:
            self.stat_temp_avg.set(f"{sum(temp_src)/len(temp_src):.1f}")
            self.stat_temp_max.set(f"{max(temp_src):.1f}")
            self.stat_temp_min.set(f"{min(temp_src):.1f}")
        else:
            self.stat_temp_avg.set("--.-")
            self.stat_temp_max.set("--.-")
            self.stat_temp_min.set("--.-")
        if hum_src:
            self.stat_hum_avg.set(f"{sum(hum_src)/len(hum_src):.1f}")
            self.stat_hum_max.set(f"{max(hum_src):.1f}")
            self.stat_hum_min.set(f"{min(hum_src):.1f}")
        else:
            self.stat_hum_avg.set("--.-")
            self.stat_hum_max.set("--.-")
            self.stat_hum_min.set("--.-")
        if self.co2_values:
            self.stat_co2_avg.set(f"{sum(self.co2_values)/len(self.co2_values):.0f}")
            self.stat_co2_max.set(f"{max(self.co2_values)}")
            self.stat_co2_min.set(f"{min(self.co2_values)}")
        else:
            self.stat_co2_avg.set("---")
            self.stat_co2_max.set("---")
            self.stat_co2_min.set("---")
        if pressure_src:
            self.stat_pressure_avg.set(f"{sum(pressure_src)/len(pressure_src):.1f}")
            self.stat_pressure_max.set(f"{max(pressure_src):.1f}")
            self.stat_pressure_min.set(f"{min(pressure_src):.1f}")
        else:
            self.stat_pressure_avg.set("----.-")
            self.stat_pressure_max.set("----.-")
            self.stat_pressure_min.set("----.-")
        if altitude_src:
            self.stat_alt_min.set(f"{min(altitude_src):.1f}")
            self.stat_alt_max.set(f"{max(altitude_src):.1f}")
        else:
            self.stat_alt_min.set("---.-")
            self.stat_alt_max.set("---.-")

    # =========================================================================
    # Charts
    # =========================================================================
    def _schedule_chart_update(self):
        now = time.time()
        if self.new_data_flag and (now - self.last_chart_update) >= (CHART_UPDATE_INTERVAL_MS / 1000.0):
            self._update_charts()
            self.last_chart_update = now
            self.new_data_flag = False
        self.root.after(500, self._schedule_chart_update)

    @staticmethod
    def _co2_value_to_color(val):
        """Map CO2 ppm to gradient color: dark green -> orange -> purple-red."""
        # 400-800: dark green (0,100,0) -> orange (255,165,0)
        # 800-1500: orange (255,165,0) -> purple-red (180,0,100)
        if val <= 400:
            return "#006400"
        elif val <= 800:
            t = (val - 400) / 400.0
            r = int(0 + t * 255)
            g = int(100 + t * 65)
            b = int(0)
            return f"#{r:02x}{g:02x}{b:02x}"
        elif val <= 1500:
            t = (val - 800) / 700.0
            r = int(255 - t * 75)
            g = int(165 - t * 165)
            b = int(0 + t * 100)
            return f"#{r:02x}{g:02x}{b:02x}"
        else:
            return "#B40064"

    def _update_charts(self):
        try:
            # CO2 with adaptive non-linear Y-axis
            if self.ax_co2 is not None:
                self.ax_co2.clear()
                if self.co2_values:
                    ts = list(self.timestamps)[-len(self.co2_values):]
                    vals = list(self.co2_values)
                    current_max = max(vals)
                    fwd, inv, y_min, y_max, focus_lo, focus_hi = _make_co2_scale(current_max)

                    self.ax_co2.set_yscale("function", functions=(fwd, inv))
                    self.ax_co2.set_title(f"CO2 (ppm) [focus: {focus_lo}-{focus_hi}]", fontsize=10, fontweight="bold")
                    self.ax_co2.set_ylabel("ppm")
                    self.ax_co2.grid(True, alpha=0.3)

                    self.ax_co2.axhspan(y_min, 800, alpha=0.08, color="green")
                    self.ax_co2.axhspan(800, min(1200, y_max), alpha=0.08, color="yellow")
                    if y_max > 1200:
                        self.ax_co2.axhspan(1200, y_max, alpha=0.08, color="red")

                    for i in range(len(vals) - 1):
                        seg_val = (vals[i] + vals[i + 1]) / 2.0
                        color = self._co2_value_to_color(seg_val)
                        self.ax_co2.plot(ts[i:i+2], vals[i:i+2], color=color, linewidth=2, solid_capstyle="round")
                    for i, v in enumerate(vals):
                        self.ax_co2.plot(ts[i], v, ".", color=self._co2_value_to_color(v), markersize=4)

                    self.ax_co2.axhline(y=800, color="orange", linestyle="--", alpha=0.5, linewidth=0.8)
                    if y_max >= 1200:
                        self.ax_co2.axhline(y=1200, color="red", linestyle="--", alpha=0.5, linewidth=0.8)
                    self.ax_co2.set_ylim(y_min, y_max)

                    ticks = sorted(set([y_min, 400, 500, 600, 700, 800, 900, 1000, 1200, 1500, 2000]) & set(range(y_min, y_max + 1)))
                    ticks = [t for t in ticks if y_min <= t <= y_max]
                    self.ax_co2.yaxis.set_major_locator(FixedLocator(ticks))
                    self.ax_co2.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v)}"))
                else:
                    fwd, inv, y_min, y_max, _, _ = _make_co2_scale(400)
                    self.ax_co2.set_yscale("function", functions=(fwd, inv))
                    self.ax_co2.set_title("CO2 (ppm)", fontsize=10, fontweight="bold")
                    self.ax_co2.set_ylabel("ppm")
                    self.ax_co2.grid(True, alpha=0.3)
                    self.ax_co2.set_ylim(y_min, y_max)
                    self.ax_co2.yaxis.set_major_locator(FixedLocator([300, 400, 500, 600, 700, 800, 1000, 1200]))
                    self.ax_co2.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v)}"))

            # Temperature
            self.ax_temp.clear()
            self.ax_temp.set_title("Temperature (C)", fontsize=9, fontweight="bold")
            self.ax_temp.set_ylabel("C")
            self.ax_temp.grid(True, alpha=0.3)
            has_both_devices = len(self.s3_temp_values) > 0 and len(self.c3_temp_values) > 0
            if has_both_devices:
                ts_s3 = list(self.s3_timestamps)[-len(self.s3_temp_values):]
                self.ax_temp.plot(ts_s3, list(self.s3_temp_values), self.chart_colors["temp_s3"], linewidth=1.5, marker=".", markersize=3, label=self._get_s3_display_name())
                ts_c3 = list(self.c3_timestamps)[-len(self.c3_temp_values):]
                self.ax_temp.plot(ts_c3, list(self.c3_temp_values), self.chart_colors["temp_c3"], linewidth=1.5, marker="s", markersize=3, label=self._get_c3_display_name())
                self.ax_temp.legend(loc="upper left", fontsize=7)
            elif self.temp_values:
                ts = list(self.timestamps)[-len(self.temp_values):]
                self.ax_temp.plot(ts, list(self.temp_values), self.chart_colors["temp_single"], linewidth=1.5, marker=".", markersize=3)

            # Humidity
            self.ax_hum.clear()
            self.ax_hum.set_title("Humidity (%)", fontsize=9, fontweight="bold")
            self.ax_hum.set_ylabel("%")
            self.ax_hum.grid(True, alpha=0.3)
            has_both_hum = len(self.s3_hum_values) > 0 and len(self.c3_hum_values) > 0
            if has_both_hum:
                ts_s3 = list(self.s3_timestamps)[-len(self.s3_hum_values):]
                self.ax_hum.plot(ts_s3, list(self.s3_hum_values), self.chart_colors["hum_s3"], linewidth=1.5, marker=".", markersize=3, label=self._get_s3_display_name())
                ts_c3 = list(self.c3_timestamps)[-len(self.c3_hum_values):]
                self.ax_hum.plot(ts_c3, list(self.c3_hum_values), self.chart_colors["hum_c3"], linewidth=1.5, marker="s", markersize=3, label=self._get_c3_display_name())
                self.ax_hum.legend(loc="upper left", fontsize=7)
            elif self.hum_values:
                ts = list(self.timestamps)[-len(self.hum_values):]
                self.ax_hum.plot(ts, list(self.hum_values), self.chart_colors["hum_single"], linewidth=1.5, marker=".", markersize=3)

            # Pressure (per-device colors, if shown)
            if self.ax_press is not None:
                self.ax_press.clear()
                self.ax_press.set_title("Pressure (hPa)", fontsize=9, fontweight="bold")
                self.ax_press.set_ylabel("hPa")
                self.ax_press.grid(True, alpha=0.3)
                self.ax_press.ticklabel_format(axis='y', useOffset=False, style='plain')
                has_both_press = len(self.s3_pressure_values) > 0 and len(self.c3_pressure_values) > 0
                if has_both_press:
                    ts_s3 = list(self.s3_pressure_timestamps)[-len(self.s3_pressure_values):]
                    self.ax_press.plot(ts_s3, list(self.s3_pressure_values), self.chart_colors["press_s3"], linewidth=1.5, marker=".", markersize=3, label=self._get_s3_display_name())
                    ts_c3 = list(self.c3_pressure_timestamps)[-len(self.c3_pressure_values):]
                    self.ax_press.plot(ts_c3, list(self.c3_pressure_values), self.chart_colors["press_c3"], linewidth=1.5, marker="s", markersize=3, label=self._get_c3_display_name())
                    self.ax_press.legend(loc="upper left", fontsize=7)
                elif self.pressure_values:
                    ts = list(self.timestamps)[-len(self.pressure_values):]
                    vals = list(self.pressure_values)
                    self.ax_press.plot(ts, vals, self.chart_colors["press_single"], linewidth=1.5, marker=".", markersize=3)
                if self.pressure_values:
                    vals = list(self.pressure_values)
                    p_min, p_max = min(vals), max(vals)
                    if p_max - p_min <= 0.1:
                        self.ax_press.set_ylim(980, 1020)
                else:
                    self.ax_press.set_ylim(980, 1020)

            # Altitude (per-device colors, if shown)
            if self.ax_alt is not None:
                self.ax_alt.clear()
                self.ax_alt.set_title("Altitude (m)", fontsize=9, fontweight="bold")
                self.ax_alt.set_ylabel("m")
                self.ax_alt.grid(True, alpha=0.3)
                has_both_alt = len(self.s3_altitude_values) > 0 and len(self.c3_altitude_values) > 0
                if has_both_alt:
                    ts_s3 = list(self.s3_pressure_timestamps)[-len(self.s3_altitude_values):]
                    self.ax_alt.plot(ts_s3, list(self.s3_altitude_values), self.chart_colors["alt_s3"], linewidth=1.5, marker=".", markersize=3, label=self._get_s3_display_name())
                    ts_c3 = list(self.c3_pressure_timestamps)[-len(self.c3_altitude_values):]
                    self.ax_alt.plot(ts_c3, list(self.c3_altitude_values), self.chart_colors["alt_c3"], linewidth=1.5, marker="s", markersize=3, label=self._get_c3_display_name())
                    self.ax_alt.legend(loc="upper left", fontsize=7)
                elif self.altitude_values:
                    ts = list(self.timestamps)[-len(self.altitude_values):]
                    self.ax_alt.plot(ts, list(self.altitude_values), self.chart_colors["alt_single"], linewidth=1.5, marker=".", markersize=3)

            # Format x-axis
            axes = [self.ax_temp, self.ax_hum]
            if self.ax_co2 is not None:
                axes.append(self.ax_co2)
            if self.ax_press is not None:
                axes.append(self.ax_press)
            if self.ax_alt is not None:
                axes.append(self.ax_alt)
            theme = self.current_theme
            self.fig.set_facecolor(theme["chart_face"])
            for ax in axes:
                ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
                ax.tick_params(axis="x", rotation=30, labelsize=7, colors=theme["chart_text"])
                ax.tick_params(axis="y", labelsize=8, colors=theme["chart_text"])
                ax.set_facecolor(theme["chart_axes_bg"])
                ax.title.set_color(theme["chart_text"])
                ax.xaxis.label.set_color(theme["chart_text"])
                ax.yaxis.label.set_color(theme["chart_text"])
                ax.grid(True, alpha=0.3, color=theme["chart_grid"])
                for spine in ax.spines.values():
                    spine.set_color(theme["chart_grid"])

            self.fig.tight_layout(pad=1.5)
            self.canvas.draw_idle()

        except Exception as e:
            self._log(f"Chart error: {e}")

    # =========================================================================
    # Utilities
    # =========================================================================
    def _log(self, message):
        self.log_text.configure(state=tk.NORMAL)
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{ts}] {message}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_ble_led(self, color):
        self.ble_led.itemconfig(self.ble_led_oval, fill=color)

    def _check_alarms(self, sd):
        if not self.alarm_enabled:
            self.alarm_temp_led.itemconfig(self.alarm_temp_oval, fill="gray")
            self.alarm_co2_led.itemconfig(self.alarm_co2_oval, fill="gray")
            self.alarm_hum_led.itemconfig(self.alarm_hum_oval, fill="gray")
            return
        alarms = []
        if sd.temp is not None:
            if sd.temp > self.alarm_temp_high:
                self.alarm_temp_led.itemconfig(self.alarm_temp_oval, fill="#F44336")
                alarms.append("Temp_High")
            elif sd.temp < self.alarm_temp_low:
                self.alarm_temp_led.itemconfig(self.alarm_temp_oval, fill="#1565C0")
                alarms.append("Temp_Low")
            else:
                self.alarm_temp_led.itemconfig(self.alarm_temp_oval, fill="gray")
        if sd.co2 is not None:
            if sd.co2 > self.alarm_co2_high:
                self.alarm_co2_led.itemconfig(self.alarm_co2_oval, fill="#F44336")
                alarms.append("CO2_High")
            elif sd.co2 < self.alarm_co2_low:
                self.alarm_co2_led.itemconfig(self.alarm_co2_oval, fill="#1565C0")
                alarms.append("CO2_Low")
            else:
                self.alarm_co2_led.itemconfig(self.alarm_co2_oval, fill="gray")
        if sd.hum is not None:
            if sd.hum > self.alarm_hum_high:
                self.alarm_hum_led.itemconfig(self.alarm_hum_oval, fill="#F44336")
                alarms.append("Hum_High")
            elif sd.hum < self.alarm_hum_low:
                self.alarm_hum_led.itemconfig(self.alarm_hum_oval, fill="#1565C0")
                alarms.append("Hum_Low")
            else:
                self.alarm_hum_led.itemconfig(self.alarm_hum_oval, fill="gray")
        sd.alarm = ",".join(alarms)

    def _export_csv(self):
        if not self.data_history:
            messagebox.showinfo("Export", "No data to export.")
            return

        devices = set(d.device for d in self.data_history if d.device)
        has_multi_device = len(devices) >= 2

        if has_multi_device:
            try:
                import openpyxl
            except ImportError:
                openpyxl = None

            if openpyxl:
                self._export_xlsx_multi_sheet(devices)
            else:
                self._export_csv_multi_file(devices)
        else:
            self._export_csv_single()

    def _export_csv_single(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        if not filename:
            return
        try:
            with open(filename, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp", "Source", "Device", "Temp(C)", "Humidity(%)",
                                 "CO2(ppm)", "Pressure(hPa)", "Altitude(m)", "RSSI", "Alarm"])
                for d in self.data_history:
                    writer.writerow(self._format_data_row(d))
            self._log(f"Exported {len(self.data_history)} records to {filename}")
            messagebox.showinfo("Export", f"Saved to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))

    def _export_xlsx_multi_sheet(self, devices):
        import openpyxl
        filename = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            filetypes=[("Excel files", "*.xlsx"), ("All files", "*.*")],
            initialfile=f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        )
        if not filename:
            return
        try:
            wb = openpyxl.Workbook()
            wb.remove(wb.active)
            header = ["Timestamp", "Source", "Device", "Temp(C)", "Humidity(%)",
                      "CO2(ppm)", "Pressure(hPa)", "Altitude(m)", "RSSI", "Alarm"]
            total = 0
            for dev in sorted(devices):
                ws = wb.create_sheet(title=dev or "Unknown")
                ws.append(header)
                for d in self.data_history:
                    if d.device == dev:
                        ws.append(self._format_data_row(d))
                        total += 1
            wb.save(filename)
            self._log(f"Exported {total} records ({len(devices)} sheets) to {filename}")
            messagebox.showinfo("Export", f"Saved {len(devices)} device sheets to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))

    def _export_csv_multi_file(self, devices):
        """Fallback when openpyxl not installed: export separate CSV per device."""
        folder = filedialog.askdirectory(title="Select folder for CSV files")
        if not folder:
            return
        try:
            ts_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            header = ["Timestamp", "Source", "Device", "Temp(C)", "Humidity(%)",
                      "CO2(ppm)", "Pressure(hPa)", "Altitude(m)", "RSSI", "Alarm"]
            total = 0
            files = []
            for dev in sorted(devices):
                fname = os.path.join(folder, f"sensor_data_{dev}_{ts_str}.csv")
                with open(fname, "w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(header)
                    for d in self.data_history:
                        if d.device == dev:
                            writer.writerow(self._format_data_row(d))
                            total += 1
                files.append(fname)
            self._log(f"Exported {total} records to {len(files)} files in {folder}")
            messagebox.showinfo("Export", f"Saved {len(files)} CSV files to:\n{folder}")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))

    @staticmethod
    def _format_data_row(d):
        return [
            d.timestamp.strftime("%Y-%m-%d %H:%M:%S"),
            d.source, d.device,
            f"{d.temp:.2f}" if d.temp is not None else "",
            f"{d.hum:.1f}" if d.hum is not None else "",
            d.co2 if d.co2 is not None else "",
            f"{d.pressure:.1f}" if d.pressure is not None else "",
            f"{d.altitude:.1f}" if d.altitude is not None else "",
            d.rssi if d.rssi is not None else "",
            d.alarm if d.alarm else ""
        ]

    def _clear_data(self):
        if messagebox.askyesno("Clear", "Clear all data?"):
            self.data_history.clear()
            self.timestamps.clear()
            self.co2_values.clear()
            self.temp_values.clear()
            self.hum_values.clear()
            self.pressure_values.clear()
            self.altitude_values.clear()
            self.s3_timestamps.clear()
            self.s3_temp_values.clear()
            self.s3_hum_values.clear()
            self.s3_pressure_timestamps.clear()
            self.s3_pressure_values.clear()
            self.s3_altitude_values.clear()
            self.c3_timestamps.clear()
            self.c3_temp_values.clear()
            self.c3_hum_values.clear()
            self.c3_pressure_timestamps.clear()
            self.c3_pressure_values.clear()
            self.c3_altitude_values.clear()
            self.total_data_received = 0
            self.ble_device_status.clear()
            self.current_temp.set("--.-")
            self.current_hum.set("--.-")
            self.current_co2.set("---")
            self.current_pressure.set("----.-")
            self.current_altitude.set("---.-")
            self.current_co2_level.set("")
            self.device_label.set("--")
            self.stat_temp_avg.set("--.-")
            self.stat_temp_max.set("--.-")
            self.stat_temp_min.set("--.-")
            self.stat_hum_avg.set("--.-")
            self.stat_hum_max.set("--.-")
            self.stat_hum_min.set("--.-")
            self.stat_co2_avg.set("---")
            self.stat_co2_max.set("---")
            self.stat_co2_min.set("---")
            self.stat_pressure_avg.set("----.-")
            self.stat_pressure_max.set("----.-")
            self.stat_pressure_min.set("----.-")
            self.stat_alt_min.set("---.-")
            self.stat_alt_max.set("---.-")
            for item in self.history_tree.get_children():
                self.history_tree.delete(item)
            self.lbl_records.config(text="Records: 0")
            self._create_chart_axes()
            self.canvas.draw_idle()
            self._log("Data cleared.")

    def _ask_custom_color(self, parent):
        dlg = tk.Toplevel(parent)
        dlg.title("Custom Color")
        dlg.resizable(False, False)
        dlg.grab_set()
        ttk.Label(dlg, text="Enter RGB hex (e.g. #FF8800):", font=("Segoe UI", 10)).pack(padx=15, pady=(15, 5))
        hex_var = tk.StringVar(value="#")
        entry = ttk.Entry(dlg, textvariable=hex_var, width=12, font=("Consolas", 12))
        entry.pack(padx=15, pady=5)
        entry.focus_set()
        result = [None]

        def _ok():
            val = hex_var.get().strip().upper()
            if not val.startswith("#"):
                val = "#" + val
            if len(val) == 7 and all(c in "0123456789ABCDEF" for c in val[1:]):
                result[0] = val
                dlg.destroy()
            else:
                messagebox.showerror("Error", "Invalid hex color.\nUse format: #RRGGBB (e.g. #FF8800)", parent=dlg)

        entry.bind("<Return>", lambda e: _ok())
        btn_frame = ttk.Frame(dlg)
        btn_frame.pack(pady=(5, 15))
        ttk.Button(btn_frame, text="OK", command=_ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=dlg.destroy).pack(side=tk.LEFT, padx=5)
        dlg.wait_window()
        return result[0]

    def _show_settings(self):
        global _sea_level_pressure_hpa, _sea_level_pressure_source
        global LOCATION_LAT, LOCATION_LON, LOCATION_NAME

        settings_win = tk.Toplevel(self.root)
        settings_win.title("Settings")
        settings_win.resizable(False, False)
        settings_win.grab_set()

        frame = ttk.Frame(settings_win, padding=20)
        frame.pack()

        ttk.Label(frame, text="Location & Sea-Level Pressure", font=("Segoe UI", 12, "bold")).grid(
            row=0, column=0, columnspan=2, pady=(0, 12))

        # Location Name
        ttk.Label(frame, text="Location Name:", font=("Segoe UI", 10)).grid(row=1, column=0, sticky=tk.E, padx=5, pady=4)
        name_var = tk.StringVar(value=LOCATION_NAME)
        ttk.Entry(frame, textvariable=name_var, width=20).grid(row=1, column=1, sticky=tk.W, padx=5, pady=4)

        # Latitude
        ttk.Label(frame, text="Latitude:", font=("Segoe UI", 10)).grid(row=2, column=0, sticky=tk.E, padx=5, pady=4)
        lat_var = tk.StringVar(value=str(LOCATION_LAT))
        ttk.Entry(frame, textvariable=lat_var, width=20).grid(row=2, column=1, sticky=tk.W, padx=5, pady=4)

        # Longitude
        ttk.Label(frame, text="Longitude:", font=("Segoe UI", 10)).grid(row=3, column=0, sticky=tk.E, padx=5, pady=4)
        lon_var = tk.StringVar(value=str(LOCATION_LON))
        ttk.Entry(frame, textvariable=lon_var, width=20).grid(row=3, column=1, sticky=tk.W, padx=5, pady=4)

        # Separator
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=10)

        # Sea-level pressure
        ttk.Label(frame, text="Sea-Level Pressure (hPa):", font=("Segoe UI", 10)).grid(row=5, column=0, sticky=tk.E, padx=5, pady=4)
        pressure_var = tk.StringVar(value=f"{_sea_level_pressure_hpa:.2f}")
        ttk.Entry(frame, textvariable=pressure_var, width=20).grid(row=5, column=1, sticky=tk.W, padx=5, pady=4)

        # Current source info
        src_label = ttk.Label(frame, text=f"Source: {_sea_level_pressure_source}", font=("Consolas", 9), foreground="#666")
        src_label.grid(row=6, column=0, columnspan=2, pady=(2, 8))

        # Separator
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=7, column=0, columnspan=2, sticky=tk.EW, pady=10)

        # Console print toggle
        console_var = tk.BooleanVar(value=self.console_print_enabled)
        ttk.Checkbutton(frame, text="Enable console status print (BLE/runtime info)",
                        variable=console_var).grid(row=8, column=0, columnspan=2, sticky=tk.W, padx=5, pady=4)

        # Separator
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=9, column=0, columnspan=2, sticky=tk.EW, pady=10)

        # Day/Night theme
        ttk.Label(frame, text="Display Theme:", font=("Segoe UI", 10)).grid(row=10, column=0, sticky=tk.E, padx=5, pady=4)
        theme_var = tk.StringVar(value=self.current_theme["name"])
        theme_combo = ttk.Combobox(frame, textvariable=theme_var, values=["Day", "Night"],
                                   state="readonly", width=10)
        theme_combo.grid(row=10, column=1, sticky=tk.W, padx=5, pady=4)

        def _on_theme_change(event=None):
            selected = theme_var.get()
            theme = THEME_NIGHT if selected == "Night" else THEME_DAY
            self._apply_theme(theme)

        theme_combo.bind("<<ComboboxSelected>>", _on_theme_change)

        # Separator
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=11, column=0, columnspan=2, sticky=tk.EW, pady=10)

        # Device Settings
        ttk.Label(frame, text="Device Settings", font=("Segoe UI", 11, "bold")).grid(
            row=12, column=0, columnspan=2, pady=(0, 6))

        ttk.Label(frame, text="Stats Source:", font=("Segoe UI", 10)).grid(
            row=13, column=0, sticky=tk.E, padx=5, pady=4)
        stats_dev_var = tk.StringVar(value=self.stats_device)
        stats_dev_combo = ttk.Combobox(frame, textvariable=stats_dev_var,
                                       values=["All", "S3", "C3"], state="readonly", width=10)
        stats_dev_combo.grid(row=13, column=1, sticky=tk.W, padx=5, pady=4)

        ttk.Label(frame, text="S3 Name Suffix:", font=("Segoe UI", 10)).grid(
            row=14, column=0, sticky=tk.E, padx=5, pady=4)
        s3_suffix_var = tk.StringVar(value=self.s3_name_suffix)
        s3_suffix_entry = ttk.Entry(frame, textvariable=s3_suffix_var, width=16)
        s3_suffix_entry.grid(row=14, column=1, sticky=tk.W, padx=5, pady=4)

        ttk.Label(frame, text="C3 Name Suffix:", font=("Segoe UI", 10)).grid(
            row=15, column=0, sticky=tk.E, padx=5, pady=4)
        c3_suffix_var = tk.StringVar(value=self.c3_name_suffix)
        c3_suffix_entry = ttk.Entry(frame, textvariable=c3_suffix_var, width=16)
        c3_suffix_entry.grid(row=15, column=1, sticky=tk.W, padx=5, pady=4)

        ttk.Label(frame, text="(max 12 chars, shown in chart legends)",
                  font=("Segoe UI", 8), foreground="#888").grid(
            row=16, column=0, columnspan=2, pady=(0, 4))

        # Separator
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=17, column=0, columnspan=2, sticky=tk.EW, pady=10)

        # Alarm Thresholds
        ttk.Label(frame, text="Alarm Thresholds", font=("Segoe UI", 11, "bold")).grid(
            row=18, column=0, columnspan=2, pady=(0, 6))

        alarm_enable_var = tk.BooleanVar(value=self.alarm_enabled)
        ttk.Checkbutton(frame, text="Enable alarm monitoring",
                        variable=alarm_enable_var).grid(row=19, column=0, columnspan=2, sticky=tk.W, padx=5, pady=2)

        alarm_frame = ttk.Frame(frame)
        alarm_frame.grid(row=20, column=0, columnspan=2, sticky=tk.EW, padx=5)

        ttk.Label(alarm_frame, text="Temp High (°C):", font=("Segoe UI", 9)).grid(row=0, column=0, sticky=tk.E, padx=4, pady=2)
        alarm_temp_high_var = tk.StringVar(value=str(self.alarm_temp_high))
        ttk.Entry(alarm_frame, textvariable=alarm_temp_high_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=4, pady=2)
        ttk.Label(alarm_frame, text="Temp Low (°C):", font=("Segoe UI", 9)).grid(row=0, column=2, sticky=tk.E, padx=4, pady=2)
        alarm_temp_low_var = tk.StringVar(value=str(self.alarm_temp_low))
        ttk.Entry(alarm_frame, textvariable=alarm_temp_low_var, width=8).grid(row=0, column=3, sticky=tk.W, padx=4, pady=2)

        ttk.Label(alarm_frame, text="CO2 High (ppm):", font=("Segoe UI", 9)).grid(row=1, column=0, sticky=tk.E, padx=4, pady=2)
        alarm_co2_high_var = tk.StringVar(value=str(self.alarm_co2_high))
        ttk.Entry(alarm_frame, textvariable=alarm_co2_high_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=4, pady=2)
        ttk.Label(alarm_frame, text="CO2 Low (ppm):", font=("Segoe UI", 9)).grid(row=1, column=2, sticky=tk.E, padx=4, pady=2)
        alarm_co2_low_var = tk.StringVar(value=str(self.alarm_co2_low))
        ttk.Entry(alarm_frame, textvariable=alarm_co2_low_var, width=8).grid(row=1, column=3, sticky=tk.W, padx=4, pady=2)

        ttk.Label(alarm_frame, text="Hum High (%):", font=("Segoe UI", 9)).grid(row=2, column=0, sticky=tk.E, padx=4, pady=2)
        alarm_hum_high_var = tk.StringVar(value=str(self.alarm_hum_high))
        ttk.Entry(alarm_frame, textvariable=alarm_hum_high_var, width=8).grid(row=2, column=1, sticky=tk.W, padx=4, pady=2)
        ttk.Label(alarm_frame, text="Hum Low (%):", font=("Segoe UI", 9)).grid(row=2, column=2, sticky=tk.E, padx=4, pady=2)
        alarm_hum_low_var = tk.StringVar(value=str(self.alarm_hum_low))
        ttk.Entry(alarm_frame, textvariable=alarm_hum_low_var, width=8).grid(row=2, column=3, sticky=tk.W, padx=4, pady=2)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=21, column=0, columnspan=2, sticky=tk.EW, pady=10)

        # Chart Line Colors
        ttk.Label(frame, text="Chart Line Colors", font=("Segoe UI", 11, "bold")).grid(
            row=22, column=0, columnspan=2, pady=(0, 6))

        color_frame = ttk.Frame(frame)
        color_frame.grid(row=23, column=0, columnspan=2, sticky=tk.EW, padx=5)
        color_frame.columnconfigure(1, weight=1)
        color_frame.columnconfigure(3, weight=1)

        color_labels = [
            ("Temp S3:", "temp_s3"),
            ("Temp C3:", "temp_c3"),
            ("Temp Single:", "temp_single"),
            ("Hum S3:", "hum_s3"),
            ("Hum C3:", "hum_c3"),
            ("Hum Single:", "hum_single"),
            ("Press S3:", "press_s3"),
            ("Press C3:", "press_c3"),
            ("Press Single:", "press_single"),
            ("Alt S3:", "alt_s3"),
            ("Alt C3:", "alt_c3"),
            ("Alt Single:", "alt_single"),
        ]

        color_vars = {}
        for idx, (label_text, key) in enumerate(color_labels):
            row_idx = idx // 2
            col_offset = (idx % 2) * 2
            ttk.Label(color_frame, text=label_text, font=("Segoe UI", 9)).grid(
                row=row_idx, column=col_offset, sticky=tk.E, padx=(8, 2), pady=2)
            current_hex = self.chart_colors[key]
            current_name = CHART_COLOR_HEX_TO_NAME.get(current_hex, current_hex)
            var = tk.StringVar(value=current_name)
            color_vars[key] = var
            opt = tk.OptionMenu(color_frame, var, *CHART_COLOR_NAMES)
            opt.config(width=10, font=("Segoe UI", 9),
                       fg=CHART_COLOR_PALETTE.get(current_name, current_name if current_name.startswith("#") else "#000000"))
            menu = opt["menu"]
            menu.config(font=("Segoe UI", 9))
            for i, cname in enumerate(CHART_COLOR_NAMES):
                menu.entryconfig(i, foreground=CHART_COLOR_PALETTE.get(cname, "#000000"))
            def _on_color_change(var_ref=var, widget=opt, parent=settings_win):
                sel = var_ref.get()
                if sel == "Custom...":
                    hex_val = self._ask_custom_color(parent)
                    if hex_val:
                        var_ref.set(hex_val)
                        widget.config(fg=hex_val)
                    else:
                        var_ref.set("Green")
                        widget.config(fg=CHART_COLOR_PALETTE["Green"])
                else:
                    widget.config(fg=CHART_COLOR_PALETTE.get(sel, sel if sel.startswith("#") else "#000000"))
            var.trace_add("write", lambda *_a, cb=_on_color_change: cb())
            opt.grid(row=row_idx, column=col_offset + 1, sticky=tk.W, padx=(2, 8), pady=2)

        # Buttons row
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=24, column=0, columnspan=2, pady=(8, 0))

        def _fetch_now():
            try:
                lat = float(lat_var.get())
                lon = float(lon_var.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid latitude/longitude values.", parent=settings_win)
                return
            p = fetch_sea_level_pressure(lat, lon)
            if p is not None:
                pressure_var.set(f"{p:.2f}")
                src_label.config(text=f"Source: Open-Meteo ({name_var.get()}) — fetched just now")
                self._log(f"Fetched sea-level pressure: {p:.2f} hPa for {name_var.get()}")
            else:
                messagebox.showwarning("Fetch Failed", "Could not fetch pressure from Open-Meteo.\nCheck network or coordinates.", parent=settings_win)

        def _apply():
            global _sea_level_pressure_hpa, _sea_level_pressure_source
            global LOCATION_LAT, LOCATION_LON, LOCATION_NAME
            try:
                lat = float(lat_var.get())
                lon = float(lon_var.get())
                pres = float(pressure_var.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid numeric values.", parent=settings_win)
                return
            if not (900 < pres < 1100):
                messagebox.showerror("Error", "Pressure must be between 900 and 1100 hPa.", parent=settings_win)
                return
            LOCATION_LAT = lat
            LOCATION_LON = lon
            LOCATION_NAME = name_var.get().strip() or "Custom"
            _sea_level_pressure_hpa = pres
            _sea_level_pressure_source = f"Manual ({LOCATION_NAME})"
            self.console_print_enabled = console_var.get()
            selected_theme = THEME_NIGHT if theme_var.get() == "Night" else THEME_DAY
            if self.current_theme != selected_theme:
                self._apply_theme(selected_theme)
            # Apply chart colors
            for key, var in color_vars.items():
                name = var.get()
                if name.startswith("#") and len(name) == 7:
                    self.chart_colors[key] = name
                elif name in CHART_COLOR_PALETTE:
                    self.chart_colors[key] = CHART_COLOR_PALETTE[name]
            # Apply device settings
            self.stats_device = stats_dev_var.get()
            self.s3_name_suffix = s3_suffix_var.get().strip()[:12]
            self.c3_name_suffix = c3_suffix_var.get().strip()[:12]
            self._update_statistics()
            # Apply alarm thresholds
            self.alarm_enabled = alarm_enable_var.get()
            try:
                self.alarm_temp_high = float(alarm_temp_high_var.get())
                self.alarm_temp_low = float(alarm_temp_low_var.get())
                self.alarm_co2_high = int(float(alarm_co2_high_var.get()))
                self.alarm_co2_low = int(float(alarm_co2_low_var.get()))
                self.alarm_hum_high = float(alarm_hum_high_var.get())
                self.alarm_hum_low = float(alarm_hum_low_var.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid alarm threshold values.", parent=settings_win)
                return
            self.new_data_flag = True
            self.stat_sea_level_p.set(f"{pres:.1f}")
            self._log(f"Settings updated: {LOCATION_NAME} ({lat}, {lon}), P0={pres:.2f} hPa")
            self._log(f"Console status print: {'ON' if self.console_print_enabled else 'OFF'}")
            self._log(f"Theme: {self.current_theme['name']}")
            self._log(f"Stats source: {self.stats_device}, S3 suffix: '{self.s3_name_suffix}', C3 suffix: '{self.c3_name_suffix}'")
            self._log("Chart colors updated.")
            print(f"[Settings] Location: {LOCATION_NAME} ({lat}, {lon}), Sea-level pressure: {pres:.2f} hPa")
            print(f"[Settings] Console status print: {'ON' if self.console_print_enabled else 'OFF'}")
            print(f"[Settings] Theme: {self.current_theme['name']}")
            # Persist to config file
            _save_config({
                "s3_name_suffix": self.s3_name_suffix,
                "c3_name_suffix": self.c3_name_suffix,
                "stats_device": self.stats_device,
                "chart_colors": self.chart_colors,
                "theme": self.current_theme["name"],
                "location_name": LOCATION_NAME,
                "location_lat": LOCATION_LAT,
                "location_lon": LOCATION_LON,
                "sea_level_pressure": _sea_level_pressure_hpa,
                "console_print": self.console_print_enabled,
                "alarm_enabled": self.alarm_enabled,
                "alarm_temp_high": self.alarm_temp_high,
                "alarm_temp_low": self.alarm_temp_low,
                "alarm_co2_high": self.alarm_co2_high,
                "alarm_co2_low": self.alarm_co2_low,
                "alarm_hum_high": self.alarm_hum_high,
                "alarm_hum_low": self.alarm_hum_low,
            })
            settings_win.destroy()

        ttk.Button(btn_frame, text="Fetch Online", command=_fetch_now).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Apply", command=_apply).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=settings_win.destroy).pack(side=tk.LEFT, padx=5)

    def _show_about(self):
        about_win = tk.Toplevel(self.root)
        about_win.title("About")
        about_win.resizable(False, False)
        about_win.grab_set()

        frame = ttk.Frame(about_win, padding=20)
        frame.pack()

        # Show app icon in About dialog
        try:
            icon_img = Image.open(APP_ICON_PATH)
            self._app_icon_about = ImageTk.PhotoImage(icon_img.resize((64, 64), Image.LANCZOS))
            ttk.Label(frame, image=self._app_icon_about).pack(pady=(0, 8))
        except Exception:
            pass

        ttk.Label(frame, text=f"Sensor Monitor {APP_VERSION['version']}", font=("Segoe UI", 13, "bold")).pack(pady=(0, 8))
        info_lines = [
            "Universal CO2 + ENV Monitor",
            "",
            f"Author:  {APP_VERSION['author']} (tudzl@hotmail.de)",
            f"Date:    {APP_VERSION['date']}",
            "Deps:    matplotlib, bleak, pyserial",
            "",
            "Supported Devices:",
            "  ZELL_S3CO2 — ESP32-S3 CO2 sensor (pkt v0x03/v0x02)",
            "  ZELL_C3ENV — ESP32-C3 ENV sensor (pkt v0x01)",
            "",
            "Data Sources:",
            "  BLE — manufacturer data + scan response",
            "  Serial — JSON lines @ 115200 baud",
            "",
            "Features:",
            "  CO2, Temperature, Humidity, Pressure, Altitude",
            "  Per-device statistics & custom device naming",
            "  Alarm thresholds (Temp/CO2/Hum) with LED indicators",
            "  Double-click chart to maximize in new window",
            "  Settings persistence (sensor_monitor_config.json)",
            "",
            "BLE Packet (company ID 0x90FC):",
            "  v0x03: Temp+Hum+CO2+Pressure 12B (S3+BMP580)",
            "  v0x02: Temp+Hum+CO2 10B (S3)",
            "  v0x01: Temp+Hum+Pressure 10B (C3)",
            "Scan Response: T/H/C/P on UUID 0xFC90",
        ]
        for line in info_lines:
            ttk.Label(frame, text=line, font=("Consolas", 9)).pack(anchor=tk.W)

        ttk.Button(frame, text="OK", command=about_win.destroy).pack(pady=(12, 0))

    def _schedule_console_status(self):
        self._print_console_status()
        self.root.after(10000, self._schedule_console_status)

    def _print_console_status(self):
        if not self.console_print_enabled:
            return
        elapsed = time.time() - self.start_time
        hours, rem = divmod(int(elapsed), 3600)
        minutes, seconds = divmod(rem, 60)
        runtime_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"

        print(f"\n--- BLE Status [{datetime.now().strftime('%H:%M:%S')}] | GUI runtime: {runtime_str} | Total data received: {self.total_data_received} ---")
        print(f"  Sea-level pressure: {_sea_level_pressure_hpa:.2f} hPa (source: {_sea_level_pressure_source})")
        if self.ble_device_status:
            for name, info in self.ble_device_status.items():
                status = "CONNECTED" if info["connected"] else "disconnected"
                last_time = info["last_data_time"].strftime("%H:%M:%S") if info["last_data_time"] else "never"
                print(f"  [{name}] {status} | Last data: {last_time} | Count: {info['data_count']}")
        else:
            print("  No BLE devices detected yet.")
        print("---")

    def _start_pressure_updater(self):
        """Start background thread to periodically fetch sea-level pressure."""
        def _updater():
            first_run = True
            while self.ble_running or not hasattr(self, '_closing'):
                success = update_sea_level_pressure()
                if success:
                    msg = f"Sea-level pressure updated: {_sea_level_pressure_hpa:.2f} hPa ({_sea_level_pressure_source})"
                    self.root.after(0, self._log, msg)
                    self.root.after(0, self.stat_sea_level_p.set, f"{_sea_level_pressure_hpa:.1f}")
                    if first_run:
                        print(f"  [OK] {msg}")
                else:
                    msg = f"Sea-level pressure update failed, using: {_sea_level_pressure_hpa:.2f} hPa"
                    self.root.after(0, self._log, msg)
                    if first_run:
                        print(f"  [FAIL] {msg}")
                first_run = False
                time.sleep(SEA_LEVEL_PRESSURE_UPDATE_INTERVAL_S)

        thread = threading.Thread(target=_updater, daemon=True)
        thread.start()

    def _on_close(self):
        self.ble_running = False
        self.serial_connected = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.root.destroy()


# =============================================================================
if __name__ == "__main__":
    print(f"{APP_VERSION['name']} GUI based on tkinter starting...")
    print(f"Version:{APP_VERSION['version']}, {APP_VERSION['date']}, {APP_VERSION['author']}")
    print(f"BLE devices: {BLE_DEVICE_NAMES}")
    print(f"Default location: {LOCATION_NAME} ({LOCATION_LAT}, {LOCATION_LON})")
    print(f"Sea-level pressure: {_sea_level_pressure_hpa:.2f} hPa (will fetch online in background)")
    print("Ensure Bluetooth is enabled and/or select a serial port in the GUI.")
    root = tk.Tk()
    app = SensorMonitorV3(root)
    root.mainloop()
