"""
BLE + UART Sensor Monitor for ESP32C3 ZELL_C3ENV_Sensor
Receives sensor data via BLE broadcast AND/OR serial port (UART).
Displays current values, history table, and trend charts.

Requires: pip install bleak matplotlib pyserial
"""

import asyncio
import struct
import threading
import time
import csv
import json
import math
import re
from datetime import datetime
from collections import deque
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.dates as mdates

# ========== Version Info ==========
APP_VERSION = {
    "name": "BLE + UART Sensor Monitor",
    "version": "1.3",
    "date": "8.June.2026",
    "author": "ZELL",
    "email": "tudzl@hotmail.de",
    "target": "ESP32C3 ZELL_C3ENV_Sensor",
    "requires": "bleak, matplotlib, pyserial",
}

# ========== Configuration ==========
BLE_DEVICE_NAME = "ZELL_C3ENV"  # matches both short name and full name containing this prefix
BLE_COMPANY_ID = 0x90FC  # little-endian: 0xFC, 0x90
BLE_SERVICE_DATA_UUID = "0000fc90-0000-1000-8000-00805f9b34fb"  # 16-bit 0xFC90 as 128-bit
SEA_LEVEL_PRESSURE_HPA = 1025.1
MAX_HISTORY = 500
SCAN_INTERVAL_S = 2.0
SERIAL_BAUDRATE = 115200

# ========== BLE Data Decode ==========

def decode_manufacturer_data(raw_bytes):
    """Decode 10-byte manufacturer data from ESP32C3."""
    if len(raw_bytes) < 10:
        return None

    data = raw_bytes[:10]
    if data[0] != 0xFC or data[1] != 0x90:
        return None

    xor_check = 0
    for i in range(2, 9):
        xor_check ^= data[i]
    if xor_check != data[9]:
        return None

    version = data[2]
    temp_raw = struct.unpack_from('<h', data, 3)[0]
    hum_raw = struct.unpack_from('<h', data, 5)[0]
    press_raw = struct.unpack_from('<H', data, 7)[0]

    temp = None if temp_raw == 0x7FFF else temp_raw / 100.0
    humidity = None if hum_raw == 0x7FFF else hum_raw / 100.0
    pressure = None if press_raw == 0xFFFF else (press_raw + 8000) / 10.0

    altitude = None
    if pressure is not None:
        altitude = 44330.0 * (1.0 - math.pow(pressure / SEA_LEVEL_PRESSURE_HPA, 1.0 / 5.255))

    return {
        "version": version,
        "temperature": temp,
        "humidity": humidity,
        "pressure": pressure,
        "altitude": altitude,
    }


def decode_scan_response_service_data(raw_bytes):
    """Decode scan response service data: plaintext 'T23.5 H65.2 P1013.2' from UUID 0xFC90."""
    try:
        text = raw_bytes.decode("utf-8", errors="replace").strip()
    except Exception:
        return None

    m = re.match(r"T([-\d.]+|--)\s+H([-\d.]+|--)\s+P([-\d.]+|--)", text)
    if not m:
        return None

    temp_str, hum_str, press_str = m.group(1), m.group(2), m.group(3)
    temp = None if temp_str == "--" else float(temp_str)
    humidity = None if hum_str == "--" else float(hum_str)
    pressure = None if press_str == "--" else float(press_str)

    altitude = None
    if pressure is not None:
        altitude = 44330.0 * (1.0 - math.pow(pressure / SEA_LEVEL_PRESSURE_HPA, 1.0 / 5.255))

    return {
        "version": "SR",
        "temperature": temp,
        "humidity": humidity,
        "pressure": pressure,
        "altitude": altitude,
    }


# ========== UART Data Parser ==========

class UARTParser:
    """Parses ESP32 UART output to extract sensor readings.
    Supports two formats:
    1. Line-by-line: "Temperature: 25.30 C", "Humidity: 55.20 %", etc.
    2. JSON block from 'update' command
    """

    def __init__(self):
        self._temp = None
        self._humidity = None
        self._pressure = None
        self._altitude = None
        self._in_json = False
        self._json_buf = ""

    def parse_line(self, line):
        """Parse a single UART line. Returns a sensor dict when a complete reading is available."""
        line = line.strip()
        if not line:
            return None

        # Detect JSON start
        if line == "{":
            self._in_json = True
            self._json_buf = line
            return None

        if self._in_json:
            self._json_buf += "\n" + line
            if line == "}":
                self._in_json = False
                return self._parse_json(self._json_buf)
            return None

        # Line-by-line sensor parsing
        m = re.match(r"Temperature:\s*([-\d.]+)\s*C", line)
        if m:
            self._temp = float(m.group(1))
            return None

        m = re.match(r"Humidity:\s*([-\d.]+)\s*%", line)
        if m:
            self._humidity = float(m.group(1))
            return None

        m = re.match(r"Pressure:\s*([-\d.]+)\s*hPa", line)
        if m:
            self._pressure = float(m.group(1))
            return None

        m = re.match(r"Altitude:\s*([-\d.]+)\s*m", line)
        if m:
            self._altitude = float(m.group(1))
            result = self._build_result()
            self._reset()
            return result

        # "--- Current Sensor Readings ---" resets partial state
        if "Current Sensor Readings" in line:
            self._reset()

        return None

    def _build_result(self):
        return {
            "source": "UART",
            "version": "-",
            "temperature": self._temp,
            "humidity": self._humidity,
            "pressure": self._pressure,
            "altitude": self._altitude,
            "rssi": "-",
            "timestamp": datetime.now(),
        }

    def _reset(self):
        self._temp = None
        self._humidity = None
        self._pressure = None
        self._altitude = None

    def _parse_json(self, json_str):
        """Parse JSON from all_info_print_json()."""
        try:
            data = json.loads(json_str)
            current = data.get("current", {})
            temp = current.get("temperature")
            hum = current.get("humidity")
            press = current.get("pressure")
            alt = current.get("altitude")
            return {
                "source": "UART/JSON",
                "version": "-",
                "temperature": temp,
                "humidity": hum,
                "pressure": press,
                "altitude": alt,
                "rssi": "-",
                "timestamp": datetime.now(),
            }
        except (json.JSONDecodeError, KeyError):
            return None


# ========== Serial Reader Thread ==========

class SerialReader:
    def __init__(self, data_callback, log_callback=None):
        self.data_callback = data_callback
        self.log_callback = log_callback
        self.running = False
        self._thread = None
        self._serial = None
        self._parser = UARTParser()
        self.port = None
        self.connected = False

    def start(self, port, baudrate=SERIAL_BAUDRATE):
        self.port = port
        self.running = True
        self._thread = threading.Thread(target=self._run, args=(port, baudrate), daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.connected = False

    def send_command(self, cmd):
        if self._serial and self._serial.is_open:
            self._serial.write((cmd + "\r\n").encode())
            self._serial.flush()

    def _run(self, port, baudrate):
        try:
            self._serial = serial.Serial(port, baudrate, timeout=1)
            self.connected = True
            if self.log_callback:
                self.log_callback(f"[UART] Connected to {port} @ {baudrate}")
        except serial.SerialException as e:
            if self.log_callback:
                self.log_callback(f"[UART Error] Cannot open {port}: {e}")
            self.connected = False
            return

        while self.running:
            try:
                if self._serial.in_waiting:
                    line = self._serial.readline().decode("utf-8", errors="replace")
                    if self.log_callback:
                        self.log_callback(line.rstrip())
                    result = self._parser.parse_line(line)
                    if result:
                        self.data_callback(result)
                else:
                    time.sleep(0.05)
            except serial.SerialException:
                if self.log_callback:
                    self.log_callback("[UART] Connection lost.")
                self.connected = False
                break
            except Exception as e:
                if self.log_callback:
                    self.log_callback(f"[UART Error] {e}")

        if self._serial and self._serial.is_open:
            self._serial.close()
        self.connected = False


# ========== BLE Scanner Thread ==========

class BLEScanner:
    def __init__(self, data_callback, status_callback=None):
        self.data_callback = data_callback
        self.status_callback = status_callback
        self.running = False
        self._thread = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False

    def _run(self):
        asyncio.run(self._scan_loop())

    async def _scan_loop(self):
        from bleak import BleakScanner

        while self.running:
            found_this_scan = False
            try:
                devices = await BleakScanner.discover(
                    timeout=SCAN_INTERVAL_S,
                    return_adv=True
                )
                for addr, (device, adv_data) in devices.items():
                    if not self._match_device(device, adv_data):
                        continue
                    found_this_scan = True
                    print(f"[BLE] Found: {device.name or 'N/A'} ({addr}) RSSI={adv_data.rssi}dBm")

                    result = None

                    # Method 1: manufacturer data (advData)
                    mfr = adv_data.manufacturer_data
                    for company_id, raw in mfr.items():
                        if company_id == BLE_COMPANY_ID:
                            full_data = bytes([company_id & 0xFF, (company_id >> 8) & 0xFF]) + raw
                            result = decode_manufacturer_data(full_data)
                            if result:
                                result["source"] = "BLE/MFR"
                                print(f"[BLE] Decoded via manufacturer data")
                            break

                    # Method 2: scan response service data (UUID 0xFC90 plaintext)
                    if result is None:
                        svc_data = adv_data.service_data
                        for uuid_str, raw in svc_data.items():
                            if "fc90" in uuid_str.lower():
                                result = decode_scan_response_service_data(raw)
                                if result:
                                    result["source"] = "BLE/SR"
                                    print(f"[BLE] Decoded via scan response: {raw.decode('utf-8', errors='replace').strip()}")
                                break

                    if result:
                        result["rssi"] = adv_data.rssi
                        result["timestamp"] = datetime.now()
                        self.data_callback(result)
                    else:
                        print(f"[BLE] Device found but data decode failed. "
                              f"MFR keys={list(mfr.keys())}, SVC keys={list(adv_data.service_data.keys())}")
            except Exception as e:
                print(f"[BLE Error] {e}")
                await asyncio.sleep(2)

            if self.status_callback:
                self.status_callback(found_this_scan)

    def _match_device(self, device, adv_data):
        if device.name and BLE_DEVICE_NAME in device.name:
            return True
        if BLE_COMPANY_ID in adv_data.manufacturer_data:
            return True
        for uuid_str in adv_data.service_data.keys():
            if "fc90" in uuid_str.lower():
                return True
        return False


# ========== GUI Application ==========

class SensorMonitorApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title(f"{APP_VERSION['name']} V{APP_VERSION['version']} - {BLE_DEVICE_NAME}")
        self.root.geometry("1200x850")
        self.root.minsize(1000, 700)

        self.history = deque(maxlen=MAX_HISTORY)
        self.scanner = BLEScanner(self._on_data_received, self._on_ble_status)
        self.serial_reader = None
        self._pending_data = []
        self._pending_logs = []
        self._lock = threading.Lock()
        self._ble_device_found = False
        self._ble_last_seen = 0

        self._build_ui()
        self._start_scanner()
        self._poll_data()

    def _build_ui(self):
        # ===== Top frame: current values + controls =====
        top_frame = ttk.Frame(self.root, padding=5)
        top_frame.pack(fill=tk.X)

        # Current values panel
        values_frame = ttk.LabelFrame(top_frame, text="Current Sensor Values", padding=8)
        values_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.lbl_status = ttk.Label(values_frame, text="Scanning via BLE...", font=("Consolas", 9))
        self.lbl_status.grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 3))

        # BLE device indicator (blue=found, gray=not found)
        ble_ind_frame = ttk.Frame(values_frame)
        ble_ind_frame.grid(row=0, column=4, sticky=tk.E, padx=(10, 0))
        self.ble_indicator = tk.Canvas(ble_ind_frame, width=16, height=16, highlightthickness=0)
        self.ble_indicator.pack(side=tk.LEFT)
        self._ble_led = self.ble_indicator.create_oval(2, 2, 14, 14, fill="gray", outline="darkgray")
        self.lbl_ble_ind = ttk.Label(ble_ind_frame, text="BLE: N/A", font=("Consolas", 8))
        self.lbl_ble_ind.pack(side=tk.LEFT, padx=(3, 0))

        labels = ["Temperature", "Humidity", "Pressure", "Altitude"]
        units = ["°C", "%", "hPa", "m"]
        self.value_labels = []
        for i, (name, unit) in enumerate(zip(labels, units)):
            ttk.Label(values_frame, text=f"{name}:", font=("Segoe UI", 10)).grid(row=1, column=i, sticky=tk.W, padx=5)
            lbl = ttk.Label(values_frame, text="---", font=("Consolas", 15, "bold"), foreground="#0066CC")
            lbl.grid(row=2, column=i, sticky=tk.W, padx=5)
            ttk.Label(values_frame, text=unit, font=("Segoe UI", 8)).grid(row=3, column=i, sticky=tk.W, padx=5)
            self.value_labels.append(lbl)

        # Controls panel (right side)
        ctrl_frame = ttk.LabelFrame(top_frame, text="Controls", padding=5)
        ctrl_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))

        ttk.Button(ctrl_frame, text="Export CSV", command=self._export_csv).pack(pady=2, fill=tk.X)
        ttk.Button(ctrl_frame, text="Clear History", command=self._clear_history).pack(pady=2, fill=tk.X)
        self.lbl_count = ttk.Label(ctrl_frame, text="Records: 0")
        self.lbl_count.pack(pady=2)
        ttk.Button(ctrl_frame, text="About", command=self._show_about).pack(pady=2, fill=tk.X)

        # ===== Serial port frame =====
        serial_frame = ttk.LabelFrame(self.root, text="UART Serial Port", padding=5)
        serial_frame.pack(fill=tk.X, padx=10, pady=(5, 0))

        port_row = ttk.Frame(serial_frame)
        port_row.pack(fill=tk.X)

        ttk.Label(port_row, text="Port:").pack(side=tk.LEFT)
        self.combo_port = ttk.Combobox(port_row, width=15, state="readonly")
        self.combo_port.pack(side=tk.LEFT, padx=5)
        ttk.Button(port_row, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=2)
        self.btn_connect = ttk.Button(port_row, text="Connect", command=self._toggle_serial)
        self.btn_connect.pack(side=tk.LEFT, padx=5)
        self.lbl_serial_status = ttk.Label(port_row, text="Disconnected", foreground="gray")
        self.lbl_serial_status.pack(side=tk.LEFT, padx=10)

        # UART command entry
        ttk.Label(port_row, text="CMD:").pack(side=tk.LEFT, padx=(20, 0))
        self.entry_cmd = ttk.Entry(port_row, width=20)
        self.entry_cmd.pack(side=tk.LEFT, padx=3)
        self.entry_cmd.bind("<Return>", self._send_serial_cmd)
        ttk.Button(port_row, text="Send", command=self._send_serial_cmd).pack(side=tk.LEFT, padx=2)

        # UART log + History/Charts in a resizable PanedWindow
        paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=(3, 0))

        # -- Pane 1: UART log (resizable height) --
        log_pane = ttk.LabelFrame(paned, text="UART Log (drag border to resize)", padding=3)
        log_frame = ttk.Frame(log_pane)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.uart_log = tk.Text(log_frame, height=7, font=("Consolas", 8), wrap=tk.NONE,
                                bg="#2E2E2E", fg="#2E8B57", insertbackground="#FFFFFF", state=tk.DISABLED)
        self.uart_log.tag_configure("cmd", foreground="#FFA500")
        log_scroll_y = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.uart_log.yview)
        log_scroll_x = ttk.Scrollbar(log_frame, orient=tk.HORIZONTAL, command=self.uart_log.xview)
        self.uart_log.configure(yscrollcommand=log_scroll_y.set, xscrollcommand=log_scroll_x.set)
        log_scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.uart_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        paned.add(log_pane, weight=1)

        # -- Pane 2: History + Charts --
        bottom_pane = ttk.Frame(paned)
        paned.add(bottom_pane, weight=3)

        self._refresh_ports()

        # ===== History table =====
        table_frame = ttk.LabelFrame(bottom_pane, text="History", padding=3)
        table_frame.pack(fill=tk.X, pady=(5, 3))

        style = ttk.Style()
        style.configure("History.Treeview",
                        background="#F5F5F0",
                        foreground="#333333",
                        fieldbackground="#F5F5F0",
                        font=("Consolas", 9))
        style.configure("History.Treeview.Heading",
                        background="#E0E0D8",
                        foreground="#2C3E50",
                        font=("Segoe UI", 9, "bold"))
        style.map("History.Treeview",
                  background=[("selected", "#B3D9FF")],
                  foreground=[("selected", "#1A1A1A")])

        cols = ("Time", "Idx", "Source", "Temp(°C)", "Humidity(%)", "Pressure(hPa)", "Altitude(m)", "RSSI")
        self.tree = ttk.Treeview(table_frame, columns=cols, show="headings", height=5,
                                 style="History.Treeview")
        self.tree.tag_configure("oddrow", background="#EDEDEA")
        self.tree.tag_configure("evenrow", background="#F5F5F0")
        self._row_index = 0
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=110, anchor=tk.CENTER)
        self.tree.column("Time", width=145)
        self.tree.column("Idx", width=45)
        self.tree.column("Source", width=70)
        self.tree.column("RSSI", width=60)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.X, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # ===== Charts =====
        chart_frame = ttk.Frame(bottom_pane)
        chart_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.fig = Figure(figsize=(10, 3.5), dpi=90)
        self.fig.set_tight_layout(True)
        self.axes = [
            self.fig.add_subplot(2, 2, 1),
            self.fig.add_subplot(2, 2, 2),
            self.fig.add_subplot(2, 2, 3),
            self.fig.add_subplot(2, 2, 4),
        ]
        titles = ["Temperature (°C)", "Humidity (%)", "Pressure (hPa)", "Altitude (m)"]
        colors = ["#FF8C00", "#87CEEB", "#2A9D8F", "#8B0000"]
        self.chart_colors = colors
        for ax, title in zip(self.axes, titles):
            ax.set_title(title, fontsize=9)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ===== Serial port methods =====

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.combo_port["values"] = port_list
        if port_list:
            self.combo_port.current(0)

    def _toggle_serial(self):
        if self.serial_reader and self.serial_reader.connected:
            self.serial_reader.stop()
            self.serial_reader = None
            self.btn_connect.config(text="Connect")
            self.lbl_serial_status.config(text="Disconnected", foreground="gray")
        else:
            selection = self.combo_port.get()
            if not selection:
                messagebox.showwarning("UART", "No port selected.")
                return
            port = selection.split(" - ")[0].strip()
            self.serial_reader = SerialReader(self._on_data_received, self._on_uart_log)
            self.serial_reader.start(port)
            self.btn_connect.config(text="Disconnect")
            self.lbl_serial_status.config(text=f"Connecting to {port}...", foreground="orange")
            self.root.after(1000, self._check_serial_status)

    def _check_serial_status(self):
        if self.serial_reader and self.serial_reader.connected:
            self.lbl_serial_status.config(text=f"Connected: {self.serial_reader.port}", foreground="green")
        elif self.serial_reader and not self.serial_reader.connected:
            self.lbl_serial_status.config(text="Connection failed", foreground="red")
            self.btn_connect.config(text="Connect")

    def _send_serial_cmd(self, event=None):
        cmd = self.entry_cmd.get().strip()
        if not cmd:
            return
        if self.serial_reader and self.serial_reader.connected:
            self.serial_reader.send_command(cmd)
            self._append_uart_log(f">>> {cmd}")
            self.entry_cmd.delete(0, tk.END)
        else:
            messagebox.showwarning("UART", "Serial port not connected.")

    def _on_uart_log(self, msg):
        with self._lock:
            self._pending_logs.append(msg)

    def _append_uart_log(self, text):
        self.uart_log.config(state=tk.NORMAL)
        tag = "cmd" if text.startswith(">>>") else None
        self.uart_log.insert(tk.END, text + "\n", tag)
        self.uart_log.see(tk.END)
        line_count = int(self.uart_log.index('end-1c').split('.')[0])
        if line_count > 200:
            self.uart_log.delete('1.0', f'{line_count - 200}.0')
        self.uart_log.config(state=tk.DISABLED)

    # ===== BLE status =====

    def _on_ble_status(self, found):
        with self._lock:
            self._ble_device_found = found
            if found:
                self._ble_last_seen = time.time()

    def _update_ble_indicator(self):
        if self._ble_device_found or (time.time() - self._ble_last_seen < 10):
            self.ble_indicator.itemconfig(self._ble_led, fill="#2196F3", outline="#1565C0")
            self.lbl_ble_ind.config(text="BLE: Online")
        else:
            self.ble_indicator.itemconfig(self._ble_led, fill="gray", outline="darkgray")
            self.lbl_ble_ind.config(text="BLE: N/A")

    # ===== Data handling =====

    def _start_scanner(self):
        self.scanner.start()

    def _on_data_received(self, data):
        with self._lock:
            self._pending_data.append(data)

    def _poll_data(self):
        with self._lock:
            pending = list(self._pending_data)
            self._pending_data.clear()
            logs = list(self._pending_logs)
            self._pending_logs.clear()

        for log_msg in logs:
            self._append_uart_log(log_msg)

        for data in pending:
            self.history.append(data)
            self._update_current_values(data)
            self._add_table_row(data)

        if pending:
            self._update_charts()
            self.lbl_count.config(text=f"Records: {len(self.history)}")

        self._update_ble_indicator()
        self.root.after(500, self._poll_data)

    def _update_current_values(self, data):
        fields = ["temperature", "humidity", "pressure", "altitude"]
        for i, field in enumerate(fields):
            val = data.get(field)
            if val is not None:
                self.value_labels[i].config(text=f"{val:.1f}")
            else:
                self.value_labels[i].config(text="---")

        ts = data["timestamp"].strftime("%H:%M:%S")
        source = data.get("source", "?")
        rssi = data.get("rssi", "-")
        self.lbl_status.config(text=f"Last: {ts} | Source: {source} | RSSI: {rssi} dBm | Ver: {data.get('version', '-')}")

    def _add_table_row(self, data):
        self._row_index += 1
        ts = data["timestamp"].strftime("%Y-%m-%d %H:%M:%S")
        source = data.get("source", "?")
        temp = f"{data['temperature']:.1f}" if data["temperature"] is not None else "---"
        hum = f"{data['humidity']:.1f}" if data["humidity"] is not None else "---"
        press = f"{data['pressure']:.1f}" if data["pressure"] is not None else "---"
        alt = f"{data['altitude']:.1f}" if data["altitude"] is not None else "---"
        rssi = str(data.get("rssi", "-"))

        row_count = len(self.tree.get_children())
        tag = "oddrow" if row_count % 2 else "evenrow"
        self.tree.insert("", 0, values=(ts, self._row_index, source, temp, hum, press, alt, rssi), tags=(tag,))
        children = self.tree.get_children()
        if len(children) > 100:
            for item in children[100:]:
                self.tree.delete(item)

    def _update_charts(self):
        if not self.history:
            return

        times = [d["timestamp"] for d in self.history]
        fields = ["temperature", "humidity", "pressure", "altitude"]

        for i, (ax, field, color) in enumerate(zip(self.axes, fields, self.chart_colors)):
            ax.clear()
            values = [d.get(field) for d in self.history]
            valid_times = [t for t, v in zip(times, values) if v is not None]
            valid_values = [v for v in values if v is not None]

            if valid_values:
                ax.plot(valid_times, valid_values, color=color, linewidth=1.2, marker='o', markersize=2)
                ax.set_title(f"{field.capitalize()} ({valid_values[-1]:.1f})", fontsize=9)
            else:
                ax.set_title(f"{field.capitalize()} (---)", fontsize=9)

            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
            ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))

        self.fig.set_tight_layout(True)
        self.canvas.draw_idle()

    def _export_csv(self):
        if not self.history:
            messagebox.showinfo("Export", "No data to export.")
            return

        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv")],
            initialfile=f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        if not filepath:
            return

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Source", "Temperature(C)", "Humidity(%)", "Pressure(hPa)", "Altitude(m)", "RSSI"])
            for d in self.history:
                writer.writerow([
                    d["timestamp"].strftime("%Y-%m-%d %H:%M:%S"),
                    d.get("source", ""),
                    f"{d['temperature']:.2f}" if d["temperature"] is not None else "",
                    f"{d['humidity']:.2f}" if d["humidity"] is not None else "",
                    f"{d['pressure']:.2f}" if d["pressure"] is not None else "",
                    f"{d['altitude']:.2f}" if d["altitude"] is not None else "",
                    d.get("rssi", ""),
                ])
        messagebox.showinfo("Export", f"Data exported to:\n{filepath}")

    def _clear_history(self):
        self.history.clear()
        self._row_index = 0
        for item in self.tree.get_children():
            self.tree.delete(item)
        self._update_charts()
        self.lbl_count.config(text="Records: 0")
        for lbl in self.value_labels:
            lbl.config(text="---")
        self.lbl_status.config(text="History cleared. Scanning...")

    def _show_about(self):
        about_win = tk.Toplevel(self.root)
        about_win.title("About")
        about_win.resizable(False, False)
        about_win.grab_set()

        frame = ttk.Frame(about_win, padding=20)
        frame.pack()

        v = APP_VERSION
        ttk.Label(frame, text=v["name"], font=("Segoe UI", 12, "bold")).pack(pady=(0, 5))
        info_lines = [
            f"Version: {v['version']}",
            f"Date: {v['date']}",
            f"Author: {v['author']} ({v['email']})",
            f"Target Device: {v['target']}",
            f"Dependencies: {v['requires']}",
            "",
            "Receives sensor data via BLE broadcast",
            "and/or UART serial port. Displays current",
            "values, history table, and trend charts.",
        ]
        for line in info_lines:
            ttk.Label(frame, text=line, font=("Consolas", 9)).pack(anchor=tk.W)

        ttk.Button(frame, text="OK", command=about_win.destroy).pack(pady=(12, 0))

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.scanner.stop()
        if self.serial_reader:
            self.serial_reader.stop()
        self.root.destroy()


if __name__ == "__main__":
    print(f"{APP_VERSION['name']} GUI based on tkinter starting...")
    print(f"Version:{APP_VERSION['version']}, {APP_VERSION['date']}, {APP_VERSION['author']}")
    print(f"BLE device: {BLE_DEVICE_NAME}")
    print(f"Sea level pressure(default): {SEA_LEVEL_PRESSURE_HPA} hPa")
    print("Ensure Bluetooth is enabled and/or select a serial port in the GUI.")
    print("-" * 50)
    app = SensorMonitorApp()
    app.run()
