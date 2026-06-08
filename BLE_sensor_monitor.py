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

#to fix version 1603 issue
#import platform 
#platform.mac_ver = lambda: ('26.3', ('', '', ''), 'arm64')

# ========== Configuration ==========
BLE_DEVICE_NAME = "ZELL_C3ENV_Sensor"
BLE_COMPANY_ID = 0x90FC  # little-endian: 0xFC, 0x90
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
            self._serial.write((cmd + "\n").encode())

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
    def __init__(self, data_callback):
        self.data_callback = data_callback
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
            try:
                devices = await BleakScanner.discover(
                    timeout=SCAN_INTERVAL_S,
                    return_adv=True
                )
                for addr, (device, adv_data) in devices.items():
                    if not self._match_device(device, adv_data):
                        continue
                    mfr = adv_data.manufacturer_data
                    for company_id, raw in mfr.items():
                        if company_id == BLE_COMPANY_ID:
                            full_data = bytes([company_id & 0xFF, (company_id >> 8) & 0xFF]) + raw
                            result = decode_manufacturer_data(full_data)
                            if result:
                                result["source"] = "BLE"
                                result["rssi"] = adv_data.rssi
                                result["timestamp"] = datetime.now()
                                self.data_callback(result)
                            break
            except Exception as e:
                print(f"[BLE Error] {e}")
                await asyncio.sleep(2)

    def _match_device(self, device, adv_data):
        if device.name and BLE_DEVICE_NAME in device.name:
            return True
        if BLE_COMPANY_ID in adv_data.manufacturer_data:
            return True
        return False


# ========== GUI Application ==========

class SensorMonitorApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("BLE + UART Sensor Monitor V1.1 - ZELL_C3ENV_Sensor")
        self.root.geometry("1200x850")
        self.root.minsize(1000, 700)

        self.history = deque(maxlen=MAX_HISTORY)
        self.scanner = BLEScanner(self._on_data_received)
        self.serial_reader = None
        self._pending_data = []
        self._pending_logs = []
        self._lock = threading.Lock()

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

        # UART log
        log_frame = ttk.Frame(serial_frame)
        log_frame.pack(fill=tk.X, pady=(3, 0))
        self.uart_log = tk.Text(log_frame, height=4, font=("Consolas", 8), wrap=tk.NONE, state=tk.DISABLED)
        log_scroll_y = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.uart_log.yview)
        log_scroll_x = ttk.Scrollbar(log_frame, orient=tk.HORIZONTAL, command=self.uart_log.xview)
        self.uart_log.configure(yscrollcommand=log_scroll_y.set, xscrollcommand=log_scroll_x.set)
        self.uart_log.pack(side=tk.LEFT, fill=tk.X, expand=True)
        log_scroll_y.pack(side=tk.RIGHT, fill=tk.Y)

        self._refresh_ports()

        # ===== History table =====
        table_frame = ttk.LabelFrame(self.root, text="History", padding=3)
        table_frame.pack(fill=tk.X, padx=10, pady=(5, 3))

        cols = ("Time", "Source", "Temp(°C)", "Humidity(%)", "Pressure(hPa)", "Altitude(m)", "RSSI")
        self.tree = ttk.Treeview(table_frame, columns=cols, show="headings", height=5)
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=110, anchor=tk.CENTER)
        self.tree.column("Time", width=145)
        self.tree.column("Source", width=70)
        self.tree.column("RSSI", width=60)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.X, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # ===== Charts =====
        chart_frame = ttk.Frame(self.root)
        chart_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.fig = Figure(figsize=(10, 3.5), dpi=90)
        self.fig.set_tight_layout(True)
        self.axes = [
            self.fig.add_subplot(2, 2, 1),
            self.fig.add_subplot(2, 2, 2),
            self.fig.add_subplot(2, 2, 3),
            self.fig.add_subplot(2, 2, 4),
        ]
        titles = ["Temperature (°C)", "Humidity (%)", "Pressure (hPa)", "Altitude (m)"]
        colors = ["#E63946", "#457B9D", "#2A9D8F", "#E9C46A"]
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
        self.uart_log.insert(tk.END, text + "\n")
        self.uart_log.see(tk.END)
        # Keep log manageable
        line_count = int(self.uart_log.index('end-1c').split('.')[0])
        if line_count > 200:
            self.uart_log.delete('1.0', f'{line_count - 200}.0')
        self.uart_log.config(state=tk.DISABLED)

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
        ts = data["timestamp"].strftime("%Y-%m-%d %H:%M:%S")
        source = data.get("source", "?")
        temp = f"{data['temperature']:.1f}" if data["temperature"] is not None else "---"
        hum = f"{data['humidity']:.1f}" if data["humidity"] is not None else "---"
        press = f"{data['pressure']:.1f}" if data["pressure"] is not None else "---"
        alt = f"{data['altitude']:.1f}" if data["altitude"] is not None else "---"
        rssi = str(data.get("rssi", "-"))

        self.tree.insert("", 0, values=(ts, source, temp, hum, press, alt, rssi))
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
        for item in self.tree.get_children():
            self.tree.delete(item)
        self._update_charts()
        self.lbl_count.config(text="Records: 0")
        for lbl in self.value_labels:
            lbl.config(text="---")
        self.lbl_status.config(text="History cleared. Scanning...")

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.scanner.stop()
        if self.serial_reader:
            self.serial_reader.stop()
        self.root.destroy()


if __name__ == "__main__":
    print("BLE + UART Sensor Monitor GUI based on tkinter starting...")
    print("Version:1.1,7.June.2026, ZELL")
    print(f"BLE device: {BLE_DEVICE_NAME}")
    print(f"Sea level pressure(default): {SEA_LEVEL_PRESSURE_HPA} hPa")
    print("Ensure Bluetooth is enabled and/or select a serial port in the GUI.")
    print("-" * 50)
    app = SensorMonitorApp()
    app.run()
