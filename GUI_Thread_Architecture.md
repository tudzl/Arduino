# Sensor Monitor V3.3 — GUI Thread Architecture

## Overview

This document explains how the front-end (main) thread and back-end (daemon) threads are implemented in `BLE_sensor_monitor_co2_temp_pressureV3.3.py`.

---

## Front-end Thread (Main Thread)

**Tkinter main event loop** — only one, running in `root.mainloop()`:

```python
root = tk.Tk()
app = SensorMonitorV3(root)
root.mainloop()  # Main thread blocks here, processing all UI events
```

Responsibilities:
- All UI rendering and user interactions (button clicks, window refresh, etc.)
- Scheduled periodic tasks via `root.after(ms, callback)`:
  - `_schedule_chart_update()` — checks every 500ms whether charts need refreshing
  - `_schedule_console_status()` — prints backend status to console every 10000ms

**Key constraint**: Tkinter is NOT thread-safe. All UI operations must execute in the main thread.

---

## Back-end Threads (Daemon Threads)

There are 3 background threads, all set to `daemon=True` (automatically terminated when the main thread exits):

### 1. BLE Scan Thread

```python
self.ble_thread = threading.Thread(target=self._ble_thread_func, daemon=True)
```

Internally creates an independent asyncio event loop to run async BLE scanning:

```python
def _ble_thread_func(self):
    self.ble_loop = asyncio.new_event_loop()      # Independent event loop
    asyncio.set_event_loop(self.ble_loop)
    self.ble_loop.run_until_complete(self._ble_scan_loop())  # Blocks here
```

`_ble_scan_loop()` is an `async` coroutine that loops calling `BleakScanner.discover()`. When data is found, it uses `self.root.after(0, callback)` to pass results back to the main thread for processing.

### 2. Serial Reader Thread

```python
self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
```

```python
def _serial_reader(self):
    while self.serial_connected and self.serial_port:
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline()...
            self.root.after(0, self._process_serial_line, line)  # Pass back to main thread
        else:
            time.sleep(0.05)  # Avoid busy-waiting
```

### 3. Pressure Updater Thread

```python
thread = threading.Thread(target=_updater, daemon=True)
```

```python
def _updater():
    while ...:
        success = update_sea_level_pressure()  # HTTP request (may block up to 10s)
        self.root.after(0, self._log, msg)     # Pass result back to main thread
        time.sleep(1800)                       # Every 30 minutes
```

---

## Inter-thread Communication

The only bridge is **`self.root.after(0, callback, *args)`**:

```
Background thread ──── root.after(0, fn) ────> Main thread event queue ──── mainloop picks up and executes
```

This is Tkinter's thread-safe calling method — background threads never directly manipulate the UI. Instead, they post callback functions into the main thread's event queue, which `mainloop` executes on its next iteration.

---

## Flow Diagram

```
Main Thread (Tkinter mainloop)
  |
  |-- after(500ms) --> _schedule_chart_update()   [Periodic chart refresh]
  |-- after(10s)   --> _schedule_console_status() [Periodic status print]
  |-- UI event handling (buttons/inputs)
  |
  |  <-- root.after(0, _add_data, ...)            [Receive data from backend]
  |  <-- root.after(0, _log, ...)                 [Receive logs from backend]
  |
Background Thread 1: BLE scan loop (asyncio)
Background Thread 2: Serial reader (while loop + sleep)
Background Thread 3: Pressure updater (while loop + sleep 30min)
```

---

## Summary

**Main thread handles UI only; background threads handle blocking I/O operations.** Data flows one-way from background threads to the main thread via `root.after()`, avoiding locks and race conditions.

---

*Document created: 2026/06/15*
*Applies to: BLE_sensor_monitor_co2_temp_pressureV3.3.py*
