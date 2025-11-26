#!/usr/bin/env python3
"""
Gimbal Light Controller GUI
Controls dual gimbal systems on single Arduino Mega via serial
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import json
import os

CONFIG_FILE = "gimbal_config.json"
DEFAULT_PORT = "COM6"

class GimbalUnit:
    """Represents a single gimbal unit's controls (shares serial with other gimbals)"""
    def __init__(self, gimbal_id, name, parent_frame, send_cmd_callback, log_callback, config=None):
        self.gimbal_id = gimbal_id  # 1 or 2
        self.name = name
        self.send_command = send_cmd_callback
        self.log = log_callback

        # Load config or use defaults (different defaults per gimbal)
        config = config or {}
        if gimbal_id == 1:
            self.servo1_pin_default = config.get('servo1_pin', '33')
            self.servo2_pin_default = config.get('servo2_pin', '45')
            self.light_pin_default = config.get('light_pin', '9')
        else:  # gimbal_id == 2
            self.servo1_pin_default = config.get('servo1_pin', '33')
            self.servo2_pin_default = config.get('servo2_pin', '45')
            self.light_pin_default = config.get('light_pin', '10')
        print(f"[{name}] servo1_pin_default={self.servo1_pin_default}, servo2_pin_default={self.servo2_pin_default}")
        self.center1_default = config.get('center1', 90)
        self.center2_default = config.get('center2', 90)
        self.min1_default = config.get('min1', 0)
        self.max1_default = config.get('max1', 180)
        self.min2_default = config.get('min2', 0)
        self.max2_default = config.get('max2', 180)

        self.pin_change_pending = None
        self.light_pin_change_pending = None

        self.create_widgets(parent_frame)

    def create_widgets(self, parent):
        # Pin Configuration Frame
        pin_frame = ttk.LabelFrame(parent, text="Pin Configuration", padding=10)
        pin_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(pin_frame, text="Servo 1 (Top):").grid(row=0, column=0, sticky="w")
        self.servo1_pin_var = tk.StringVar(value=self.servo1_pin_default)
        self.servo1_pin_var.trace_add("write", lambda *args: self.on_pin_change())
        ttk.Entry(pin_frame, textvariable=self.servo1_pin_var, width=5).grid(row=0, column=1, padx=5)

        ttk.Label(pin_frame, text="Servo 2 (Bottom):").grid(row=0, column=2, sticky="w", padx=(10, 0))
        self.servo2_pin_var = tk.StringVar(value=self.servo2_pin_default)
        self.servo2_pin_var.trace_add("write", lambda *args: self.on_pin_change())
        ttk.Entry(pin_frame, textvariable=self.servo2_pin_var, width=5).grid(row=0, column=3, padx=5)

        ttk.Label(pin_frame, text="Light Pin:").grid(row=1, column=0, sticky="w", pady=(5, 0))
        self.light_pin_var = tk.StringVar(value=self.light_pin_default)
        self.light_pin_var.trace_add("write", lambda *args: self.on_light_pin_change())
        ttk.Entry(pin_frame, textvariable=self.light_pin_var, width=5).grid(row=1, column=1, padx=5, pady=(5, 0))

        # Calibration Frame
        cal_frame = ttk.LabelFrame(parent, text="Servo Calibration & Range of Motion", padding=10)
        cal_frame.pack(fill="x", padx=10, pady=5)

        # Servo 1 calibration (top servo - tilt/nadir)
        ttk.Label(cal_frame, text="Servo 1 (Tilt):").grid(row=0, column=0, sticky="w")
        self.servo1_pos_var = tk.IntVar(value=self.center1_default)
        self.servo1_scale = ttk.Scale(cal_frame, from_=-360, to=360, variable=self.servo1_pos_var,
                                       orient="horizontal", length=150, command=self.on_servo1_move)
        self.servo1_scale.grid(row=0, column=1, padx=5)
        self.servo1_pos_label = ttk.Label(cal_frame, text=str(self.center1_default), width=4)
        self.servo1_pos_label.grid(row=0, column=2)
        ttk.Button(cal_frame, text="Set Nadir", command=self.set_servo1_center).grid(row=0, column=3, padx=5)

        # Servo 1 range limits
        s1_range_frame = ttk.Frame(cal_frame)
        s1_range_frame.grid(row=1, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(s1_range_frame, text="  Range:").pack(side="left")
        ttk.Button(s1_range_frame, text="Set Min", command=self.set_servo1_min, width=8).pack(side="left", padx=2)
        self.min1_var = tk.IntVar(value=self.min1_default)
        ttk.Entry(s1_range_frame, textvariable=self.min1_var, width=4).pack(side="left", padx=2)
        ttk.Button(s1_range_frame, text="Set Max", command=self.set_servo1_max, width=8).pack(side="left", padx=2)
        self.max1_var = tk.IntVar(value=self.max1_default)
        ttk.Entry(s1_range_frame, textvariable=self.max1_var, width=4).pack(side="left", padx=2)

        # Servo 2 calibration (bottom servo - lateral swing)
        ttk.Label(cal_frame, text="Servo 2 (Swing):").grid(row=2, column=0, sticky="w", pady=(10, 0))
        self.servo2_pos_var = tk.IntVar(value=self.center2_default)
        self.servo2_scale = ttk.Scale(cal_frame, from_=-360, to=360, variable=self.servo2_pos_var,
                                       orient="horizontal", length=150, command=self.on_servo2_move)
        self.servo2_scale.grid(row=2, column=1, padx=5, pady=(10, 0))
        self.servo2_pos_label = ttk.Label(cal_frame, text=str(self.center2_default), width=4)
        self.servo2_pos_label.grid(row=2, column=2, pady=(10, 0))
        ttk.Button(cal_frame, text="Set Center", command=self.set_servo2_center).grid(row=2, column=3, padx=5, pady=(10, 0))

        # Servo 2 range limits
        s2_range_frame = ttk.Frame(cal_frame)
        s2_range_frame.grid(row=3, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(s2_range_frame, text="  Range:").pack(side="left")
        ttk.Button(s2_range_frame, text="Set Min", command=self.set_servo2_min, width=8).pack(side="left", padx=2)
        self.min2_var = tk.IntVar(value=self.min2_default)
        ttk.Entry(s2_range_frame, textvariable=self.min2_var, width=4).pack(side="left", padx=2)
        ttk.Button(s2_range_frame, text="Set Max", command=self.set_servo2_max, width=8).pack(side="left", padx=2)
        self.max2_var = tk.IntVar(value=self.max2_default)
        ttk.Entry(s2_range_frame, textvariable=self.max2_var, width=4).pack(side="left", padx=2)

        # Center values display
        ttk.Label(cal_frame, text="Centers:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        self.center1_var = tk.IntVar(value=self.center1_default)
        self.center2_var = tk.IntVar(value=self.center2_default)
        center_frame = ttk.Frame(cal_frame)
        center_frame.grid(row=4, column=1, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Label(center_frame, text="S1:").pack(side="left")
        ttk.Entry(center_frame, textvariable=self.center1_var, width=4).pack(side="left", padx=2)
        ttk.Label(center_frame, text="S2:").pack(side="left", padx=(10, 0))
        ttk.Entry(center_frame, textvariable=self.center2_var, width=4).pack(side="left", padx=2)
        ttk.Button(center_frame, text="Apply", command=self.apply_centers).pack(side="left", padx=5)
        ttk.Button(center_frame, text="Apply Limits", command=self.apply_limits).pack(side="left", padx=5)

        # Swing Control Frame
        swing_frame = ttk.LabelFrame(parent, text="Swing Control", padding=10)
        swing_frame.pack(fill="x", padx=10, pady=5)

        # BPM Control
        ttk.Label(swing_frame, text="BPM:").grid(row=0, column=0, sticky="w")
        self.bpm_var = tk.DoubleVar(value=15.0)
        self.bpm_scale = ttk.Scale(swing_frame, from_=1, to=120, variable=self.bpm_var,
                                    orient="horizontal", length=200, command=self.on_bpm_change)
        self.bpm_scale.grid(row=0, column=1, padx=5)
        self.bpm_label = ttk.Label(swing_frame, text="15.0")
        self.bpm_label.grid(row=0, column=2)
        self.bpm_pending = None

        # Amplitude Control (top servo tilt)
        ttk.Label(swing_frame, text="Tilt Amp:").grid(row=1, column=0, sticky="w")
        self.amp_var = tk.DoubleVar(value=55.0)
        self.amp_scale = ttk.Scale(swing_frame, from_=0, to=90, variable=self.amp_var,
                                    orient="horizontal", length=200, command=self.on_amp_change)
        self.amp_scale.grid(row=1, column=1, padx=5)
        self.amp_label = ttk.Label(swing_frame, text="55.0")
        self.amp_label.grid(row=1, column=2)
        self.amp_pending = None

        # Swing Width Control (bottom servo lateral swing)
        ttk.Label(swing_frame, text="Swing Width:").grid(row=2, column=0, sticky="w")
        self.swing_width_var = tk.DoubleVar(value=50.0)
        self.swing_width_scale = ttk.Scale(swing_frame, from_=10, to=120, variable=self.swing_width_var,
                                    orient="horizontal", length=200, command=self.on_swing_width_change)
        self.swing_width_scale.grid(row=2, column=1, padx=5)
        self.swing_width_label = ttk.Label(swing_frame, text="50.0")
        self.swing_width_label.grid(row=2, column=2)
        self.swing_width_pending = None

        # Presets
        ttk.Label(swing_frame, text="Presets:").grid(row=3, column=0, sticky="w", pady=5)
        preset_frame = ttk.Frame(swing_frame)
        preset_frame.grid(row=3, column=1, columnspan=2, sticky="w")

        for preset in ["GENTLE", "NORMAL", "WILD", "DYING", "CIRCULAR"]:
            ttk.Button(preset_frame, text=preset, width=8,
                      command=lambda p=preset: self.send_command(f"G{self.gimbal_id}:PRESET:{p}")).pack(side="left", padx=2)

        # Light Control Frame
        light_frame = ttk.LabelFrame(parent, text="Light LFO Control", padding=10)
        light_frame.pack(fill="x", padx=10, pady=5)

        # LFO Frequency
        ttk.Label(light_frame, text="LFO Freq (Hz):").grid(row=0, column=0, sticky="w")
        self.lfo_freq_var = tk.DoubleVar(value=1.0)
        self.lfo_freq_scale = ttk.Scale(light_frame, from_=0.1, to=10.0, variable=self.lfo_freq_var,
                                         orient="horizontal", length=200, command=self.on_lfo_freq_change)
        self.lfo_freq_scale.grid(row=0, column=1, padx=5)
        self.lfo_freq_label = ttk.Label(light_frame, text="1.0")
        self.lfo_freq_label.grid(row=0, column=2)
        self.lfo_pending = None

        # LFO Depth
        ttk.Label(light_frame, text="LFO Depth:").grid(row=1, column=0, sticky="w")
        self.lfo_depth_var = tk.DoubleVar(value=1.0)
        self.lfo_depth_scale = ttk.Scale(light_frame, from_=0.0, to=1.0, variable=self.lfo_depth_var,
                                          orient="horizontal", length=200, command=self.on_lfo_depth_change)
        self.lfo_depth_scale.grid(row=1, column=1, padx=5)
        self.lfo_depth_label = ttk.Label(light_frame, text="1.0")
        self.lfo_depth_label.grid(row=1, column=2)

        # Brightness
        ttk.Label(light_frame, text="Brightness:").grid(row=2, column=0, sticky="w")
        self.brightness_var = tk.IntVar(value=10)
        self.brightness_scale = ttk.Scale(light_frame, from_=0, to=255, variable=self.brightness_var,
                                           orient="horizontal", length=200, command=self.on_brightness_change)
        self.brightness_scale.grid(row=2, column=1, padx=5)
        self.brightness_label = ttk.Label(light_frame, text="10")
        self.brightness_label.grid(row=2, column=2)

        # Main Controls Frame
        control_frame = ttk.LabelFrame(parent, text="Main Controls", padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)

        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack()

        ttk.Button(btn_frame, text="START SWING", command=lambda: self.send_command(f"G{self.gimbal_id}:START"),
                   width=15).grid(row=0, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="STOP SWING", command=lambda: self.send_command(f"G{self.gimbal_id}:STOP"),
                   width=15).grid(row=0, column=1, padx=5, pady=2)

        ttk.Button(btn_frame, text="LIGHT ON", command=lambda: self.send_command(f"G{self.gimbal_id}:LIGHT_ON"),
                   width=15).grid(row=1, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="LIGHT OFF", command=lambda: self.send_command(f"G{self.gimbal_id}:LIGHT_OFF"),
                   width=15).grid(row=1, column=1, padx=5, pady=2)

        ttk.Button(btn_frame, text="ALL ON", command=lambda: self.send_command(f"G{self.gimbal_id}:ALL_ON"),
                   width=15).grid(row=2, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="ALL OFF", command=lambda: self.send_command(f"G{self.gimbal_id}:ALL_OFF"),
                   width=15).grid(row=2, column=1, padx=5, pady=2)

        ttk.Button(btn_frame, text="RESET", command=lambda: self.send_command(f"G{self.gimbal_id}:RESET"),
                   width=15).grid(row=3, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="STATUS", command=lambda: self.send_command(f"G{self.gimbal_id}:STATUS"),
                   width=15).grid(row=3, column=1, padx=5, pady=2)

    # Slider event handlers
    def on_bpm_change(self, val):
        self.bpm_label.config(text=f"{float(val):.1f}")
        if self.bpm_pending:
            try:
                self.bpm_scale.after_cancel(self.bpm_pending)
            except:
                pass
        self.bpm_pending = self.bpm_scale.after(500, self.set_bpm)

    def on_amp_change(self, val):
        self.amp_label.config(text=f"{float(val):.1f}")
        if self.amp_pending:
            try:
                self.amp_scale.after_cancel(self.amp_pending)
            except:
                pass
        self.amp_pending = self.amp_scale.after(500, self.set_amplitude)

    def on_swing_width_change(self, val):
        self.swing_width_label.config(text=f"{float(val):.1f}")
        if self.swing_width_pending:
            try:
                self.swing_width_scale.after_cancel(self.swing_width_pending)
            except:
                pass
        self.swing_width_pending = self.swing_width_scale.after(500, self.set_swing_width)

    def on_lfo_freq_change(self, val):
        self.lfo_freq_label.config(text=f"{float(val):.1f}")
        if self.lfo_pending:
            try:
                self.lfo_freq_scale.after_cancel(self.lfo_pending)
            except:
                pass
        self.lfo_pending = self.lfo_freq_scale.after(500, self.set_lfo)

    def on_lfo_depth_change(self, val):
        self.lfo_depth_label.config(text=f"{float(val):.2f}")
        if self.lfo_pending:
            try:
                self.lfo_depth_scale.after_cancel(self.lfo_pending)
            except:
                pass
        self.lfo_pending = self.lfo_depth_scale.after(500, self.set_lfo)

    def on_brightness_change(self, val):
        self.brightness_label.config(text=f"{int(float(val))}")
        if self.lfo_pending:
            try:
                self.brightness_scale.after_cancel(self.lfo_pending)
            except:
                pass
        self.lfo_pending = self.brightness_scale.after(500, self.set_lfo)

    def set_bpm(self):
        bpm = self.bpm_var.get()
        self.send_command(f"G{self.gimbal_id}:BPM:{bpm:.1f}")

    def set_amplitude(self):
        amp = self.amp_var.get()
        period = 60000.0 / self.bpm_var.get()
        self.send_command(f"G{self.gimbal_id}:SWING1:{amp:.1f},{period:.0f},0")

    def set_swing_width(self):
        width = self.swing_width_var.get()
        period = 60000.0 / self.bpm_var.get()
        self.send_command(f"G{self.gimbal_id}:SWING2:{width:.1f},{period:.0f},0")

    def on_pin_change(self):
        # Debounce pin changes
        if self.pin_change_pending:
            try:
                self.servo1_scale.after_cancel(self.pin_change_pending)
            except:
                pass
        self.pin_change_pending = self.servo1_scale.after(500, self.set_servo_pins)

    def on_light_pin_change(self):
        # Debounce light pin changes
        if self.light_pin_change_pending:
            try:
                self.servo1_scale.after_cancel(self.light_pin_change_pending)
            except:
                pass
        self.light_pin_change_pending = self.servo1_scale.after(500, self.set_light_pin)

    def set_light_pin(self):
        pin = self.light_pin_var.get()
        if pin.isdigit():
            self.send_command(f"G{self.gimbal_id}:LIGHT_PIN:{pin}")

    def set_servo_pins(self):
        pin1 = self.servo1_pin_var.get()
        pin2 = self.servo2_pin_var.get()
        if pin1.isdigit() and pin2.isdigit():
            self.send_command(f"G{self.gimbal_id}:SERVO_PINS:{pin1},{pin2}")

    def on_servo1_move(self, val):
        pos = int(float(val))
        self.servo1_pos_label.config(text=str(pos))
        self.send_command(f"G{self.gimbal_id}:MOVE1:{pos}")

    def on_servo2_move(self, val):
        pos = int(float(val))
        self.servo2_pos_label.config(text=str(pos))
        self.send_command(f"G{self.gimbal_id}:MOVE2:{pos}")

    def set_servo1_center(self):
        pos = self.servo1_pos_var.get()
        self.center1_var.set(pos)
        self.log(f"[{self.name}] Servo 1 nadir set to {pos}")

    def set_servo2_center(self):
        pos = self.servo2_pos_var.get()
        self.center2_var.set(pos)
        self.log(f"[{self.name}] Servo 2 center set to {pos}")

    def set_servo1_min(self):
        pos = self.servo1_pos_var.get()
        self.min1_var.set(pos)
        self.log(f"[{self.name}] Servo 1 min set to {pos}")

    def set_servo1_max(self):
        pos = self.servo1_pos_var.get()
        self.max1_var.set(pos)
        self.log(f"[{self.name}] Servo 1 max set to {pos}")

    def set_servo2_min(self):
        pos = self.servo2_pos_var.get()
        self.min2_var.set(pos)
        self.log(f"[{self.name}] Servo 2 min set to {pos}")

    def set_servo2_max(self):
        pos = self.servo2_pos_var.get()
        self.max2_var.set(pos)
        self.log(f"[{self.name}] Servo 2 max set to {pos}")

    def apply_centers(self):
        c1 = self.center1_var.get()
        c2 = self.center2_var.get()
        self.send_command(f"G{self.gimbal_id}:CENTER:{c1},{c2}")

    def apply_limits(self):
        min1 = self.min1_var.get()
        max1 = self.max1_var.get()
        min2 = self.min2_var.get()
        max2 = self.max2_var.get()
        self.send_command(f"G{self.gimbal_id}:LIMITS:{min1},{max1},{min2},{max2}")
        self.log(f"[{self.name}] Applied limits: S1={min1}-{max1}, S2={min2}-{max2}")

    def set_lfo(self):
        freq = self.lfo_freq_var.get()
        depth = self.lfo_depth_var.get()
        brightness = self.brightness_var.get()
        self.send_command(f"G{self.gimbal_id}:LIGHT_LFO:{freq:.2f},{depth:.2f},{brightness}")

    def get_config(self):
        """Return current configuration for saving"""
        return {
            'servo1_pin': self.servo1_pin_var.get(),
            'servo2_pin': self.servo2_pin_var.get(),
            'light_pin': self.light_pin_var.get(),
            'center1': self.center1_var.get(),
            'center2': self.center2_var.get(),
            'min1': self.min1_var.get(),
            'max1': self.max1_var.get(),
            'min2': self.min2_var.get(),
            'max2': self.max2_var.get(),
        }


class GimbalControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual Gimbal Light Controller")
        self.root.geometry("560x950")
        self.root.resizable(False, False)

        self.serial_port = None
        self.connected = False
        self.reader_thread = None
        self.running = True

        self.config = self.load_config()
        self.gimbal_units = []

        self.create_widgets()
        self.refresh_ports()

        # Auto-connect on startup
        self.root.after(100, self.auto_connect)

    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    return json.load(f)
            except:
                pass
        return {}

    def save_config(self):
        config = {
            'port': self.port_var.get(),
            'gimbal1': self.gimbal_units[0].get_config() if len(self.gimbal_units) > 0 else {},
            'gimbal2': self.gimbal_units[1].get_config() if len(self.gimbal_units) > 1 else {},
        }
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            self.log("Configuration saved")
        except Exception as e:
            self.log(f"Error saving config: {e}")

    def create_widgets(self):
        # Connection Frame at top (shared by all gimbals)
        conn_frame = ttk.LabelFrame(self.root, text="Connection (Arduino Mega)", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky="w")
        saved_port = self.config.get('port', DEFAULT_PORT)
        self.port_var = tk.StringVar(value=saved_port)
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=2)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=2)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=4, pady=5)

        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=5, pady=5)

        # Create scrollable frames for each gimbal
        gimbal1_container = ttk.Frame(self.notebook)
        gimbal2_container = ttk.Frame(self.notebook)

        # Add canvas and scrollbar for scrolling
        for container, gimbal_id, name, config_key in [
            (gimbal1_container, 1, "Gimbal 1", "gimbal1"),
            (gimbal2_container, 2, "Gimbal 2", "gimbal2")
        ]:
            canvas = tk.Canvas(container, highlightthickness=0)
            scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
            scrollable_frame = ttk.Frame(canvas)

            scrollable_frame.bind(
                "<Configure>",
                lambda e, c=canvas: c.configure(scrollregion=c.bbox("all"))
            )

            canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
            canvas.configure(yscrollcommand=scrollbar.set)

            canvas.pack(side="left", fill="both", expand=True)
            scrollbar.pack(side="right", fill="y")

            # Bind mousewheel for scrolling
            def _on_mousewheel(event, canvas=canvas):
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            canvas.bind_all("<MouseWheel>", _on_mousewheel)

            # Create gimbal unit
            gimbal_config = self.config.get(config_key, {})
            gimbal = GimbalUnit(gimbal_id, name, scrollable_frame, self.send_command, self.log, gimbal_config)
            self.gimbal_units.append(gimbal)

        self.notebook.add(gimbal1_container, text="Gimbal 1")
        self.notebook.add(gimbal2_container, text="Gimbal 2")

        # Global controls tab
        global_frame = ttk.Frame(self.notebook)
        self.notebook.add(global_frame, text="Global")
        self.create_global_controls(global_frame)

        # Log Frame at bottom
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=5)
        log_frame.pack(fill="both", expand=False, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=6, width=60, state="disabled")
        self.log_text.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)

    def create_global_controls(self, parent):
        """Create controls that affect both gimbals"""
        ttk.Label(parent, text="Global Controls", font=('TkDefaultFont', 12, 'bold')).pack(pady=10)

        btn_frame = ttk.Frame(parent)
        btn_frame.pack(pady=10)

        ttk.Button(btn_frame, text="START ALL", command=self.start_all, width=20).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(btn_frame, text="STOP ALL", command=self.stop_all, width=20).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(btn_frame, text="LIGHTS ON ALL", command=self.lights_on_all, width=20).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(btn_frame, text="LIGHTS OFF ALL", command=self.lights_off_all, width=20).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(btn_frame, text="ALL ON", command=self.all_on, width=20).grid(row=2, column=0, padx=5, pady=5)
        ttk.Button(btn_frame, text="ALL OFF", command=self.all_off, width=20).grid(row=2, column=1, padx=5, pady=5)

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=20)

        # Save/Load config
        config_frame = ttk.Frame(parent)
        config_frame.pack(pady=10)
        ttk.Button(config_frame, text="Save Configuration", command=self.save_config, width=20).pack(pady=5)
        ttk.Button(config_frame, text="Reload Ports", command=self.refresh_ports, width=20).pack(pady=5)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        current = self.port_var.get()
        if current and current in ports:
            pass  # Keep current selection
        elif DEFAULT_PORT in ports:
            self.port_var.set(DEFAULT_PORT)
        elif ports:
            self.port_combo.current(0)

    def auto_connect(self):
        port = self.port_var.get()
        if port:
            self.connect()

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port")
            return

        try:
            self.serial_port = serial.Serial(port, 9600, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text=f"Connected to {port}", foreground="green")
            self.log(f"Connected to {port}")

            # Start reader thread
            self.reader_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.reader_thread.start()

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        if self.serial_port:
            self.serial_port.close()
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.log("Disconnected")

    def read_serial(self):
        while self.connected and self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.root.after(0, self.log, f"< {line}")
            except Exception as e:
                if self.connected:
                    self.root.after(0, self.log, f"Read error: {e}")
            time.sleep(0.05)

    def send_command(self, cmd):
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to COM port first")
            return

        try:
            self.serial_port.write(f"{cmd}\n".encode())
            self.log(f"> {cmd}")
        except Exception as e:
            self.log(f"Send error: {e}")

    def start_all(self):
        self.send_command("G1:START")
        self.send_command("G2:START")

    def stop_all(self):
        self.send_command("G1:STOP")
        self.send_command("G2:STOP")

    def lights_on_all(self):
        self.send_command("G1:LIGHT_ON")
        self.send_command("G2:LIGHT_ON")

    def lights_off_all(self):
        self.send_command("G1:LIGHT_OFF")
        self.send_command("G2:LIGHT_OFF")

    def all_on(self):
        self.send_command("G1:ALL_ON")
        self.send_command("G2:ALL_ON")

    def all_off(self):
        self.send_command("G1:ALL_OFF")
        self.send_command("G2:ALL_OFF")

    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def on_closing(self):
        self.save_config()
        self.running = False
        if self.connected:
            self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = GimbalControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
