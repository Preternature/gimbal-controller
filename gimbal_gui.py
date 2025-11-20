#!/usr/bin/env python3
"""
Gimbal Light Controller GUI
Controls dual-servo gimbal swing and red light LFO via serial
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class GimbalControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Gimbal Light Controller")
        self.root.geometry("520x850")
        self.root.resizable(False, False)

        self.serial_port = None
        self.connected = False
        self.reader_thread = None
        self.running = True

        self.create_widgets()
        self.refresh_ports()

        # Auto-connect to COM6 on startup
        self.root.after(100, self.auto_connect)

    def create_widgets(self):
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=2)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=2)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=4, pady=5)

        # Pin Configuration Frame
        pin_frame = ttk.LabelFrame(self.root, text="Pin Configuration", padding=10)
        pin_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(pin_frame, text="Servo 1 (Top):").grid(row=0, column=0, sticky="w")
        self.servo1_pin_var = tk.StringVar(value="26")
        ttk.Entry(pin_frame, textvariable=self.servo1_pin_var, width=5).grid(row=0, column=1, padx=5)

        ttk.Label(pin_frame, text="Servo 2 (Bottom):").grid(row=0, column=2, sticky="w", padx=(10, 0))
        self.servo2_pin_var = tk.StringVar(value="48")
        ttk.Entry(pin_frame, textvariable=self.servo2_pin_var, width=5).grid(row=0, column=3, padx=5)

        ttk.Button(pin_frame, text="Set Servos", command=self.set_servo_pins).grid(row=0, column=4, padx=5)

        ttk.Label(pin_frame, text="Light Pin:").grid(row=1, column=0, sticky="w", pady=(5, 0))
        self.light_pin_var = tk.StringVar(value="9")
        ttk.Entry(pin_frame, textvariable=self.light_pin_var, width=5).grid(row=1, column=1, padx=5, pady=(5, 0))
        ttk.Button(pin_frame, text="Set Light", command=self.set_light_pin).grid(row=1, column=2, padx=5, pady=(5, 0))

        # Calibration Frame
        cal_frame = ttk.LabelFrame(self.root, text="Servo Calibration", padding=10)
        cal_frame.pack(fill="x", padx=10, pady=5)

        # Servo 1 calibration (top servo - tilt/nadir)
        ttk.Label(cal_frame, text="Servo 1 (Tilt):").grid(row=0, column=0, sticky="w")
        self.servo1_pos_var = tk.IntVar(value=0)
        self.servo1_scale = ttk.Scale(cal_frame, from_=-360, to=360, variable=self.servo1_pos_var,
                                       orient="horizontal", length=150, command=self.on_servo1_move)
        self.servo1_scale.grid(row=0, column=1, padx=5)
        self.servo1_pos_label = ttk.Label(cal_frame, text="0", width=4)
        self.servo1_pos_label.grid(row=0, column=2)
        ttk.Button(cal_frame, text="Set Nadir", command=self.set_servo1_center).grid(row=0, column=3, padx=5)

        # Servo 2 calibration (bottom servo - lateral swing)
        ttk.Label(cal_frame, text="Servo 2 (Swing):").grid(row=1, column=0, sticky="w")
        self.servo2_pos_var = tk.IntVar(value=0)
        self.servo2_scale = ttk.Scale(cal_frame, from_=-360, to=360, variable=self.servo2_pos_var,
                                       orient="horizontal", length=150, command=self.on_servo2_move)
        self.servo2_scale.grid(row=1, column=1, padx=5)
        self.servo2_pos_label = ttk.Label(cal_frame, text="0", width=4)
        self.servo2_pos_label.grid(row=1, column=2)
        ttk.Button(cal_frame, text="Set Center", command=self.set_servo2_center).grid(row=1, column=3, padx=5)

        # Center values display
        ttk.Label(cal_frame, text="Centers:").grid(row=2, column=0, sticky="w", pady=(5, 0))
        self.center1_var = tk.IntVar(value=90)
        self.center2_var = tk.IntVar(value=90)
        center_frame = ttk.Frame(cal_frame)
        center_frame.grid(row=2, column=1, columnspan=3, sticky="w", pady=(5, 0))
        ttk.Label(center_frame, text="S1:").pack(side="left")
        ttk.Entry(center_frame, textvariable=self.center1_var, width=4).pack(side="left", padx=2)
        ttk.Label(center_frame, text="S2:").pack(side="left", padx=(10, 0))
        ttk.Entry(center_frame, textvariable=self.center2_var, width=4).pack(side="left", padx=2)
        ttk.Button(center_frame, text="Apply Centers", command=self.apply_centers).pack(side="left", padx=10)

        # Swing Control Frame
        swing_frame = ttk.LabelFrame(self.root, text="Swing Control", padding=10)
        swing_frame.pack(fill="x", padx=10, pady=5)

        # BPM Control
        ttk.Label(swing_frame, text="BPM:").grid(row=0, column=0, sticky="w")
        self.bpm_var = tk.DoubleVar(value=15.0)
        self.bpm_scale = ttk.Scale(swing_frame, from_=1, to=120, variable=self.bpm_var,
                                    orient="horizontal", length=200, command=self.on_bpm_change)
        self.bpm_scale.grid(row=0, column=1, padx=5)
        self.bpm_label = ttk.Label(swing_frame, text="15.0")
        self.bpm_label.grid(row=0, column=2)
        self.bpm_pending = None  # For delayed send

        # Amplitude Control (top servo tilt)
        ttk.Label(swing_frame, text="Tilt Amp:").grid(row=1, column=0, sticky="w")
        self.amp_var = tk.DoubleVar(value=55.0)
        self.amp_scale = ttk.Scale(swing_frame, from_=0, to=90, variable=self.amp_var,
                                    orient="horizontal", length=200, command=self.on_amp_change)
        self.amp_scale.grid(row=1, column=1, padx=5)
        self.amp_label = ttk.Label(swing_frame, text="55.0")
        self.amp_label.grid(row=1, column=2)
        self.amp_pending = None  # For delayed send

        # Swing Width Control (bottom servo lateral swing)
        ttk.Label(swing_frame, text="Swing Width:").grid(row=2, column=0, sticky="w")
        self.swing_width_var = tk.DoubleVar(value=50.0)
        self.swing_width_scale = ttk.Scale(swing_frame, from_=10, to=120, variable=self.swing_width_var,
                                    orient="horizontal", length=200, command=self.on_swing_width_change)
        self.swing_width_scale.grid(row=2, column=1, padx=5)
        self.swing_width_label = ttk.Label(swing_frame, text="50.0")
        self.swing_width_label.grid(row=2, column=2)
        self.swing_width_pending = None  # For delayed send

        # Presets
        ttk.Label(swing_frame, text="Presets:").grid(row=3, column=0, sticky="w", pady=5)
        preset_frame = ttk.Frame(swing_frame)
        preset_frame.grid(row=3, column=1, columnspan=3, sticky="w")

        for preset in ["GENTLE", "NORMAL", "WILD", "DYING", "CIRCULAR"]:
            ttk.Button(preset_frame, text=preset, width=8,
                      command=lambda p=preset: self.send_command(f"PRESET:{p}")).pack(side="left", padx=2)

        # Light Control Frame
        light_frame = ttk.LabelFrame(self.root, text="Light LFO Control", padding=10)
        light_frame.pack(fill="x", padx=10, pady=5)

        # LFO Frequency
        ttk.Label(light_frame, text="LFO Freq (Hz):").grid(row=0, column=0, sticky="w")
        self.lfo_freq_var = tk.DoubleVar(value=1.0)
        self.lfo_freq_scale = ttk.Scale(light_frame, from_=0.1, to=10.0, variable=self.lfo_freq_var,
                                         orient="horizontal", length=200, command=self.on_lfo_freq_change)
        self.lfo_freq_scale.grid(row=0, column=1, padx=5)
        self.lfo_freq_label = ttk.Label(light_frame, text="1.0")
        self.lfo_freq_label.grid(row=0, column=2)
        self.lfo_pending = None  # For delayed send

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
        control_frame = ttk.LabelFrame(self.root, text="Main Controls", padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)

        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack()

        ttk.Button(btn_frame, text="START SWING", command=lambda: self.send_command("START"),
                   width=15).grid(row=0, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="STOP SWING", command=lambda: self.send_command("STOP"),
                   width=15).grid(row=0, column=1, padx=5, pady=2)

        ttk.Button(btn_frame, text="LIGHT ON", command=lambda: self.send_command("LIGHT_ON"),
                   width=15).grid(row=1, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="LIGHT OFF", command=lambda: self.send_command("LIGHT_OFF"),
                   width=15).grid(row=1, column=1, padx=5, pady=2)

        ttk.Button(btn_frame, text="ALL ON", command=lambda: self.send_command("ALL_ON"),
                   width=15).grid(row=2, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="ALL OFF", command=lambda: self.send_command("ALL_OFF"),
                   width=15).grid(row=2, column=1, padx=5, pady=2)

        ttk.Button(btn_frame, text="RESET", command=lambda: self.send_command("RESET"),
                   width=15).grid(row=3, column=0, padx=5, pady=2)
        ttk.Button(btn_frame, text="STATUS", command=lambda: self.send_command("STATUS"),
                   width=15).grid(row=3, column=1, padx=5, pady=2)

        # Log Frame
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=5)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=8, width=50, state="disabled")
        self.log_text.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        # Default to COM6 if available, otherwise first port
        if 'COM6' in ports:
            self.port_var.set('COM6')
        elif ports:
            self.port_combo.current(0)

    def auto_connect(self):
        if 'COM6' in self.port_combo['values']:
            self.port_var.set('COM6')
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

            # Auto-start light at low brightness after connection settles
            self.root.after(500, self.auto_start_light)

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def auto_start_light(self):
        # Set LFO with default brightness and turn on
        self.set_lfo()
        self.send_command("LIGHT_ON")

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
            messagebox.showwarning("Not Connected", "Please connect to a COM port first")
            return

        try:
            self.serial_port.write(f"{cmd}\n".encode())
            self.log(f"> {cmd}")
        except Exception as e:
            self.log(f"Send error: {e}")

    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def on_bpm_change(self, val):
        self.bpm_label.config(text=f"{float(val):.1f}")
        # Cancel pending send and schedule new one
        if self.bpm_pending:
            self.root.after_cancel(self.bpm_pending)
        self.bpm_pending = self.root.after(1000, self.set_bpm)

    def on_amp_change(self, val):
        self.amp_label.config(text=f"{float(val):.1f}")
        # Cancel pending send and schedule new one
        if self.amp_pending:
            self.root.after_cancel(self.amp_pending)
        self.amp_pending = self.root.after(1000, self.set_amplitude)

    def on_swing_width_change(self, val):
        self.swing_width_label.config(text=f"{float(val):.1f}")
        # Cancel pending send and schedule new one
        if self.swing_width_pending:
            self.root.after_cancel(self.swing_width_pending)
        self.swing_width_pending = self.root.after(1000, self.set_swing_width)

    def on_lfo_freq_change(self, val):
        self.lfo_freq_label.config(text=f"{float(val):.1f}")
        # Cancel pending send and schedule new one
        if self.lfo_pending:
            self.root.after_cancel(self.lfo_pending)
        self.lfo_pending = self.root.after(1000, self.set_lfo)

    def on_lfo_depth_change(self, val):
        self.lfo_depth_label.config(text=f"{float(val):.2f}")
        # Cancel pending send and schedule new one
        if self.lfo_pending:
            self.root.after_cancel(self.lfo_pending)
        self.lfo_pending = self.root.after(1000, self.set_lfo)

    def on_brightness_change(self, val):
        self.brightness_label.config(text=f"{int(float(val))}")
        # Cancel pending send and schedule new one
        if self.lfo_pending:
            self.root.after_cancel(self.lfo_pending)
        self.lfo_pending = self.root.after(1000, self.set_lfo)

    def set_bpm(self):
        bpm = self.bpm_var.get()
        self.send_command(f"BPM:{bpm:.1f}")

    def set_amplitude(self):
        # Tilt amplitude (top servo)
        amp = self.amp_var.get()
        period = 60000.0 / self.bpm_var.get()
        self.send_command(f"SWING1:{amp:.1f},{period:.0f},0")

    def set_swing_width(self):
        # Swing width (bottom servo lateral swing)
        width = self.swing_width_var.get()
        period = 60000.0 / self.bpm_var.get()
        self.send_command(f"SWING2:{width:.1f},{period:.0f},0")

    def set_light_pin(self):
        pin = self.light_pin_var.get()
        self.send_command(f"LIGHT_PIN:{pin}")

    def set_servo_pins(self):
        pin1 = self.servo1_pin_var.get()
        pin2 = self.servo2_pin_var.get()
        self.send_command(f"SERVO_PINS:{pin1},{pin2}")

    def on_servo1_move(self, val):
        pos = int(float(val))
        self.servo1_pos_label.config(text=str(pos))
        self.send_command(f"MOVE1:{pos}")

    def on_servo2_move(self, val):
        pos = int(float(val))
        self.servo2_pos_label.config(text=str(pos))
        self.send_command(f"MOVE2:{pos}")

    def set_servo1_center(self):
        pos = self.servo1_pos_var.get()
        self.center1_var.set(pos)
        self.log(f"Servo 1 center set to {pos}")

    def set_servo2_center(self):
        pos = self.servo2_pos_var.get()
        self.center2_var.set(pos)
        self.log(f"Servo 2 center set to {pos}")

    def apply_centers(self):
        c1 = self.center1_var.get()
        c2 = self.center2_var.get()
        self.send_command(f"CENTER:{c1},{c2}")

    def set_lfo(self):
        freq = self.lfo_freq_var.get()
        depth = self.lfo_depth_var.get()
        brightness = self.brightness_var.get()
        self.send_command(f"LIGHT_LFO:{freq:.2f},{depth:.2f},{brightness}")

    def on_closing(self):
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
