#!/usr/bin/env python3
"""Servo Test GUI - tests base (pin 52) and camera (pin 36) servos"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time

PORT = "COM6"
BAUD = 9600


class ServoTestGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Rail and Gimbal")
        self.root.geometry("500x750")
        self.root.resizable(True, True)

        self.ser = None
        self.connected = False

        self.create_widgets()
        self.refresh_ports()
        self.root.after(200, self.auto_connect)

    def create_widgets(self):
        # Connection
        conn = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value=PORT)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=10)
        self.port_combo.grid(row=0, column=1, padx=5)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=2)
        self.conn_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.conn_btn.grid(row=0, column=3, padx=2)
        self.status_lbl = ttk.Label(conn, text="Disconnected", foreground="red")
        self.status_lbl.grid(row=1, column=0, columnspan=4, pady=(5, 0))

        # Base servo
        base = ttk.LabelFrame(self.root, text="Gimbal  (pin 52)", padding=10)
        base.pack(fill="x", padx=10, pady=5)

        self.base_var = tk.IntVar(value=90)
        self.base_label = ttk.Label(base, text="90°", width=5)
        ttk.Scale(base, from_=0, to=180, variable=self.base_var, orient="horizontal",
                  length=300, command=self.on_base_move).grid(row=0, column=0, padx=5)
        self.base_label.grid(row=0, column=1)

        base_btns = ttk.Frame(base)
        base_btns.grid(row=1, column=0, columnspan=2, pady=(8, 0))
        ttk.Button(base_btns, text="0°",   width=6, command=lambda: self.set_base(0)).pack(side="left", padx=2)
        ttk.Button(base_btns, text="90°",  width=6, command=lambda: self.set_base(90)).pack(side="left", padx=2)
        ttk.Button(base_btns, text="180°", width=6, command=lambda: self.set_base(180)).pack(side="left", padx=2)
        ttk.Button(base_btns, text="Sweep", width=8, command=lambda: self.send("b")).pack(side="left", padx=6)

        # Camera servo
        cam = ttk.LabelFrame(self.root, text="Camera Rail  (pin 36)", padding=10)
        cam.pack(fill="x", padx=10, pady=5)

        # Angle slider
        ttk.Label(cam, text="Angle:").grid(row=0, column=0, sticky="w")
        self.cam_var = tk.IntVar(value=90)
        self.cam_label = ttk.Label(cam, text="90°", width=5)
        ttk.Scale(cam, from_=0, to=180, variable=self.cam_var, orient="horizontal",
                  length=260, command=self.on_cam_move).grid(row=0, column=1, padx=5)
        self.cam_label.grid(row=0, column=2)

        # Microsecond slider
        ttk.Label(cam, text="Pulse (µs):").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.cam_us_var = tk.IntVar(value=1500)
        self.cam_us_label = ttk.Label(cam, text="1500µs", width=7)
        ttk.Scale(cam, from_=400, to=2600, variable=self.cam_us_var, orient="horizontal",
                  length=260, command=self.on_cam_us_move).grid(row=1, column=1, padx=5, pady=(8, 0))
        self.cam_us_label.grid(row=1, column=2, pady=(8, 0))

        cam_btns = ttk.Frame(cam)
        cam_btns.grid(row=2, column=0, columnspan=3, pady=(8, 0))
        ttk.Button(cam_btns, text="400µs",  width=7, command=lambda: self.set_cam_us(400)).pack(side="left", padx=2)
        ttk.Button(cam_btns, text="1000µs", width=7, command=lambda: self.set_cam_us(1000)).pack(side="left", padx=2)
        ttk.Button(cam_btns, text="1500µs", width=7, command=lambda: self.set_cam_us(1500)).pack(side="left", padx=2)
        ttk.Button(cam_btns, text="2000µs", width=7, command=lambda: self.set_cam_us(2000)).pack(side="left", padx=2)
        ttk.Button(cam_btns, text="2600µs", width=7, command=lambda: self.set_cam_us(2600)).pack(side="left", padx=2)
        ttk.Button(cam_btns, text="Sweep",  width=8, command=lambda: self.send("c")).pack(side="left", padx=6)

        # Global controls
        glob = ttk.LabelFrame(self.root, text="Both Servos", padding=10)
        glob.pack(fill="x", padx=10, pady=5)

        gf = ttk.Frame(glob)
        gf.pack()
        ttk.Button(gf, text="Center Both", width=14, command=self.center_both).pack(side="left", padx=4)
        ttk.Button(gf, text="Sweep Both",  width=14, command=lambda: self.send("a")).pack(side="left", padx=4)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=5)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log = tk.Text(log_frame, height=7, state="disabled", bg="#1e1e1e", fg="#d4d4d4",
                           font=("Consolas", 9))
        self.log.pack(fill="both", expand=True)

    # --- Servo control ---

    def on_base_move(self, val):
        angle = int(float(val))
        self.base_label.config(text=f"{angle}°")
        self._send_raw(f"B{angle}\n")

    def on_cam_move(self, val):
        angle = int(float(val))
        self.cam_label.config(text=f"{angle}°")
        self._send_raw(f"C{angle}\n")

    def on_cam_us_move(self, val):
        us = int(float(val))
        self.cam_us_label.config(text=f"{us}µs")
        self._send_raw(f"CUS{us}\n")

    def set_base(self, angle):
        self.base_var.set(angle)
        self.base_label.config(text=f"{angle}°")
        self._send_raw(f"B{angle}\n")

    def set_cam_us(self, us):
        self.cam_us_var.set(us)
        self.cam_us_label.config(text=f"{us}µs")
        self._send_raw(f"CUS{us}\n")

    def center_both(self):
        self.set_base(90)
        self.set_cam_us(1500)

    def send(self, cmd):
        self._send_raw(cmd + "\n")

    def _send_raw(self, text):
        if not self.connected:
            return
        try:
            self.ser.write(text.encode())
            self.append_log(f"> {text.strip()}")
        except Exception as e:
            self.append_log(f"Send error: {e}")

    # --- Connection ---

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if PORT in ports:
            self.port_var.set(PORT)
        elif ports:
            self.port_combo.current(0)

    def auto_connect(self):
        if self.port_var.get():
            self.connect()

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1)
            time.sleep(2)
            self.connected = True
            self.conn_btn.config(text="Disconnect")
            self.status_lbl.config(text=f"Connected to {port}", foreground="green")
            self.append_log(f"Connected to {port}")
            threading.Thread(target=self.reader, daemon=True).start()
        except Exception as e:
            self.status_lbl.config(text="Connection failed", foreground="red")
            self.append_log(f"Error: {e}")

    def disconnect(self):
        self.connected = False
        if self.ser:
            self.ser.close()
        self.conn_btn.config(text="Connect")
        self.status_lbl.config(text="Disconnected", foreground="red")
        self.append_log("Disconnected")

    def reader(self):
        while self.connected:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.root.after(0, self.append_log, f"< {line}")
            except:
                break
            time.sleep(0.05)

    def append_log(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    ServoTestGUI(root)
    root.mainloop()
