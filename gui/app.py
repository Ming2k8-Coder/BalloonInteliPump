import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import csv
import queue
import socket
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import collections

# Protocol Constants
BAUD_RATE = 115200

class SerialManager:
    def __init__(self, port, baud, data_callback):
        self.port = port
        self.baud = baud
        self.serial = None
        self.running = False
        self.thread = None
        self.data_callback = data_callback
        self.cmd_queue = queue.Queue()

    def start(self):
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"Error opening serial: {e}")
            return False

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()

    def send(self, cmd):
        self.cmd_queue.put(cmd)

    def _reader_loop(self):
        while self.running:
            try:
                # Write waiting commands
                if not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get()
                    if self.serial and self.serial.is_open:
                        self.serial.write((cmd + '\n').encode())
                
                # Read data
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_callback(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"Serial Error: {e}")
                break

class UDPManager:
    def __init__(self, port, data_callback):
        self.port = port
        self.data_callback = data_callback
        self.socket = None
        self.running = False
        self.thread = None
        self.device_ip = None  # Discovered dynamically
        self.cmd_queue = queue.Queue()

    def start(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.socket.bind(("", self.port))
            self.socket.settimeout(0.5)
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"Error opening UDP socket: {e}")
            return False

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

    def send(self, cmd):
        self.cmd_queue.put(cmd)

    def _reader_loop(self):
        while self.running:
            try:
                # Send queued commands if device IP has been auto-discovered
                if not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get()
                    if self.device_ip:
                        self.socket.sendto((cmd + '\n').encode(), (self.device_ip, self.port))
                    else:
                        print(f"UDP Command skipped: Device IP not discovered yet (waiting for telemetry)")
                
                # Listen for telemetry data
                try:
                    data, addr = self.socket.recvfrom(4096)
                    self.device_ip = addr[0]  # Auto-learn IP of the ESP32
                    packet_str = data.decode('utf-8', errors='ignore')
                    for line in packet_str.split('\n'):
                        line = line.strip()
                        if line:
                            self.data_callback(line)
                except socket.timeout:
                    pass
            except Exception as e:
                if self.running:
                    print(f"UDP Thread Error: {e}")
                break

class CalibrationWizard(tk.Toplevel):
    def __init__(self, parent_app, send_cmd_cb):
        super().__init__(parent_app.root)
        self.title("Sensor Calibration Assistant")
        self.geometry("450x550")
        self.app = parent_app
        self.send_cmd = send_cmd_cb
        
        main_frame = ttk.Frame(self, padding=20)
        main_frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main_frame, text="Sensor Calibration Wizard", font=('Arial', 14, 'bold')).pack(pady=10)
        
        # Step 1
        group1 = ttk.LabelFrame(main_frame, text="1. Select Sensor", padding=10)
        group1.pack(fill=tk.X, pady=5)
        self.sensor_var = tk.StringVar(value="pressure")
        for s in ["pressure", "voltage", "current"]:
            ttk.Radiobutton(group1, text=s.capitalize(), variable=self.sensor_var, value=s).pack(side=tk.LEFT, padx=10)
            
        ttk.Button(group1, text="Reset Values", command=self.reset_cal).pack(side=tk.RIGHT)
        
        # Step 2
        group2 = ttk.LabelFrame(main_frame, text="2. Add Data Points", padding=10)
        group2.pack(fill=tk.X, pady=5)
        
        input_frame = ttk.Frame(group2)
        input_frame.pack(pady=5)
        
        ttk.Label(input_frame, text="Raw ADC:").grid(row=0, column=0, sticky='w')
        self.ent_raw = ttk.Entry(input_frame, width=15)
        self.ent_raw.grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Button(input_frame, text="Use Current Raw", command=self.use_current_raw).grid(row=0, column=2, padx=5)
        
        ttk.Label(input_frame, text="Real Value:").grid(row=1, column=0, sticky='w')
        self.ent_real = ttk.Entry(input_frame, width=15)
        self.ent_real.grid(row=1, column=1, padx=5, pady=2)
        self.lbl_unit = ttk.Label(input_frame, text="kPa / V / A")
        self.lbl_unit.grid(row=1, column=2, sticky='w')
        
        ttk.Button(group2, text="Add Calibration Point", command=self.add_point).pack(fill=tk.X, pady=10)
        
        # Step 3
        group3 = ttk.LabelFrame(main_frame, text="3. Finalize", padding=10)
        group3.pack(fill=tk.X, pady=5)
        
        ttk.Label(group3, text="Points are stored in temporary buffer until saved.", wraplength=350).pack(pady=5)
        ttk.Button(group3, text="SAVE PERMANENTLY TO EEPROM", command=self.save_cal, style="Danger.TButton").pack(fill=tk.X, pady=5)
        
    def use_current_raw(self):
        s = self.sensor_var.get()
        val = 0
        if s == "pressure": val = self.app.raw_press
        elif s == "voltage": val = self.app.raw_volts
        elif s == "current": val = self.app.raw_amps
        self.ent_raw.delete(0, tk.END)
        self.ent_raw.insert(0, str(val))

    def reset_cal(self):
        if messagebox.askyesno("Confirm", f"Reset internal buffer for {self.sensor_var.get()}?"):
            self.send_cmd(f"CAL_CLR {self.sensor_var.get()}")

    def add_point(self):
        raw = self.ent_raw.get()
        real = self.ent_real.get()
        if raw and real:
            try:
                float(raw)
                float(real)
            except ValueError:
                messagebox.showerror("Error", "Invalid numeric value")
                return
            self.send_cmd(f"CAL_PT {self.sensor_var.get()} {raw} {real}")
            self.ent_real.delete(0, tk.END)
            messagebox.showinfo("Success", f"Point ({raw}, {real}) added.")

    def save_cal(self):
        if messagebox.askyesno("Confirm", "Commit all changes to Arduino EEPROM?"):
            self.send_cmd("CAL_SAVE")
            self.destroy()

class BalloonApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Balloon Intelligent Pump Controller")
        self.root.geometry("1200x850")
        
        self.connection_mgr = None
        self.is_connected = False
        self.recording = False
        self.log_file = None
        self.csv_writer = None
        
        # Telemetry Store
        self.raw_press = 0
        self.raw_volts = 0
        self.raw_amps = 0
        
        # Balloon Health & Analysis Metrics
        self.fatigue_index = 0.0
        self.estimated_burst = 50.0  # Estimate in kPa, dynamically adjusted
        self.last_time = None
        
        # Data Buffers for Plotting (Sliding window of points)
        self.max_points = 500
        self.times = collections.deque(maxlen=self.max_points)
        self.pressures = collections.deque(maxlen=self.max_points)
        self.slopes = collections.deque(maxlen=self.max_points)
        self.slope_buffer = collections.deque(maxlen=10) # 100ms regression window
        
        self._setup_theme()
        self._init_ui()

    def _setup_theme(self):
        self.root.configure(bg="#0b0f19")
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except:
            pass
        
        # Configure Colors for Modern Dark Theme
        style.configure(".", background="#0b0f19", foreground="#f8fafc", fieldbackground="#1e293b")
        style.configure("TFrame", background="#0b0f19")
        style.configure("TLabelframe", background="#0f172a", foreground="#94a3b8", bordercolor="#334155")
        style.configure("TLabelframe.Label", background="#0f172a", foreground="#94a3b8", font=('Helvetica', 9, 'bold'))
        style.configure("TLabel", background="#0b0f19", foreground="#f8fafc")
        style.configure("TButton", background="#3b82f6", foreground="#ffffff", font=('Helvetica', 9, 'bold'), borderwidth=0)
        style.map("TButton", background=[('active', '#2563eb')])
        style.configure("Danger.TButton", background="#f43f5e", foreground="#ffffff", font=('Helvetica', 9, 'bold'))
        style.map("Danger.TButton", background=[('active', '#e11d48')])
        style.configure("TNotebook", background="#0b0f19", borderwidth=0)
        style.configure("TNotebook.Tab", background="#1e293b", foreground="#94a3b8", padding=[10, 4])
        style.map("TNotebook.Tab", background=[('selected', '#3b82f6')], foreground=[('selected', '#ffffff')])
        
    def _init_ui(self):
        # 1. Connection Bar (Grid Layout for clean responsiveness)
        top_frame = ttk.Frame(self.root, padding=5)
        top_frame.pack(fill=tk.X)
        
        ttk.Label(top_frame, text="Conn Mode:").grid(row=0, column=0, padx=2)
        self.conn_mode = ttk.Combobox(top_frame, values=["Serial", "WiFi (UDP)"], width=11, state="readonly")
        self.conn_mode.grid(row=0, column=1, padx=2)
        self.conn_mode.current(0)
        self.conn_mode.bind("<<ComboboxSelected>>", self.on_conn_mode_changed)
        
        self.port_label = ttk.Label(top_frame, text="Port:")
        self.port_label.grid(row=0, column=2, padx=2)
        self.port_combo = ttk.Combobox(top_frame, width=10)
        self.port_combo.grid(row=0, column=3, padx=2)
        self.refresh_ports()
        
        self.udp_label = ttk.Label(top_frame, text="UDP Port:")
        self.ent_udp_port = ttk.Entry(top_frame, width=6)
        self.ent_udp_port.insert(0, "8888")
        
        self.btn_connect = ttk.Button(top_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=4, padx=5)
        
        self.btn_refresh = ttk.Button(top_frame, text="Refresh", command=self.refresh_ports)
        self.btn_refresh.grid(row=0, column=5, padx=2)
        
        self.btn_record = ttk.Button(top_frame, text="Start Log", command=self.toggle_recording)
        self.btn_record.grid(row=0, column=6, padx=20, sticky='e')
        
        # 2. Main Content (Paned Window)
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left Panel: Controls & Gauges
        left_panel = ttk.Frame(paned, width=300)
        paned.add(left_panel, weight=1)
        
        self._init_gauges(left_panel)
        self._init_controls(left_panel)
        
        # Right Panel: Graphs
        right_panel = ttk.Frame(paned)
        paned.add(right_panel, weight=3)
        self._init_plots(right_panel)

    def on_conn_mode_changed(self, event=None):
        mode = self.conn_mode.get()
        if mode == "Serial":
            self.udp_label.grid_forget()
            self.ent_udp_port.grid_forget()
            self.port_label.grid(row=0, column=2, padx=2)
            self.port_combo.grid(row=0, column=3, padx=2)
            self.btn_refresh.grid(row=0, column=5, padx=2)
        else:
            self.port_label.grid_forget()
            self.port_combo.grid_forget()
            self.btn_refresh.grid_forget()
            self.udp_label.grid(row=0, column=2, padx=2)
            self.ent_udp_port.grid(row=0, column=3, padx=2)

    def _init_gauges(self, parent):
        frame = ttk.LabelFrame(parent, text="Telemetry & Material Health", padding=10)
        frame.pack(fill=tk.X, pady=5)
        
        self.lbl_pressure = self._create_display_val(frame, "Pressure", "0.00 kPa", 0)
        self.lbl_pwm = self._create_display_val(frame, "PWM Speed", "0", 1)
        self.lbl_volts = self._create_display_val(frame, "Voltage", "0.00 V", 2)
        self.lbl_amps = self._create_display_val(frame, "Current", "0.00 A", 3)
        self.lbl_power = self._create_display_val(frame, "Power", "0.00 W", 4)
        self.lbl_state = self._create_display_val(frame, "Mode", "IDLE", 5)
        
        # Advanced Play/Lifetime Gauges
        self.lbl_reserve = self._create_display_val(frame, "Elastic Reserve", "100.0%", 6)
        self.lbl_health = self._create_display_val(frame, "Balloon Health", "100.0%", 7)
        self.lbl_slope = self._create_display_val(frame, "dP/dt Rate", "0.00 kPa/s", 8)

    def _create_display_val(self, parent, label, val, row):
        ttk.Label(parent, text=label, font=('Arial', 9, 'bold')).grid(row=row, column=0, sticky='w', pady=2)
        lbl = ttk.Label(parent, text=val, font=('Arial', 12, 'bold'), foreground='white')
        lbl.grid(row=row, column=1, sticky='e', padx=10, pady=2)
        return lbl

    def _init_controls(self, parent):
        tabs = ttk.Notebook(parent)
        tabs.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Manual Tab
        tab_man = ttk.Frame(tabs, padding=10)
        tabs.add(tab_man, text="Manual")
        
        ttk.Label(tab_man, text="PWM Control").pack(anchor='w')
        self.slider_pwm = ttk.Scale(tab_man, from_=0, to=255, orient=tk.HORIZONTAL, command=self.on_pwm_change)
        self.slider_pwm.pack(fill=tk.X, pady=5)
        
        ttk.Separator(tab_man).pack(fill=tk.X, pady=10)
        
        ttk.Label(tab_man, text="Target Pressure (kPa)").pack(anchor='w')
        self.ent_target_p = ttk.Entry(tab_man)
        self.ent_target_p.insert(0, "20.0")
        self.ent_target_p.pack(fill=tk.X, pady=5)
        ttk.Button(tab_man, text="Set Target Pressure", command=self.set_target_p).pack(fill=tk.X, pady=2)
        
        ttk.Separator(tab_man).pack(fill=tk.X, pady=10)
        
        ttk.Label(tab_man, text="Solenoid Relief Valve").pack(anchor='w')
        frame_valve = ttk.Frame(tab_man)
        frame_valve.pack(fill=tk.X, pady=2)
        ttk.Button(frame_valve, text="Open (100%)", command=lambda: self.send_cmd("VALVE_ON")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(frame_valve, text="Close (0%)", command=lambda: self.send_cmd("VALVE_OFF")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        ttk.Label(tab_man, text="Valve PWM (0-255)").pack(anchor='w', pady=(5,0))
        self.slider_valve_pwm = ttk.Scale(tab_man, from_=0, to=255, orient=tk.HORIZONTAL, command=self.on_valve_pwm_change)
        self.slider_valve_pwm.pack(fill=tk.X, pady=2)
        
        ttk.Separator(tab_man).pack(fill=tk.X, pady=10)
        
        ttk.Button(tab_man, text="Start Manual", command=lambda: self.send_cmd("START_MANUAL")).pack(fill=tk.X, pady=5)
        ttk.Button(tab_man, text="STOP PUMP", command=lambda: self.send_cmd("STOP"), style="Danger.TButton").pack(fill=tk.X, pady=5)
        
        # Smart Tab
        tab_smart = ttk.Frame(tabs, padding=10)
        tabs.add(tab_smart, text="Smart")
        ttk.Label(tab_smart, text="Yield Ratio").pack(anchor='w')
        self.ent_ratio = ttk.Entry(tab_smart)
        self.ent_ratio.insert(0, "1.2")
        self.ent_ratio.pack(fill=tk.X)
        
        ttk.Label(tab_smart, text="Est. Burst kPa (For Play Health)").pack(anchor='w', pady=(10, 0))
        self.ent_burst_est = ttk.Entry(tab_smart)
        self.ent_burst_est.insert(0, "50.0")
        self.ent_burst_est.pack(fill=tk.X)
        ttk.Button(tab_smart, text="Update Estimated Burst", command=self.update_burst_est).pack(fill=tk.X, pady=5)
        
        ttk.Button(tab_smart, text="Start Smart Test", command=self.start_smart).pack(fill=tk.X, pady=10)
        
        # Limits Tab
        tab_lim = ttk.Frame(tabs, padding=10)
        tabs.add(tab_lim, text="Limits")
        ttk.Label(tab_lim, text="UV Limit (V)").grid(row=0, column=0, sticky='w')
        self.ent_uv = ttk.Entry(tab_lim, width=10); self.ent_uv.grid(row=0, column=1, pady=2); self.ent_uv.insert(0, "10.5")
        ttk.Label(tab_lim, text="OC Limit (A)").grid(row=1, column=0, sticky='w')
        self.ent_oc = ttk.Entry(tab_lim, width=10); self.ent_oc.grid(row=1, column=1, pady=2); self.ent_oc.insert(0, "5.0")
        ttk.Label(tab_lim, text="Sag Limit (%)").grid(row=2, column=0, sticky='w')
        self.ent_sag = ttk.Entry(tab_lim, width=10); self.ent_sag.grid(row=2, column=1, pady=2); self.ent_sag.insert(0, "10")
        ttk.Button(tab_lim, text="Update Limits", command=self.update_limits).grid(row=3, column=0, columnspan=2, pady=10)
        
        # PID Tuning Tab
        tab_pid = ttk.Frame(tabs, padding=10)
        tabs.add(tab_pid, text="PID")
        ttk.Label(tab_pid, text="Kp (Prop)").grid(row=0, column=0, sticky='w')
        self.ent_kp = ttk.Entry(tab_pid, width=10); self.ent_kp.grid(row=0, column=1, pady=2); self.ent_kp.insert(0, "25.0")
        ttk.Label(tab_pid, text="Ki (Integ)").grid(row=1, column=0, sticky='w')
        self.ent_ki = ttk.Entry(tab_pid, width=10); self.ent_ki.grid(row=1, column=1, pady=2); self.ent_ki.insert(0, "1.2")
        ttk.Label(tab_pid, text="Kd (Deriv)").grid(row=2, column=0, sticky='w')
        self.ent_kd = ttk.Entry(tab_pid, width=10); self.ent_kd.grid(row=2, column=1, pady=2); self.ent_kd.insert(0, "4.0")
        ttk.Button(tab_pid, text="Update PID Gains", command=self.update_pid).grid(row=3, column=0, columnspan=2, pady=10)
        
        # Calibration Tab
        tab_cal = ttk.Frame(tabs, padding=10)
        tabs.add(tab_cal, text="Calib")
        ttk.Button(tab_cal, text="Manual Calib Wizard", command=self.open_cal_wizard).pack(fill=tk.X)
        ttk.Button(tab_cal, text="BMP280 Self-Calib (Ref)", command=lambda: self.send_cmd("CAL_MODE")).pack(fill=tk.X, pady=5)
        
        ttk.Separator(tab_cal).pack(fill=tk.X, pady=10)
        
        ttk.Button(tab_cal, text="Capture Raw", command=self.capture_raw).pack(fill=tk.X, pady=5)
        self.lbl_raw_info = ttk.Label(tab_cal, text="Raw: --")
        self.lbl_raw_info.pack()

    def update_burst_est(self):
        try:
            self.estimated_burst = float(self.ent_burst_est.get())
            messagebox.showinfo("Success", f"Estimated burst pressure set to {self.estimated_burst} kPa")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid float for estimated burst.")

    def _init_plots(self, parent):
        # Setup Dual-Axis Plotting with Dark Theme
        self.fig = Figure(figsize=(5, 5), dpi=100, facecolor='#0f172a')
        
        # Axis 1: Pressure
        self.ax1 = self.fig.add_subplot(111)
        self.ax1.set_facecolor('#1e293b')
        self.ax1.set_title("Balloon Test Analytics (Real-Time)", color='#f8fafc', fontsize=12, fontweight='bold')
        self.ax1.set_xlabel("Time (s)", color='#94a3b8')
        self.ax1.set_ylabel("Pressure (kPa)", color='#f43f5e', fontweight='bold')
        self.line_p, = self.ax1.plot([], [], color='#f43f5e', linewidth=2, label='Pressure')
        self.ax1.tick_params(colors='#94a3b8')
        self.ax1.grid(True, color='#334155', linestyle='--', alpha=0.5)
        
        # Axis 2: dP/dt rate of change
        self.ax2 = self.ax1.twinx()
        self.ax2.set_ylabel("dP/dt rate of change (kPa/s)", color='#3b82f6', fontweight='bold')
        self.line_s, = self.ax2.plot([], [], color='#3b82f6', linestyle='--', alpha=0.8, label='dP/dt')
        self.ax2.tick_params(axis='y', colors='#3b82f6')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        ttk.Button(parent, text="Clear Plot & Reset Fatigue", command=self.clear_plot).pack()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            mode = self.conn_mode.get()
            if mode == "Serial":
                port = self.port_combo.get()
                if not port: return
                self.connection_mgr = SerialManager(port, BAUD_RATE, self.process_data)
            else:
                try:
                    port = int(self.ent_udp_port.get())
                except ValueError:
                    port = 8888
                self.connection_mgr = UDPManager(port, self.process_data)
                
            if self.connection_mgr.start():
                self.is_connected = True
                self.btn_connect.config(text="Disconnect")
                self.conn_mode.config(state="disabled")
                self.root.after(1500, lambda: self.send_cmd("CONNECT"))
        else:
            if self.connection_mgr:
                self.connection_mgr.stop()
                self.connection_mgr = None
            self.is_connected = False
            self.btn_connect.config(text="Connect")
            self.conn_mode.config(state="readonly")

    def toggle_recording(self):
        if not self.recording:
            file_path = filedialog.asksaveasfilename(defaultextension=".csv",filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
            if file_path:
                self.log_file = open(file_path, 'w', newline='')
                self.csv_writer = csv.writer(self.log_file)
                self.csv_writer.writerow(["Time", "Pressure", "PWM", "Volts", "Amps", "Mode", "RawP", "RawV", "RawI", "Slope", "Health"])
                self.recording = True
                self.btn_record.config(text="Stop Log")
        else:
            self.recording = False
            if self.log_file: self.log_file.close()
            self.btn_record.config(text="Start Log")

    def send_cmd(self, cmd):
        if self.connection_mgr and self.is_connected:
            self.connection_mgr.send(cmd)

    def on_pwm_change(self, val):
        self.send_cmd(f"SET_PWM {int(float(val))}")

    def on_valve_pwm_change(self, val):
        self.send_cmd(f"SET_VALVE_PWM {int(float(val))}")

    def start_smart(self):
        self.send_cmd(f"SET_RATIO {self.ent_ratio.get()}")
        self.send_cmd("START_SMART")

    def set_target_p(self):
        val = self.ent_target_p.get()
        self.send_cmd(f"SET_TARGET {val}")

    def update_limits(self):
        cmd = f"SET_LIMITS {self.ent_uv.get()} {self.ent_oc.get()} {self.ent_sag.get()}"
        self.send_cmd(cmd)

    def update_pid(self):
        cmd = f"SET_PID {self.ent_kp.get()} {self.ent_ki.get()} {self.ent_kd.get()}"
        self.send_cmd(cmd)

    def clear_plot(self):
        self.times.clear()
        self.pressures.clear()
        self.slopes.clear()
        self.slope_buffer.clear()
        self.fatigue_index = 0.0
        self.last_time = None
        self.root.after(0, self.update_plot)

    def open_cal_wizard(self):
        CalibrationWizard(self, self.send_cmd)

    def capture_raw(self):
        if self.raw_press == 0 and self.raw_volts == 0 and self.raw_amps == 0:
            msg = "Raw P: N/A V: N/A I: N/A"
        else:
            msg = f"Raw P:{self.raw_press} V:{self.raw_volts} I:{self.raw_amps}"
        self.lbl_raw_info.config(text=msg)
        print(msg)

    def process_data(self, line):
        if not line.startswith('$') or '*' not in line:
            return
        try:
            # Parse XOR NMEA Checksum
            parts_cs = line.split('*')
            if len(parts_cs) != 2:
                return
            
            payload, received_cs_str = parts_cs[0][1:], parts_cs[1]
            
            calculated_cs = 0
            for char in payload:
                calculated_cs ^= ord(char)
                
            calculated_cs_str = f"{calculated_cs:02X}"
            if calculated_cs_str != received_cs_str.upper():
                print(f"Checksum mismatch: Calc {calculated_cs_str} vs Recv {received_cs_str}")
                return
                
            parts = payload.split(',')
            if len(parts) >= 7 and parts[0] == "BIP":
                t = float(parts[1]) / 1000.0
                p = float(parts[2])
                pwm = int(parts[3])
                v = float(parts[4])
                i = float(parts[5])
                mode_code = int(parts[6])
                
                m_str = {0: "IDLE", 1: "MANUAL", 2: "SMART", 3: "BURST", 4: "CALIB", 5: "ERROR"}.get(mode_code, f"MODE_{mode_code}")
                
                if len(parts) >= 10:
                    self.raw_press = float(parts[7])
                    self.raw_volts = float(parts[8])
                    self.raw_amps = float(parts[9])

                # ----------------------------------------------------
                # Advanced Material Analysis Calculations (The Algorithms)
                # ----------------------------------------------------
                
                if self.last_time is not None and t < self.last_time - 1.0:
                    self.slope_buffer.clear()
                    self.fatigue_index = 0.0

                # 1. Linear Regression dP/dt Slope (savitzky-golay style)
                self.slope_buffer.append((t, p))
                slope = 0.0
                if len(self.slope_buffer) >= 5:
                    xs = [pt[0] for pt in self.slope_buffer]
                    ys = [pt[1] for pt in self.slope_buffer]
                    n = len(xs)
                    mean_x = sum(xs) / n
                    mean_y = sum(ys) / n
                    num = sum((xs[idx] - mean_x) * (ys[idx] - mean_y) for idx in range(n))
                    custom_den = sum((xs[idx] - mean_x) ** 2 for idx in range(n))
                    if custom_den > 1e-6:
                        slope = num / custom_den
                
                # 2. Fatigue Index & Lifetime Decay
                if self.last_time is not None:
                    dt = t - self.last_time
                    stress_ratio = max(0.0, p / self.estimated_burst)
                    if dt > 0 and dt < 1.0: # Ignore lag/jump hiccups
                        self.fatigue_index += (stress_ratio ** 3.0) * dt * 10.0
                self.last_time = t
                
                # Remaining Balloon Health Percentage
                health_pct = max(0.0, 100.0 - self.fatigue_index)
                
                # 3. Elastic Reserve Limit
                reserve_pct = max(0.0, ((self.estimated_burst - p) / self.estimated_burst) * 100.0)

                # Store plotting buffers
                self.times.append(t)
                self.pressures.append(p)
                self.slopes.append(slope)
                
                # Update GUI variables on main thread safely
                self.root.after(0, lambda: self._update_gui(p, pwm, v, i, m_str, reserve_pct, health_pct, slope))
                
                if self.recording and self.csv_writer:
                    self.csv_writer.writerow(parts[1:] + [f"{slope:.4f}", f"{health_pct:.2f}"])
                    self.log_file.flush()
                    
        except Exception as e:
            print(f"Parse error: {e}")

    def _update_gui(self, p, pwm, v, i, mode, reserve_pct, health_pct, slope):
        self.lbl_pressure.config(text=f"{p:.2f} kPa")
        self.lbl_pwm.config(text=f"{pwm}")
        self.lbl_volts.config(text=f"{v:.2f} V")
        self.lbl_amps.config(text=f"{i:.2f} A")
        self.lbl_power.config(text=f"{v*i:.2f} W")
        self.lbl_state.config(text=mode)
        
        # Update advanced labels
        self.lbl_reserve.config(text=f"{reserve_pct:.1f}%")
        self.lbl_health.config(text=f"{health_pct:.1f}%")
        self.lbl_slope.config(text=f"{slope:.2f} kPa/s")
        
        # Color code Health for warnings
        if health_pct > 50:
            self.lbl_health.config(foreground="green")
        elif health_pct > 20:
            self.lbl_health.config(foreground="orange")
        else:
            self.lbl_health.config(foreground="red")
            
        # Color code Reserve for warnings
        if reserve_pct < 15:
            self.lbl_reserve.config(foreground="red") # Near pop limit!
        elif reserve_pct < 35:
            self.lbl_reserve.config(foreground="orange") # Caution
        else:
            self.lbl_reserve.config(foreground="green") # Safe
            
        # Throttle plot updates to 30Hz (every 33ms) to prevent Tkinter freezing at high sample rates (2kSPS)
        now = time.time()
        if not hasattr(self, "last_plot_time"):
            self.last_plot_time = 0.0
        if now - self.last_plot_time > 0.033:
            self.last_plot_time = now
            self.update_plot()

    def update_plot(self):
        self.line_p.set_data(list(self.times), list(self.pressures))
        self.line_s.set_data(list(self.times), list(self.slopes))
        
        if self.times:
            x_min, x_max = min(self.times), max(self.times)
            self.ax1.set_xlim(x_min, x_max)
            
            p_min, p_max = min(self.pressures), max(self.pressures)
            self.ax1.set_ylim(min(0.0, p_min - 1), p_max + 2)
            
            s_min, s_max = min(self.slopes), max(self.slopes)
            self.ax2.set_ylim(s_min - 0.2, s_max + 0.2)
            
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    app = BalloonApp(root)
    root.mainloop()
