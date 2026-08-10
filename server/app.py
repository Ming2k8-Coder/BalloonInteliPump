import http.server
import socketserver
import urllib.parse
import json
import threading
import time
import csv
import queue
import socket
import collections
import os
import urllib.request
import numpy as np

# System Constants
BAUD_RATE = 115200
DEFAULT_UDP_PORT = 8888
HTTP_PORT = 8080
LOG_DIR = "logs"
MODEL_PATH = "logs/balloon_state_model.pkl"

# Ensure log directory exists
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# ----------------------------------------------------------------------
# Helper: Query Local LLM Server (Ollama / Llama.cpp)
# ----------------------------------------------------------------------
def query_local_llm(prompt):
    """
    Attempts to query Ollama (on default port 11434) with llama3 fallback,
    and falls back to a llama.cpp server (on default port 8080 /completion endpoint)
    if Ollama is offline.
    """
    # 1. Try Ollama (Standard local LLM engine)
    url_ollama = "http://localhost:11434/api/generate"
    data_ollama = {
        "model": "llama3",
        "prompt": prompt,
        "format": "json",
        "stream": False,
        "options": {
            "temperature": 0.2
        }
    }
    try:
        req = urllib.request.Request(url_ollama)
        req.add_header('Content-Type', 'application/json')
        jsondata = json.dumps(data_ollama).encode('utf-8')
        with urllib.request.urlopen(req, jsondata, timeout=3.5) as response:
            res = json.loads(response.read().decode('utf-8'))
            return res.get("response")
    except Exception:
        pass

    # 2. Fallback: Try llama.cpp server
    LLAMACPP_PORT = 8081
    url_cpp = f"http://localhost:{LLAMACPP_PORT}/completion"
    data_cpp = {
        "prompt": prompt,
        "n_predict": 128,
        "temperature": 0.2
    }
    try:
        req = urllib.request.Request(url_cpp)
        req.add_header('Content-Type', 'application/json')
        jsondata = json.dumps(data_cpp).encode('utf-8')
        with urllib.request.urlopen(req, jsondata, timeout=3.5) as response:
            res = json.loads(response.read().decode('utf-8'))
            return res.get("content")
    except Exception as e:
        print(f"⚠️ LLM DRIVER: Local LLM offline. Start Ollama or llama.cpp to enable. Error: {e}")
        return None

# ----------------------------------------------------------------------
# 1. TIME-SERIES PREDICTIVE ML: RECURSIVE LEAST SQUARES (RLS)
# ----------------------------------------------------------------------
class RLSPredictor:
    """
    Recursive Least Squares (RLS) Adaptive Time-Series Predictor.
    Dynamically models the balloon's viscoelastic expansion curve and
    forecasts pressure N steps into the future.
    """
    def __init__(self, order=5, forgetting_factor=0.98, forecast_steps=15):
        self.p = order                      # Number of past samples to look at
        self.lam = forgetting_factor        # Forgetting factor (lambda), weights recent history
        self.forecast_steps = forecast_steps # Steps to forecast ahead (e.g., 15 samples @ 100Hz = 150ms)
        
        # State variables
        self.w = np.zeros(order)            # AR coefficients (weights)
        self.P = np.eye(order) * 10.0       # Inverse covariance matrix
        self.x = collections.deque(maxlen=order) # Input buffer (past samples)
        
    def update(self, sample):
        self.x.appendleft(sample)
        if len(self.x) < self.p:
            return sample # Not enough data yet
        
        x_vector = np.array(list(self.x))
        
        # 1. Compute prediction error
        y_pred = np.dot(x_vector, self.w)
        error = sample - y_pred
        
        # 2. Compute gain vector
        Px = np.dot(self.P, x_vector)
        gain = Px / (self.lam + np.dot(x_vector, Px))
        
        # 3. Update weights
        self.w += gain * error
        
        # 4. Update inverse covariance matrix
        self.P = (self.P - np.outer(gain, np.dot(x_vector, self.P))) / self.lam
        
        return y_pred

    def forecast(self):
        """Forecast future pressure values based on current model weights."""
        if len(self.x) < self.p:
            return 0.0
            
        pred_buffer = list(self.x)
        forecast_val = 0.0
        
        # Iteratively forecast steps ahead
        for _ in range(self.forecast_steps):
            x_vec = np.array(pred_buffer[:self.p])
            forecast_val = np.dot(x_vec, self.w)
            pred_buffer.insert(0, forecast_val)
            
        if not np.isfinite(forecast_val):
            return 0.0
            
        return max(0.0, float(forecast_val))

# ----------------------------------------------------------------------
# 2. CONNECTION MANAGERS (SERIAL & UDP)
# ----------------------------------------------------------------------
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
            import serial
            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"Error opening serial port {self.port}: {e}")
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
                if not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get()
                    self.serial.write((cmd + '\n').encode())
                
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_callback(line)
                else:
                    time.sleep(0.005)
            except Exception as e:
                print(f"Serial read error: {e}")
                break

class UDPManager:
    def __init__(self, port, data_callback):
        self.port = port
        self.data_callback = data_callback
        self.socket = None
        self.running = False
        self.thread = None
        self.device_ip = None
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
            print(f"Error opening UDP socket on port {self.port}: {e}")
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
                if not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get()
                    if self.device_ip:
                        self.socket.sendto((cmd + '\n').encode(), (self.device_ip, self.port))
                
                try:
                    data, addr = self.socket.recvfrom(4096)
                    self.device_ip = addr[0]  # Dynamically capture ESP32 IP
                    packet_str = data.decode('utf-8', errors='ignore')
                    # Process lines split by newlines (double buffer batch support)
                    for line in packet_str.split('\n'):
                        line = line.strip()
                        if line:
                            self.data_callback(line)
                except socket.timeout:
                    pass
            except Exception as e:
                if self.running:
                    print(f"UDP Error: {e}")
                break

# ----------------------------------------------------------------------
# 3. DYNAMIC PLAY MODE DRIVER (Loonering Pleasure Custom Driver)
# ----------------------------------------------------------------------
class LoonerPleasureDriver:
    """
    Onsite Custom Play Mode Driver.
    Controls the physical pump and solenoid valve actuators dynamically in response
    to live machine learning states or a local LLM to optimize play pleasure.
    """
    def __init__(self, engine):
        self.engine = engine
        self.mode = "Interactive"   # Modes: "Disabled", "Interactive" (Give Mode), "Massage" (Pulse Mode), "LLM" (GPU-Driven)
        self.base_target = 20.0     # Baseline target play pressure in kPa
        self.valve_open = False
        self.last_state_change = 0.0
        self.pulse_phase = 0.0
        self.last_pulse_time = 0.0
        self.llm_busy = False
        
    def update(self, latest_data, predicted_state):
        now = time.time()
        
        if self.mode == "Interactive":
            # State 1: SQUEEZE (user bouncing/sitting/squeezing), State 3: DANGER (near pop limit)
            if predicted_state in [1, 3]:
                if not self.valve_open:
                    print("🎈 LOONER DRIVER [Interactive Give Mode]: Squeeze or Danger detected! Vending valve & halting pump.")
                    self.engine.send_cmd("VALVE_ON")
                    self.engine.send_cmd("STOP")
                    self.valve_open = True
                    self.last_state_change = now
            else: # STABLE (0) or YIELDING (2)
                # Wait 1.5 seconds after squeeze yields before closing valve and pumping back up
                # to prevent rapid target oscillation
                if self.valve_open and (now - self.last_state_change > 1.5):
                    print("🎈 LOONER DRIVER [Interactive Give Mode]: Normal play state restored. Closing valve, refilling.")
                    self.engine.send_cmd("VALVE_OFF")
                    self.engine.send_cmd("START_MANUAL")
                    self.engine.send_cmd(f"SET_TARGET {self.base_target:.1f}")
                    self.valve_open = False
                    self.last_state_change = now
                    
        elif self.mode == "Massage":
            # Ensure valve starts closed
            if self.valve_open:
                self.engine.send_cmd("VALVE_OFF")
                self.valve_open = False
                
            # Create a rhythmic sinus pressure wave (+/- 1.5 kPa) at 0.08 Hz (breathing massage effect)
            if now - self.last_pulse_time > 1.0:
                self.last_pulse_time = now
                self.pulse_phase += 0.5  # Increment phase angle
                target_offset = np.sin(self.pulse_phase) * 1.5
                new_target = max(5.0, self.base_target + target_offset)
                self.engine.send_cmd(f"SET_TARGET {new_target:.1f}")
                
        elif self.mode == "LLM":
            # Periodically request decisions from the local GPU LLM (every 3 seconds)
            if now - self.last_pulse_time > 3.0 and not self.llm_busy:
                self.last_pulse_time = now
                self.llm_busy = True
                threading.Thread(target=self._run_llm_query, daemon=True).start()
                
        elif self.mode == "Disabled":
            # Safety cleanup: seal the valve
            if self.valve_open:
                self.engine.send_cmd("VALVE_OFF")
                self.valve_open = False

    def _run_llm_query(self):
        try:
            hist = list(self.engine.history_buffer)
            if not hist:
                return
                
            p_list = [item["p"] for item in hist]
            p_mean = np.mean(p_list)
            p_var = np.var(p_list) if len(p_list) >= 2 else 0.0
            health = self.engine.latest_data.get("health", 100.0)
            ml_state = self.engine.latest_data.get("ml_state", "STABLE")
            
            prompt = (
                f"You are the BIP (Balloon Intelligent Pump) AI Controller. "
                f"Determine the next target and venting setting based on this looner play telemetry:\n"
                f"- Current Pressure: {p_mean:.1f} kPa\n"
                f"- Play Intensity (Variance): {p_var:.2f} (values > 10 indicate aggressive bouncing)\n"
                f"- Balloon Health: {health:.1f}%\n"
                f"- Detected Play State: {ml_state}\n\n"
                f"Decide the new target pressure (bounds: 5.0 to 35.0 kPa) and valve mode ('PUMP' or 'RELEASE'). "
                f"Respond ONLY with a valid JSON object matching this schema: "
                f'{{"target_pressure": float, "valve_mode": "PUMP"|"RELEASE", "rationale": "string"}}'
            )
            
            res_text = query_local_llm(prompt)
            if res_text:
                start_idx = res_text.find('{')
                end_idx = res_text.rfind('}') + 1
                if start_idx >= 0 and end_idx > 0:
                    json_str = res_text[start_idx:end_idx]
                    decision = json.loads(json_str)
                    
                    target = float(decision.get("target_pressure", self.base_target))
                    valve = decision.get("valve_mode", "PUMP")
                    rationale = decision.get("rationale", "")
                    
                    print(f"🤖 LLM DECISION: Target = {target:.1f} kPa | Valve = {valve} | Rationale: {rationale}")
                    
                    # Execute decision
                    if valve == "RELEASE":
                        if not self.valve_open:
                            self.engine.send_cmd("VALVE_ON")
                            self.engine.send_cmd("STOP")
                            self.valve_open = True
                    else:
                        if self.valve_open:
                            self.engine.send_cmd("VALVE_OFF")
                            self.engine.send_cmd("START_MANUAL")
                            self.valve_open = False
                        self.engine.send_cmd(f"SET_TARGET {target:.1f}")
        except Exception as e:
            print(f"⚠️ LLM DRIVER: Error parsing LLM response: {e}")
        finally:
            self.llm_busy = False

# ----------------------------------------------------------------------
# 4. GLOBAL TELEMETRY ENGINE & HIGH-PRECISION LOGGER
# ----------------------------------------------------------------------
class TelemetryEngine:
    def __init__(self):
        self.connection = None
        self.conn_type = None
        self.is_connected = False
        self.latest_data = {}
        self.clients = []
        self.client_lock = threading.Lock()
        
        # High Precision Logging
        self.logging_active = False
        self.log_file = None
        self.csv_writer = None
        
        # DSP / ML Algorithms
        self.predictor = RLSPredictor(order=6, forgetting_factor=0.985, forecast_steps=20)
        self.slope_buffer = collections.deque(maxlen=10)
        self.fatigue_index = 0.0
        self.estimated_burst = 50.0 # Default kPa
        self.last_time = None
        
        # Online Feature Buffer (Stores raw history for rolling DSP feature calculation)
        self.history_buffer = collections.deque(maxlen=100)
        
        # Instantiate Pleasure Custom Driver
        self.pleasure_driver = LoonerPleasureDriver(self)
        
        # Load trained scikit-learn classifier model if available
        self.ml_model = None
        if os.path.exists(MODEL_PATH):
            try:
                import joblib
                self.ml_model = joblib.load(MODEL_PATH)
                print(f"🧠 ML MODEL: Successfully loaded RF classifier from {MODEL_PATH}")
            except Exception as e:
                print(f"⚠️ ML MODEL: Failed to load RF model: {e}")
        else:
            print("⚠️ ML MODEL: No pre-trained model file found. Classification runs in heuristic fallback.")
        
    def start_connection(self, conn_type, port_or_device):
        self.stop_connection()
        self.conn_type = conn_type
        
        if conn_type == "Serial":
            self.connection = SerialManager(port_or_device, BAUD_RATE, self.process_line)
        else:
            try:
                port = int(port_or_device)
            except ValueError:
                port = DEFAULT_UDP_PORT
            self.connection = UDPManager(port, self.process_line)
            
        if self.connection.start():
            self.is_connected = True
            threading.Timer(1.5, lambda: self.send_cmd("CONNECT")).start()
            return True
        return False
        
    def stop_connection(self):
        if self.connection:
            self.connection.stop()
            self.connection = None
        self.is_connected = False
        self.conn_type = None
        self.fatigue_index = 0.0
        self.last_time = None
        
    def send_cmd(self, cmd):
        if self.connection and self.is_connected:
            self.connection.send(cmd)
            
    def start_logging(self, filename):
        self.stop_logging()
        filepath = os.path.join(LOG_DIR, filename)
        self.log_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file, doublequote=True)
        # Full precision headers
        self.csv_writer.writerow([
            "Timestamp_Unix", "Time_Device_s", "Pressure_kPa", "PWM", 
            "Voltage_V", "Current_A", "Mode", "RawP", "RawV", "RawI",
            "Slope_kPa_s", "Forecasted_P_150ms", "Fatigue_Index", "Health_Pct", "Reserve_Pct"
        ])
        self.logging_active = True
        
    def stop_logging(self):
        self.logging_active = False
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.csv_writer = None

    def process_line(self, line):
        if not line.startswith('$') or '*' not in line:
            return
            
        try:
            parts_cs = line.split('*')
            if len(parts_cs) != 2:
                return
                
            payload, received_cs = parts_cs[0][1:], parts_cs[1]
            
            # Verify Checksum
            calculated_cs = 0
            for char in payload:
                calculated_cs ^= ord(char)
                
            if f"{calculated_cs:02X}" != received_cs.upper():
                return
                
            parts = payload.split(',')
            if len(parts) >= 7 and parts[0] == "BIP":
                t_device = float(parts[1]) / 1000.0
                p = float(parts[2])
                pwm = int(parts[3])
                v = float(parts[4])
                i = float(parts[5])
                mode_code = int(parts[6])
                m_str = {0: "IDLE", 1: "MANUAL", 2: "SMART", 3: "BURST", 4: "CALIB", 5: "ERROR"}.get(mode_code, f"MODE_{mode_code}")
                
                raw_p, raw_v, raw_i = 0.0, 0.0, 0.0
                if len(parts) >= 10:
                    raw_p = float(parts[7])
                    raw_v = float(parts[8])
                    raw_i = float(parts[9])

                # ----------------------------------------------------
                # DSP & PREDICTIVE ML ALGORITHMS
                # ----------------------------------------------------
                
                # 1. RLS Adaptive Forecasting
                self.predictor.update(p)
                forecasted_p = self.predictor.forecast()
                
                # Predictive Cut-off algorithm (Safety First!)
                if forecasted_p >= self.estimated_burst * 0.95 and mode_code in [1, 2, 3]:
                    print(f"⚠️ PREDICTIVE BURST SAFETY ACTIVE: Forecasted {forecasted_p:.2f} kPa >= Limit {self.estimated_burst * 0.95:.2f} kPa. Halting pump!")
                    self.send_cmd("STOP")
                
                # 2. Savitzky-Golay / Regression dP/dt Slope
                self.slope_buffer.append((t_device, p))
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

                # 3. Cubic Fatigue Index & Health decay
                if self.last_time is not None:
                    dt = t_device - self.last_time
                    stress_ratio = max(0.0, p / self.estimated_burst)
                    if dt > 0 and dt < 1.0:
                        self.fatigue_index += (stress_ratio ** 3.0) * dt * 10.0
                self.last_time = t_device
                
                health_pct = max(0.0, 100.0 - self.fatigue_index)
                reserve_pct = max(0.0, ((self.estimated_burst - p) / self.estimated_burst) * 100.0)

                # 4. Advanced Material Analytics (Online Rolling Features)
                self.history_buffer.append({
                    "p": p,
                    "v": v,
                    "i": i,
                    "pwm": pwm,
                    "slope": slope
                })
                
                # DSP calculations matching training columns
                hist_p = [item["p"] for item in self.history_buffer]
                p_var = float(np.var(hist_p[-30:])) if len(hist_p) >= 2 else 0.0
                p_integral = float(sum(hist_p[-50:])) * 0.01 if len(hist_p) >= 1 else 0.0
                power = v * i
                power_ratio = power / p if p > 2.0 else 0.0
                creep = -slope if pwm == 0 else 0.0

                # 5. Live State Classification (Scikit-Learn Random Forest or Heuristic Fallback)
                predicted_state = 0 # Default: STABLE (0)
                if self.ml_model is not None:
                    try:
                        feature_vector = np.array([[
                            p, slope, v, i, health_pct,
                            p_var, p_integral, power, power_ratio, creep
                        ]])
                        predicted_state = int(self.ml_model.predict(feature_vector)[0])
                    except Exception as e:
                        pass
                else:
                    # Fallback Heuristics
                    if health_pct <= 20.0 or p >= 45.0:
                        predicted_state = 3 # DANGER
                    elif mode_code == 2 and slope <= 0.15 and p > 15.0:
                        predicted_state = 2 # YIELDING
                    elif slope > 1.5 and mode_code == 1:
                        predicted_state = 1 # SQUEEZE
                
                ml_state_str = {0: "STABLE", 1: "SQUEEZE", 2: "YIELDING", 3: "DANGER"}.get(predicted_state, "STABLE")

                # Store latest data points
                self.latest_data = {
                    "timestamp": time.time(),
                    "time": t_device,
                    "pressure": p,
                    "pwm": pwm,
                    "voltage": v,
                    "current": i,
                    "mode": m_str,
                    "raw_p": raw_p,
                    "raw_v": raw_v,
                    "raw_i": raw_i,
                    "slope": slope,
                    "forecast": forecasted_p,
                    "health": health_pct,
                    "reserve": reserve_pct,
                    "ml_state": ml_state_str
                }
                
                # 6. Execute Looner Pleasure Custom Driver rules
                self.pleasure_driver.update(self.latest_data, predicted_state)
                
                # Full Precision CSV logging
                if self.logging_active and self.csv_writer:
                    self.csv_writer.writerow([
                        f"{time.time():.6f}", f"{t_device:.3f}", f"{p:.4f}", pwm,
                        f"{v:.4f}", f"{i:.4f}", m_str, f"{raw_p:.2f}", f"{raw_v:.2f}", f"{raw_i:.2f}",
                        f"{slope:.4f}", f"{forecasted_p:.4f}", f"{self.fatigue_index:.6f}", 
                        f"{health_pct:.2f}", f"{reserve_pct:.2f}"
                    ])
                    self.log_file.flush()

                # Broadcast to connected browser clients (SSE)
                self.broadcast_data(self.latest_data)

        except Exception as e:
            print(f"Error decoding NMEA packet: {e}")

    def add_client(self, client):
        with self.client_lock:
            self.clients.append(client)

    def remove_client(self, client):
        with self.client_lock:
            if client in self.clients:
                self.clients.remove(client)

    def broadcast_data(self, data):
        data_str = f"data: {json.dumps(data)}\n\n"
        with self.client_lock:
            for client in list(self.clients):
                client.put(data_str)

# Global Telemetry Engine Instance
engine = TelemetryEngine()

# ----------------------------------------------------------------------
# 5. HTTP SERVER & SSE ENDPOINT (WebGUI)
# ----------------------------------------------------------------------
class WebServerHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Suppress command line log clutter for telemetry endpoints

    def do_GET(self):
        parsed_url = urllib.parse.urlparse(self.path)
        
        # 1. Server Sent Events Endpoint (Low-latency streaming)
        if parsed_url.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            
            q = queue.Queue()
            engine.add_client(q)
            
            try:
                while True:
                    try:
                        # Grab data packet from queue and send
                        data_line = q.get(timeout=1.0)
                        self.wfile.write(data_line.encode("utf-8"))
                        self.wfile.flush()
                    except queue.Empty:
                        # Keep Alive Ping
                        self.wfile.write(":\n\n".encode("utf-8"))
                        self.wfile.flush()
            except Exception as e:
                # Connection dropped
                pass
            finally:
                engine.remove_client(q)
            return

        # 2. Get API Status
        elif parsed_url.path == "/api/status":
            self.send_json({
                "connected": engine.is_connected,
                "conn_type": engine.conn_type,
                "logging": engine.logging_active,
                "estimated_burst": engine.estimated_burst,
                "driver_mode": engine.pleasure_driver.mode,
                "base_target": engine.pleasure_driver.base_target
            })
            return

        # 3. Serve main GUI Dashboard
        elif parsed_url.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_GUI.encode("utf-8"))
            return
            
        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed_url = urllib.parse.urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length).decode('utf-8')
        
        try:
            params = json.loads(post_data)
        except json.JSONDecodeError:
            self.send_response(400)
            self.end_headers()
            return

        if parsed_url.path == "/api/connect":
            conn_type = params.get("type", "WiFi")
            port_val = params.get("port", "8888")
            success = engine.start_connection(conn_type, port_val)
            self.send_json({"success": success})
            
        elif parsed_url.path == "/api/disconnect":
            engine.stop_connection()
            self.send_json({"success": True})
            
        elif parsed_url.path == "/api/command":
            cmd = params.get("cmd", "")
            if cmd:
                engine.send_cmd(cmd)
                self.send_json({"success": True})
            else:
                self.send_json({"success": False, "error": "Empty command"})
                
        elif parsed_url.path == "/api/logging":
            action = params.get("action", "")
            if action == "start":
                filename = params.get("filename", f"bip_test_{int(time.time())}.csv")
                engine.start_logging(filename)
                self.send_json({"success": True, "filename": filename})
            else:
                engine.stop_logging()
                self.send_json({"success": True})
                
        elif parsed_url.path == "/api/config":
            burst = params.get("estimated_burst")
            if burst is not None:
                engine.estimated_burst = float(burst)
                self.send_json({"success": True})
            else:
                self.send_json({"success": False})
                
        elif parsed_url.path == "/api/driver":
            mode = params.get("mode")
            base_target = params.get("base_target")
            if mode is not None:
                engine.pleasure_driver.mode = mode
            if base_target is not None:
                engine.pleasure_driver.base_target = float(base_target)
            self.send_json({
                "success": True,
                "mode": engine.pleasure_driver.mode,
                "base_target": engine.pleasure_driver.base_target
            })
        elif parsed_url.path == "/api/pid":
            kp = params.get("kp", 25.0)
            ki = params.get("ki", 1.2)
            kd = params.get("kd", 4.0)
            engine.send_cmd(f"SET_PID {kp} {ki} {kd}")
            self.send_json({"success": True, "kp": kp, "ki": ki, "kd": kd})
        else:
            self.send_response(404)
            self.end_headers()

    def send_json(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode("utf-8"))

# ----------------------------------------------------------------------
# 6. EMBEDDED WEB GUI CODE (Pure HTML / CSS / JS)
# ----------------------------------------------------------------------
HTML_GUI = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BIP - Intelligent Telemetry & Controller</title>
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&family=JetBrains+Mono:wght@400;600;700&display=swap" rel="stylesheet">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        :root {
            --bg-color: #070a12;
            --card-bg: rgba(15, 23, 42, 0.65);
            --card-border: rgba(255, 255, 255, 0.08);
            --card-hover: rgba(30, 41, 59, 0.8);
            --text-primary: #f8fafc;
            --text-secondary: #94a3b8;
            --accent-indigo: #6366f1;
            --accent-blue: #3b82f6;
            --accent-emerald: #10b981;
            --accent-amber: #f59e0b;
            --accent-rose: #f43f5e;
            --font-main: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
            --font-mono: 'JetBrains Mono', monospace;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            background-color: var(--bg-color);
            background-image: 
                radial-gradient(circle at 10% 10%, rgba(99, 102, 241, 0.1) 0%, transparent 35%),
                radial-gradient(circle at 90% 90%, rgba(59, 130, 246, 0.08) 0%, transparent 35%);
            color: var(--text-primary);
            font-family: var(--font-main);
            padding: 24px;
            min-height: 100vh;
        }

        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 20px;
            border-bottom: 1px solid var(--card-border);
            margin-bottom: 24px;
        }

        .header-title-group h1 {
            font-size: 1.6rem;
            font-weight: 800;
            letter-spacing: -0.02em;
            background: linear-gradient(135deg, #a5b4fc 0%, #6366f1 50%, #3b82f6 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .header-title-group p {
            font-size: 0.825rem;
            color: var(--text-secondary);
            margin-top: 2px;
        }

        .status-pill {
            display: flex;
            align-items: center;
            gap: 8px;
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid var(--card-border);
            padding: 6px 14px;
            border-radius: 9999px;
            font-size: 0.85rem;
            font-weight: 500;
        }

        .status-dot {
            width: 9px;
            height: 9px;
            border-radius: 50%;
            background: var(--accent-rose);
            box-shadow: 0 0 10px var(--accent-rose);
        }

        .status-dot.active {
            background: var(--accent-emerald);
            box-shadow: 0 0 12px var(--accent-emerald);
            animation: pulse-glow 2s infinite;
        }

        @keyframes pulse-glow {
            0%, 100% { transform: scale(1); opacity: 1; }
            50% { transform: scale(1.2); opacity: 0.8; }
        }

        .dashboard-grid {
            display: grid;
            grid-template-columns: 360px 1fr;
            gap: 24px;
        }

        @media (max-width: 980px) {
            .dashboard-grid {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background: var(--card-bg);
            border: 1px solid var(--card-border);
            border-radius: 16px;
            padding: 20px;
            margin-bottom: 20px;
            backdrop-filter: blur(16px);
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.3);
            transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .card:hover {
            border-color: rgba(255, 255, 255, 0.12);
        }

        .card-title {
            font-size: 0.8rem;
            font-weight: 700;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.08em;
            margin-bottom: 14px;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        .form-group {
            display: flex;
            flex-direction: column;
            gap: 6px;
            margin-bottom: 14px;
        }

        label {
            font-size: 0.8rem;
            font-weight: 500;
            color: var(--text-secondary);
        }

        select, input {
            background: rgba(15, 23, 42, 0.8);
            border: 1px solid var(--card-border);
            color: var(--text-primary);
            padding: 10px 12px;
            border-radius: 8px;
            font-size: 0.875rem;
            font-family: var(--font-main);
            width: 100%;
            outline: none;
            transition: all 0.2s ease;
        }

        select:focus, input:focus {
            border-color: var(--accent-indigo);
            box-shadow: 0 0 0 3px rgba(99, 102, 241, 0.2);
            background: rgba(30, 41, 59, 0.9);
        }

        button {
            cursor: pointer;
            font-family: var(--font-main);
            font-size: 0.875rem;
            font-weight: 600;
            background: linear-gradient(135deg, var(--accent-indigo), var(--accent-blue));
            color: #ffffff;
            border: none;
            padding: 10px 16px;
            border-radius: 8px;
            transition: all 0.2s ease;
            box-shadow: 0 4px 12px rgba(99, 102, 241, 0.25);
            width: 100%;
        }

        button:hover {
            transform: translateY(-1px);
            box-shadow: 0 6px 16px rgba(99, 102, 241, 0.35);
        }

        button:active {
            transform: translateY(0);
        }

        button.btn-danger {
            background: linear-gradient(135deg, #e11d48, #f43f5e);
            box-shadow: 0 4px 12px rgba(244, 63, 94, 0.25);
        }

        button.btn-danger:hover {
            box-shadow: 0 6px 16px rgba(244, 63, 94, 0.35);
        }

        button.btn-success {
            background: linear-gradient(135deg, #059669, #10b981);
            box-shadow: 0 4px 12px rgba(16, 185, 129, 0.25);
        }

        button.btn-secondary {
            background: rgba(255, 255, 255, 0.06);
            color: var(--text-primary);
            border: 1px solid var(--card-border);
            box-shadow: none;
        }

        button.btn-secondary:hover {
            background: rgba(255, 255, 255, 0.1);
        }

        .range-slider-container {
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .range-slider-container input[type="range"] {
            flex: 1;
            accent-color: var(--accent-indigo);
        }

        .gauge-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
            gap: 14px;
        }

        .gauge-box {
            background: rgba(15, 23, 42, 0.4);
            border: 1px solid var(--card-border);
            border-radius: 12px;
            padding: 14px;
            text-align: center;
            transition: all 0.2s ease;
        }

        .gauge-box:hover {
            background: rgba(30, 41, 59, 0.4);
            border-color: rgba(255, 255, 255, 0.12);
        }

        .gauge-name {
            font-size: 0.725rem;
            font-weight: 600;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.05em;
            margin-bottom: 6px;
        }

        .gauge-val {
            font-family: var(--font-mono);
            font-size: 1.35rem;
            font-weight: 700;
            letter-spacing: -0.02em;
        }

        .ml-badge {
            display: inline-block;
            padding: 4px 10px;
            border-radius: 6px;
            font-size: 0.85rem;
            font-weight: 700;
            letter-spacing: 0.05em;
            text-transform: uppercase;
        }

        .health-bar-container {
            background: rgba(255, 255, 255, 0.06);
            border-radius: 6px;
            height: 7px;
            margin-top: 10px;
            overflow: hidden;
            width: 100%;
        }

        .health-bar {
            background: var(--accent-emerald);
            height: 100%;
            width: 100%;
            border-radius: 6px;
            transition: width 0.3s cubic-bezier(0.4, 0, 0.2, 1), background 0.3s ease;
        }

        .chart-container {
            position: relative;
            height: 420px;
            width: 100%;
        }
    </style>
</head>
<body>

    <header>
        <div class="header-title-group">
            <h1>BIP Intelligent Telemetry</h1>
            <p>High-Precision Latex Dynamics & Machine Learning Dashboard</p>
        </div>
        <div class="status-pill">
            <span class="status-dot" id="system-dot"></span>
            <span id="system-status-text">Disconnected</span>
        </div>
    </header>

    <div class="dashboard-grid">
        <!-- Sidebar Controls -->
        <div>
            <!-- Connection Control Card -->
            <div class="card">
                <div class="card-title">Device Connection</div>
                <div class="form-group">
                    <label>Interface Mode</label>
                    <select id="conn-type" onchange="onConnTypeChange()">
                        <option value="WiFi">WiFi (2kSPS UDP)</option>
                        <option value="Serial">Serial (UART COM)</option>
                    </select>
                </div>
                <div class="form-group" id="port-group">
                    <label id="port-label">UDP Listening Port</label>
                    <input type="text" id="conn-port" value="8888">
                </div>
                <button id="btn-connect" onclick="toggleConnection()">Connect Device</button>
            </div>

            <!-- Looner Play Custom Pleasure Driver Card -->
            <div class="card">
                <div class="card-title">Looner Pleasure Driver</div>
                <div class="form-group">
                    <label>Driver Operating Mode</label>
                    <select id="driver-mode" onchange="updateDriver()">
                        <option value="Disabled">Disabled</option>
                        <option value="Interactive">Interactive Give Mode</option>
                        <option value="Massage">Massage Pulse Mode</option>
                        <option value="LLM">Local LLM Driver (RX 5600XT)</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>Target Pressure (kPa)</label>
                    <input type="number" step="0.5" id="driver-base-target" value="20.0" onchange="updateDriver()">
                </div>
            </div>

            <!-- Solenoid Valve Control Card -->
            <div class="card">
                <div class="card-title">Solenoid Relief Valve</div>
                <div style="display: flex; gap: 8px; margin-bottom: 12px;">
                    <button class="btn-secondary" onclick="sendCmd('VALVE_ON')">Open (100%)</button>
                    <button class="btn-secondary" onclick="sendCmd('VALVE_OFF')">Close (0%)</button>
                </div>
                <div class="form-group">
                    <label>Valve Duty Cycle PWM (0-255)</label>
                    <div class="range-slider-container">
                        <input type="range" min="0" max="255" id="valve-pwm-slider" value="0" oninput="onValveSliderChange(this.value)">
                        <span id="valve-pwm-val" style="font-family: var(--font-mono); font-size: 0.85rem; width: 35px;">0</span>
                    </div>
                </div>
            </div>

            <!-- PID Controller Parameters -->
            <div class="card">
                <div class="card-title">PID Closed-Loop Tuning</div>
                <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-bottom: 12px;">
                    <div class="form-group">
                        <label>Kp</label>
                        <input type="number" step="0.1" id="pid-kp" value="25.0">
                    </div>
                    <div class="form-group">
                        <label>Ki</label>
                        <input type="number" step="0.1" id="pid-ki" value="1.2">
                    </div>
                    <div class="form-group">
                        <label>Kd</label>
                        <input type="number" step="0.1" id="pid-kd" value="4.0">
                    </div>
                </div>
                <button class="btn-secondary" onclick="updatePID()">Apply Gains</button>
            </div>

            <!-- Material Safety Parameters -->
            <div class="card">
                <div class="card-title">Lifespan Configuration</div>
                <div class="form-group">
                    <label>Est. Burst Limit (kPa)</label>
                    <input type="number" step="0.1" id="config-burst" value="50.0">
                </div>
                <button class="btn-secondary" onclick="updateConfig()">Apply Burst Limit</button>
            </div>

            <!-- Logging Card -->
            <div class="card">
                <div class="card-title">Full Precision Data Logger</div>
                <div class="form-group">
                    <label>Log Filename (.csv)</label>
                    <input type="text" id="log-filename" placeholder="auto-generated.csv">
                </div>
                <button class="btn-secondary" id="btn-log" onclick="toggleLogging()">Start Logging</button>
            </div>

            <!-- Operational Command Center -->
            <div class="card">
                <div class="card-title">Command Center</div>
                <div style="display: flex; gap: 8px; margin-bottom: 12px;">
                    <button class="btn-success" onclick="sendCmd('START_MANUAL')">Manual</button>
                    <button class="btn-secondary" onclick="sendCmd('START_SMART')">Smart</button>
                    <button class="btn-secondary" onclick="sendCmd('START_BURST')">Burst</button>
                </div>
                <div class="form-group" style="margin-bottom: 12px;">
                    <label>Manual Target (kPa)</label>
                    <div style="display: flex; gap: 8px;">
                        <input type="number" id="manual-target" value="20.0" style="flex: 1;">
                        <button onclick="setManualTarget()" style="width: auto;">Set</button>
                    </div>
                </div>
                <button class="btn-danger" style="padding: 12px; font-size: 1rem;" onclick="sendCmd('STOP')">EMERGENCY HALT</button>
            </div>
        </div>

        <!-- Main Chart and Gauges -->
        <div>
            <!-- Real-Time Gauges Card -->
            <div class="card">
                <div class="card-title">Real-Time Telemetry & Predictive Safety</div>
                <div class="gauge-grid">
                    <div class="gauge-box">
                        <div class="gauge-name">Pressure</div>
                        <div class="gauge-val" id="val-press" style="color: var(--accent-rose)">0.00 kPa</div>
                    </div>
                    <div class="gauge-box">
                        <div class="gauge-name">RLS Forecast (150ms)</div>
                        <div class="gauge-val" id="val-forecast" style="color: var(--accent-amber)">0.00 kPa</div>
                    </div>
                    <div class="gauge-box">
                        <div class="gauge-name">Rate dP/dt</div>
                        <div class="gauge-val" id="val-slope" style="color: var(--accent-blue)">0.00 kPa/s</div>
                    </div>
                    <div class="gauge-box">
                        <div class="gauge-name">ML Material State</div>
                        <div style="margin-top: 4px;">
                            <span class="ml-badge" id="val-ml-state" style="background: rgba(16, 185, 129, 0.15); border: 1px solid #10b981; color: #10b981;">STABLE</span>
                        </div>
                    </div>
                </div>
                
                <div class="gauge-grid" style="margin-top: 14px;">
                    <div class="gauge-box">
                        <div class="gauge-name">Elastic Reserve</div>
                        <div class="gauge-val" id="val-reserve" style="color: var(--accent-emerald);">100.0%</div>
                    </div>
                    <div class="gauge-box">
                        <div class="gauge-name">Balloon Health</div>
                        <div class="gauge-val" id="val-health">100.0%</div>
                        <div class="health-bar-container">
                            <div class="health-bar" id="health-bar"></div>
                        </div>
                    </div>
                    <div class="gauge-box">
                        <div class="gauge-name">Power Draw</div>
                        <div class="gauge-val" id="val-power" style="color: var(--accent-indigo);">0.00 W</div>
                    </div>
                    <div class="gauge-box">
                        <div class="gauge-name">Core Voltage</div>
                        <div class="gauge-val" id="val-volts" style="color: var(--text-primary);">0.00 V</div>
                    </div>
                </div>
            </div>

            <!-- Chart Card -->
            <div class="card">
                <div class="card-title">Dynamic Time-Series Analytics</div>
                <div class="chart-container">
                    <canvas id="telemetryChart"></canvas>
                </div>
            </div>
        </div>
    </div>

    <script>
        let chart;
        let eventSource = null;
        let isConnected = false;
        let isLogging = false;

        // Initialize Chart.js
        function initChart() {
            const ctx = document.getElementById('telemetryChart').getContext('2d');
            chart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Measured Pressure (kPa)',
                            data: [],
                            borderColor: '#f43f5e',
                            backgroundColor: 'rgba(244, 63, 94, 0.12)',
                            fill: true,
                            borderWidth: 2,
                            yAxisID: 'y',
                            tension: 0.15,
                            pointRadius: 0
                        },
                        {
                            label: 'RLS Forecasted Pressure (150ms)',
                            data: [],
                            borderColor: '#f59e0b',
                            borderWidth: 1.5,
                            borderDash: [5, 5],
                            fill: false,
                            yAxisID: 'y',
                            tension: 0.15,
                            pointRadius: 0
                        },
                        {
                            label: 'dP/dt Slope (kPa/s)',
                            data: [],
                            borderColor: '#3b82f6',
                            borderWidth: 1.5,
                            fill: false,
                            yAxisID: 'y1',
                            tension: 0.15,
                            pointRadius: 0
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        x: {
                            grid: { color: 'rgba(255, 255, 255, 0.04)' },
                            ticks: { color: 'rgba(255, 255, 255, 0.5)', font: { family: 'JetBrains Mono', size: 10 } }
                        },
                        y: {
                            type: 'linear',
                            position: 'left',
                            grid: { color: 'rgba(255, 255, 255, 0.04)' },
                            ticks: { color: '#f43f5e', font: { family: 'JetBrains Mono', size: 11 } },
                            title: { display: true, text: 'Pressure (kPa)', color: '#f43f5e', font: { family: 'Inter', size: 12, weight: '600' } }
                        },
                        y1: {
                            type: 'linear',
                            position: 'right',
                            grid: { drawOnChartArea: false },
                            ticks: { color: '#3b82f6', font: { family: 'JetBrains Mono', size: 11 } },
                            title: { display: true, text: 'dP/dt (kPa/s)', color: '#3b82f6', font: { family: 'Inter', size: 12, weight: '600' } }
                        }
                    },
                    plugins: {
                        legend: { labels: { color: 'rgba(255, 255, 255, 0.8)', font: { family: 'Inter', size: 12 } } }
                    }
                }
            });
        }

        // Load Status
        async function loadStatus() {
            try {
                const res = await fetch('/api/status');
                const data = await res.json();
                
                isConnected = data.connected;
                isLogging = data.logging;
                document.getElementById('config-burst').value = data.estimated_burst;
                document.getElementById('driver-mode').value = data.driver_mode;
                document.getElementById('driver-base-target').value = data.base_target;

                if (isConnected) {
                    setConnectedUI(true, data.conn_type);
                    startEventSource();
                } else {
                    setConnectedUI(false);
                }

                if (isLogging) {
                    setLoggingUI(true);
                } else {
                    setLoggingUI(false);
                }
            } catch (e) {
                console.error("Error fetching status", e);
            }
        }

        function setConnectedUI(connected, type = "") {
            const btn = document.getElementById('btn-connect');
            const dot = document.getElementById('system-dot');
            const text = document.getElementById('system-status-text');
            
            if (connected) {
                btn.innerText = "Disconnect Device";
                btn.className = "btn-danger";
                dot.className = "status-dot active";
                text.innerText = `Connected (${type})`;
            } else {
                btn.innerText = "Connect Device";
                btn.className = "";
                dot.className = "status-dot";
                text.innerText = "Disconnected";
            }
        }

        function setLoggingUI(active, filename = "") {
            const btn = document.getElementById('btn-log');
            if (active) {
                btn.innerText = "Stop Logging";
                btn.className = "btn-danger";
            } else {
                btn.innerText = "Start Logging";
                btn.className = "btn-secondary";
            }
        }

        function onConnTypeChange() {
            const type = document.getElementById('conn-type').value;
            const label = document.getElementById('port-label');
            const portInput = document.getElementById('conn-port');
            if (type === "Serial") {
                label.innerText = "COM Port";
                portInput.value = "COM3";
            } else {
                label.innerText = "UDP Listening Port";
                portInput.value = "8888";
            }
        }

        function onValveSliderChange(val) {
            document.getElementById('valve-pwm-val').innerText = val;
            sendCmd(`SET_VALVE_PWM ${val}`);
        }

        async function toggleConnection() {
            if (!isConnected) {
                const type = document.getElementById('conn-type').value;
                const port = document.getElementById('conn-port').value;
                
                const res = await fetch('/api/connect', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ type, port })
                });
                const data = await res.json();
                if (data.success) {
                    isConnected = true;
                    setConnectedUI(true, type);
                    startEventSource();
                } else {
                    alert("Failed to connect!");
                }
            } else {
                await fetch('/api/disconnect', { method: 'POST' });
                isConnected = false;
                setConnectedUI(false);
                stopEventSource();
            }
        }

        async function toggleLogging() {
            if (!isLogging) {
                const filename = document.getElementById('log-filename').value;
                const res = await fetch('/api/logging', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'start', filename })
                });
                const data = await res.json();
                if (data.success) {
                    isLogging = true;
                    setLoggingUI(true);
                }
            } else {
                await fetch('/api/logging', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'stop' })
                });
                isLogging = false;
                setLoggingUI(false);
            }
        }

        async function updateConfig() {
            const estimated_burst = parseFloat(document.getElementById('config-burst').value);
            const res = await fetch('/api/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ estimated_burst })
            });
            const data = await res.json();
            if (data.success) {
                alert("Burst threshold configured!");
            }
        }

        async function updatePID() {
            const kp = parseFloat(document.getElementById('pid-kp').value);
            const ki = parseFloat(document.getElementById('pid-ki').value);
            const kd = parseFloat(document.getElementById('pid-kd').value);
            const res = await fetch('/api/pid', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ kp, ki, kd })
            });
            const data = await res.json();
            if (data.success) {
                alert(`PID Gains updated to Kp=${kp}, Ki=${ki}, Kd=${kd}`);
            }
        }

        async function updateDriver() {
            const mode = document.getElementById('driver-mode').value;
            const base_target = parseFloat(document.getElementById('driver-base-target').value);
            await fetch('/api/driver', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode, base_target })
            });
        }

        async function sendCmd(cmd) {
            await fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd })
            });
        }

        function setManualTarget() {
            const target = document.getElementById('manual-target').value;
            sendCmd(`SET_TARGET ${target}`);
        }

        // Start SSE Listening stream
        function startEventSource() {
            stopEventSource();
            eventSource = new EventSource('/events');
            
            eventSource.onmessage = function(e) {
                const d = JSON.parse(e.data);
                
                // Update Numeric displays
                document.getElementById('val-press').innerText = `${d.pressure.toFixed(2)} kPa`;
                document.getElementById('val-forecast').innerText = `${d.forecast.toFixed(2)} kPa`;
                document.getElementById('val-slope').innerText = `${d.slope.toFixed(2)} kPa/s`;
                document.getElementById('val-reserve').innerText = `${d.reserve.toFixed(1)}%`;
                document.getElementById('val-health').innerText = `${d.health.toFixed(1)}%`;
                document.getElementById('val-power').innerText = `${(d.voltage * d.current).toFixed(2)} W`;
                document.getElementById('val-volts').innerText = `${d.voltage.toFixed(2)} V`;
                
                // Color code ML State Badge
                const mlLabel = document.getElementById('val-ml-state');
                mlLabel.innerText = d.ml_state;
                if (d.ml_state === "STABLE") {
                    mlLabel.style.background = "rgba(16, 185, 129, 0.15)";
                    mlLabel.style.borderColor = "#10b981";
                    mlLabel.style.color = "#10b981";
                } else if (d.ml_state === "SQUEEZE") {
                    mlLabel.style.background = "rgba(59, 130, 246, 0.15)";
                    mlLabel.style.borderColor = "#3b82f6";
                    mlLabel.style.color = "#3b82f6";
                } else if (d.ml_state === "YIELDING") {
                    mlLabel.style.background = "rgba(245, 158, 11, 0.15)";
                    mlLabel.style.borderColor = "#f59e0b";
                    mlLabel.style.color = "#f59e0b";
                } else if (d.ml_state === "DANGER") {
                    mlLabel.style.background = "rgba(244, 63, 94, 0.2)";
                    mlLabel.style.borderColor = "#f43f5e";
                    mlLabel.style.color = "#f43f5e";
                }
                
                // Health progress bar update
                const hBar = document.getElementById('health-bar');
                hBar.style.width = `${d.health}%`;
                if (d.health > 50) {
                    hBar.style.backgroundColor = 'var(--accent-emerald)';
                } else if (d.health > 20) {
                    hBar.style.backgroundColor = 'var(--accent-amber)';
                } else {
                    hBar.style.backgroundColor = 'var(--accent-rose)';
                }

                // Add to chart (throttle chart rendering to 30Hz to save CPU)
                let now = Date.now();
                if (!window.lastPlotTime) window.lastPlotTime = 0;
                if (now - window.lastPlotTime > 33) {
                    window.lastPlotTime = now;
                    chart.data.labels.push(d.time.toFixed(1));
                    chart.data.datasets[0].data.push(d.pressure);
                    chart.data.datasets[1].data.push(d.forecast);
                    chart.data.datasets[2].data.push(d.slope);
                    
                    if (chart.data.labels.length > 150) {
                        chart.data.labels.shift();
                        chart.data.datasets[0].data.shift();
                        chart.data.datasets[1].data.shift();
                        chart.data.datasets[2].data.shift();
                    }
                    chart.update('none'); // silent update
                }
            };
        }

        function stopEventSource() {
            if (eventSource) {
                eventSource.close();
                eventSource = null;
            }
        }

        // On Load init
        window.onload = function() {
            initChart();
            loadStatus();
        };
    </script>
</body>
</html>
"""

# ----------------------------------------------------------------------
# 7. RUNSERVER LAUNCHER
# ----------------------------------------------------------------------
def run_server():
    # Build standard threaded HTTP server wrapper
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        allow_reuse_address = True
        pass

    server_address = ("", HTTP_PORT)
    httpd = ThreadedHTTPServer(server_address, WebServerHandler)
    
    print(f"\n==================================================")
    print(f"🚀 BIP Headless Server Launching Successfully!")
    print(f"🔗 WebGUI dashboard address: http://localhost:{HTTP_PORT}")
    print(f"==================================================\n")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down BIP Server...")
    finally:
        engine.stop_connection()
        engine.stop_logging()
        httpd.server_close()

if __name__ == "__main__":
    run_server()
