"""
==============================================================================
TAUVER GROUND CONTROL STATION V6.2 - "THE BULLETPROOF FLIGHT DECK"
Architecture: PyQt5 + PyQtGraph + PyOpenGL + SQLite3 + (rclpy/Serial)
Capability: Auto-switching between Local Serial and Distributed ROS 2 Wi-Fi
Upgrades: Thread-safe Queues, PyBind11 protection, Double-Ignition Guards.
==============================================================================
"""

import sys
import json
import time
import sqlite3
import serial
import queue  # <-- THE CRITICAL ADDITION FOR THREAD SAFETY
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QPushButton, QLabel, QDialog, QTextEdit, QSplitter, QComboBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg
import pyqtgraph.opengl as gl 

# --- DYNAMIC ROS 2 IMPORTS (The Fallback Mechanism) ---
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Int32, Bool, String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("SYS WARN: 'rclpy' not found. ROS 2 Wi-Fi capabilities DISABLED. Defaulting to Local Serial.")

# ================= CONFIGURATION =================
DEFAULT_COM_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200
HISTORY_LEN = 200  
# =================================================

class CommandCodex(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Rover Command Codex")
        self.resize(450, 600)
        layout = QVBoxLayout(self)
        text = QTextEdit()
        text.setReadOnly(True)
        text.setFont(QFont("Courier", 10))
        text.setStyleSheet("background-color: #1E1E1E; color: #00FF00;")
        
        commands = """
=== CORE COMMANDS ===
[LIDS & ACTUATORS]
ROCK_OPEN  : Opens Rock Payload Bay
ROCK_CLOSE : Closes Rock Payload Bay
DEEP_OPEN  : Opens Deep Payload Bay
DEEP_CLOSE : Closes Deep Payload Bay

[IMU / GYRO]
ZERO_IMU   : Zeros Pitch/Roll to flat level

[SCALES (Dynamic Mass)]
TARE_ROCK       : Zeros the Rock Scale
TARE_DEEP       : Zeros the Deep Scale
        """
        text.setText(commands)
        layout.addWidget(text)

# ==============================================================================
# ENGINE 1: THE SERIAL HARDWARE THREAD (LOCAL)
# ==============================================================================
class SerialThread(QThread):
    data_received = pyqtSignal(dict)
    
    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None
        self.cmd_queue = queue.Queue() # Thread-safe mailbox

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5) 
            time.sleep(2) 
            while self.running:
                # 1. Process outgoing commands safely within this thread
                while not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write((cmd + '\n').encode('utf-8'))
                        print(f"TX (SERIAL) -> {cmd}") 

                # 2. Process incoming data
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            data = json.loads(line)
                            if 'rock_g' in data and 'rock_c' not in data: data['rock_c'] = data['rock_g']
                            if 'deep_g' in data and 'deep_c' not in data: data['deep_c'] = data['deep_g']
                            if 't' not in data and 'temp' in data: data['t'] = data['temp']
                            if 'h' not in data and 'hum' in data: data['h'] = data['hum']
                            if 'wind' not in data and 'wind_s' in data: data['wind'] = data['wind_s']
                            if 'wdir' not in data and 'wind_d' in data: data['wdir'] = data['wind_d']
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            pass 
        except Exception as e:
            print(f"Serial Error: {e}")

    def send_command(self, cmd):
        # The Main GUI Thread drops the command here and leaves instantly.
        self.cmd_queue.put(cmd)

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

# ==============================================================================
# ENGINE 2: THE ROS 2 DDS THREAD (DISTRIBUTED WI-FI)
# ==============================================================================
if ROS_AVAILABLE:
    class ScienceSubscriberNode(Node):
        def __init__(self, parent_thread):
            super().__init__('science_gui_node')
            self.parent_thread = parent_thread
            
            self.state_cache = {
                'pitch': 0.0, 'roll': 0.0, 'rock_c': 0.0, 'deep_c': 0.0,
                'co2': 0, 't': 0.0, 'h': 0.0, 'mq4_r': 0, 'mq4_s': 'WAITING',
                'wind': 0.0, 'wdir': 0, 'r_lid': 0, 'd_lid': 0
            }

            self.create_subscription(Float32, '/science/sensors/pitch', lambda msg: self._update('pitch', msg.data), 10)
            self.create_subscription(Float32, '/science/sensors/roll', lambda msg: self._update('roll', msg.data), 10)
            self.create_subscription(Float32, '/science/sensors/rock_mass', lambda msg: self._update('rock_c', msg.data), 10)
            self.create_subscription(Float32, '/science/sensors/deep_mass', lambda msg: self._update('deep_c', msg.data), 10)
            self.create_subscription(Int32,   '/science/sensors/co2', lambda msg: self._update('co2', msg.data), 10)
            self.create_subscription(Float32, '/science/sensors/temp', lambda msg: self._update('t', msg.data), 10)
            self.create_subscription(Float32, '/science/sensors/humidity', lambda msg: self._update('h', msg.data), 10)
            self.create_subscription(Int32,   '/science/sensors/methane_raw', lambda msg: self._update('mq4_r', msg.data), 10)
            self.create_subscription(Float32, '/science/sensors/wind_speed', lambda msg: self._update('wind', msg.data), 10)
            self.create_subscription(Int32,   '/science/sensors/wind_direction', lambda msg: self._update('wdir', msg.data), 10)
            self.create_subscription(Bool,    '/science/actuators/rock_lid', lambda msg: self._update('r_lid', 1 if msg.data else 0), 10)
            self.create_subscription(Bool,    '/science/actuators/deep_lid', lambda msg: self._update('d_lid', 1 if msg.data else 0), 10)

            self.cmd_pub = self.create_publisher(String, '/science/manual_cmds', 10)
            self.timer = self.create_timer(0.05, self.emit_to_gui)

        def _update(self, key, val):
            self.state_cache[key] = val

        def emit_to_gui(self):
            self.parent_thread.data_received.emit(dict(self.state_cache))

        def transmit_command(self, cmd_str):
            msg = String()
            msg.data = cmd_str
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"TX (ROS2 DDS) -> {cmd_str}")

    class RosThread(QThread):
        data_received = pyqtSignal(dict)
        
        def __init__(self):
            super().__init__()
            self.running = False
            self.node = None
            self.cmd_queue = queue.Queue() # Thread-safe mailbox

        def run(self):
            self.running = True
            
            if not rclpy.ok():
                rclpy.init()
                
            self.node = ScienceSubscriberNode(self)
            
            try:
                while self.running and rclpy.ok():
                    # 1. Safe PyBind11 Publishing
                    # We check the queue and publish from INSIDE the C++ native thread
                    while not self.cmd_queue.empty():
                        cmd = self.cmd_queue.get_nowait()
                        if self.node:
                            self.node.transmit_command(cmd)

                    # 2. Network Spin
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception as e:
                print(f"ROS 2 Error: {e}")
            finally:
                if self.node:
                    self.node.destroy_node()

        def send_command(self, cmd):
            # The Main GUI Thread drops the command here and leaves instantly.
            self.cmd_queue.put(cmd)

        def stop(self):
            self.running = False

# ==============================================================================
# THE GROUND CONTROL GUI
# ==============================================================================
class RoverDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TAUVER Flight Deck V6.2 (Dual-Engine)")
        self.resize(1600, 900)
        self.setStyleSheet("background-color: #0d0d0d; color: #e0e0e0;")

        self.active_engine = None 
        
        # Data buffers
        self.times = deque(maxlen=HISTORY_LEN)
        self.t_start = time.time()
        self.data_pitch = deque(maxlen=HISTORY_LEN)
        self.data_roll = deque(maxlen=HISTORY_LEN)
        self.data_rock_c = deque(maxlen=HISTORY_LEN)
        self.data_deep_c = deque(maxlen=HISTORY_LEN)
        self.data_wind = deque(maxlen=HISTORY_LEN)
        self.data_co2 = deque(maxlen=HISTORY_LEN)
        self.data_temp = deque(maxlen=HISTORY_LEN)
        self.data_hum = deque(maxlen=HISTORY_LEN)
        self.data_mq4 = deque(maxlen=HISTORY_LEN)

        self.is_logging = False
        self.db_conn = None
        self.db_cursor = None
        self.db_initialized = False

        self.init_ui()
        
        # STRUCTURAL FIX: Block signals to prevent "Double-Ignition"
        # Changing text triggers switch_engine(). We only want to boot it once.
        self.combo_engine.blockSignals(True)
        if ROS_AVAILABLE:
            self.combo_engine.setCurrentText("ROS 2 (Wi-Fi Distributed)")
        else:
            self.combo_engine.setCurrentText("LOCAL SERIAL (USB)")
            self.combo_engine.setEnabled(False) 
        self.combo_engine.blockSignals(False)

        self.switch_engine() # Fire the primary ignition

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 1. TOP COMMAND BAR
        top_bar = QHBoxLayout()
        
        self.combo_engine = QComboBox()
        self.combo_engine.addItems(["LOCAL SERIAL (USB)", "ROS 2 (Wi-Fi Distributed)"])
        self.combo_engine.setStyleSheet("background-color: #2b2b2b; color: #00ffcc; font-weight: bold; padding: 5px; border: 1px solid #00ffcc;")
        self.combo_engine.currentIndexChanged.connect(self.switch_engine)
        
        self.status_indicator = QLabel("⭕ OFFLINE")
        self.status_indicator.setStyleSheet("font-weight: bold; color: red; padding: 5px;")
        
        self.btn_reset = QPushButton("🔄 Reset Graphs")
        self.btn_reset.setStyleSheet("background-color: #404040; color: white; padding: 5px;")
        self.btn_reset.clicked.connect(self.reset_graph_view)

        btn_codex = QPushButton("📖 Codex")
        btn_codex.setStyleSheet("background-color: #2a52be; color: white; padding: 5px;")
        btn_codex.clicked.connect(self.show_codex)

        self.btn_log = QPushButton("🔴 Start SQLite Log")
        self.btn_log.setStyleSheet("background-color: #2E8B57; color: white; font-weight: bold; padding: 5px;")
        self.btn_log.clicked.connect(self.toggle_logging)
        
        top_bar.addWidget(QLabel("<b>DATA LINK:</b>"))
        top_bar.addWidget(self.combo_engine)
        top_bar.addWidget(self.status_indicator)
        top_bar.addStretch()
        top_bar.addWidget(self.btn_reset)
        top_bar.addWidget(btn_codex)
        top_bar.addWidget(self.btn_log)
        main_layout.addLayout(top_bar)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, stretch=1)

        # --- LEFT PANEL: 3D & COMMAND DECK ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0,0,0,0)
        
        # 3D Widget
        self.view3d = gl.GLViewWidget()
        self.view3d.opts['distance'] = 20
        self.view3d.setWindowTitle('3D Rover Attitude')
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.view3d.addItem(grid)
        self.rover_mesh = gl.GLBoxItem(size=pg.Vector(6, 10, 2), color=(0, 255, 204, 150))
        self.rover_mesh.translate(-3, -5, 0) 
        self.view3d.addItem(self.rover_mesh)
        
        lbl_3d = QLabel("<center><b>LIVE 3D ARTIFICIAL HORIZON</b></center>")
        lbl_3d.setStyleSheet("background-color: #1a1a1a; padding: 5px;")
        left_layout.addWidget(lbl_3d)
        left_layout.addWidget(self.view3d, stretch=1)

        # QUICK ACTION COMMAND DECK
        cmd_deck_label = QLabel("<center><b>TACTICAL COMMAND DECK</b></center>")
        cmd_deck_label.setStyleSheet("background-color: #1a1a1a; color: #ffbf00; padding: 5px;")
        left_layout.addWidget(cmd_deck_label)

        grid_layout = QGridLayout()
        
        btn_rock_open = QPushButton("ROCK 🔼")
        btn_rock_open.setStyleSheet("background-color: #006633; color: white; font-weight: bold; padding: 15px;")
        btn_rock_open.clicked.connect(lambda: self.route_command("ROCK_OPEN"))
        
        btn_rock_close = QPushButton("ROCK 🔽")
        btn_rock_close.setStyleSheet("background-color: #990000; color: white; font-weight: bold; padding: 15px;")
        btn_rock_close.clicked.connect(lambda: self.route_command("ROCK_CLOSE"))

        btn_deep_open = QPushButton("DEEP 🔼")
        btn_deep_open.setStyleSheet("background-color: #006633; color: white; font-weight: bold; padding: 15px;")
        btn_deep_open.clicked.connect(lambda: self.route_command("DEEP_OPEN"))
        
        btn_deep_close = QPushButton("DEEP 🔽")
        btn_deep_close.setStyleSheet("background-color: #990000; color: white; font-weight: bold; padding: 15px;")
        btn_deep_close.clicked.connect(lambda: self.route_command("DEEP_CLOSE"))

        btn_tare_rock = QPushButton("ZERO ROCK")
        btn_tare_rock.setStyleSheet("background-color: #b37700; color: white; font-weight: bold; padding: 15px;")
        btn_tare_rock.clicked.connect(lambda: self.route_command("TARE_ROCK"))

        btn_tare_deep = QPushButton("ZERO DEEP")
        btn_tare_deep.setStyleSheet("background-color: #b37700; color: white; font-weight: bold; padding: 15px;")
        btn_tare_deep.clicked.connect(lambda: self.route_command("TARE_DEEP"))

        btn_zero_imu = QPushButton("LEVEL IMU GYRO")
        btn_zero_imu.setStyleSheet("background-color: #004d99; color: white; font-weight: bold; padding: 15px;")
        btn_zero_imu.clicked.connect(lambda: self.route_command("ZERO_IMU"))

        grid_layout.addWidget(btn_rock_open, 0, 0)
        grid_layout.addWidget(btn_rock_close, 0, 1)
        grid_layout.addWidget(btn_deep_open, 1, 0)
        grid_layout.addWidget(btn_deep_close, 1, 1)
        grid_layout.addWidget(btn_tare_rock, 2, 0)
        grid_layout.addWidget(btn_tare_deep, 2, 1)
        grid_layout.addWidget(btn_zero_imu, 3, 0, 1, 2)

        left_layout.addLayout(grid_layout)
        splitter.addWidget(left_panel)

        # --- RIGHT PANEL: 2D GRAPHS ---
        pg.setConfigOption('background', '#0a0a0a')
        pg.setConfigOption('foreground', '#D3D3D3')
        self.graph_layout = pg.GraphicsLayoutWidget()
        splitter.addWidget(self.graph_layout)

        self.plot_imu = self.graph_layout.addPlot(title="IMU Tilt (Degrees)")
        self.curve_pitch = self.plot_imu.plot(pen=pg.mkPen('#ff3333', width=2), name="Pitch")
        self.curve_roll = self.plot_imu.plot(pen=pg.mkPen('#3366ff', width=2), name="Roll")
        
        self.plot_scale = self.graph_layout.addPlot(title="Payload Mass (Grams)")
        self.curve_rock = self.plot_scale.plot(pen=pg.mkPen('#ffcc00', width=2), name="Rock")
        self.curve_deep = self.plot_scale.plot(pen=pg.mkPen('#cc33ff', width=2), name="Deep")
        self.graph_layout.nextRow()
        
        self.plot_wind = self.graph_layout.addPlot(title="Wind Speed (km/h)")
        self.curve_wind = self.plot_wind.plot(pen=pg.mkPen('#00ffff', width=2))
        
        self.plot_env = self.graph_layout.addPlot(title="Temp (°C) & Hum (%)")
        self.curve_temp = self.plot_env.plot(pen=pg.mkPen(color='#ff6600', width=2), name="Temp")
        self.curve_hum = self.plot_env.plot(pen=pg.mkPen(color='#3399ff', width=2), name="Hum")
        self.graph_layout.nextRow()

        self.plot_co2 = self.graph_layout.addPlot(title="SCD40 (CO2 ppm)")
        self.curve_co2 = self.plot_co2.plot(pen=pg.mkPen('#00ff00', width=2))

        self.plot_mq4 = self.graph_layout.addPlot(title="MQ4 Methane (Raw Analog)")
        self.curve_mq4 = self.plot_mq4.plot(pen=pg.mkPen(color='#b366ff', width=2))

        self.plot_scale.setXLink(self.plot_imu)
        self.plot_wind.setXLink(self.plot_imu)
        self.plot_env.setXLink(self.plot_imu)
        self.plot_co2.setXLink(self.plot_imu)
        self.plot_mq4.setXLink(self.plot_imu)
        
        splitter.setSizes([450, 1150])

        # 4. GOD-EYE HUD
        self.hud_label = QLabel("Awaiting Telemetry...")
        self.hud_label.setFont(QFont("Consolas", 11, QFont.Bold))
        self.hud_label.setStyleSheet("background: #050505; color: #00ffcc; padding: 10px; border: 1px solid #333;")
        main_layout.addWidget(self.hud_label)

    # ================= ENGINE MANAGEMENT =================
    def switch_engine(self):
        if self.active_engine:
            self.active_engine.stop()
            self.active_engine.wait(1000) 
            self.active_engine = None

        mode = self.combo_engine.currentText()
        
        if "SERIAL" in mode:
            self.status_indicator.setText("🔗 LINK: LOCAL USB")
            self.status_indicator.setStyleSheet("font-weight: bold; color: #ffbf00; padding: 5px;")
            self.active_engine = SerialThread(DEFAULT_COM_PORT, BAUD_RATE)
        elif "ROS" in mode and ROS_AVAILABLE:
            self.status_indicator.setText("📡 LINK: ROS 2 DDS WI-FI")
            self.status_indicator.setStyleSheet("font-weight: bold; color: #00ffcc; padding: 5px;")
            self.active_engine = RosThread()

        if self.active_engine:
            self.active_engine.data_received.connect(self.update_gui)
            self.active_engine.start()

    def route_command(self, cmd):
        if self.active_engine:
            self.active_engine.send_command(cmd)

    def show_codex(self):
        self.codex = CommandCodex(self)
        self.codex.show()

    def reset_graph_view(self):
        self.plot_imu.enableAutoRange()
        self.plot_scale.enableAutoRange()
        self.plot_wind.enableAutoRange()
        self.plot_env.enableAutoRange()
        self.plot_co2.enableAutoRange()
        self.plot_mq4.enableAutoRange()

    def toggle_logging(self):
        if not self.is_logging:
            db_filename = f"TAUVER_DB_{time.strftime('%Y%m%d_%H%M%S')}.db"
            self.db_conn = sqlite3.connect(db_filename)
            self.db_cursor = self.db_conn.cursor()
            self.db_initialized = False 
            
            self.btn_log.setText("⏹ STOP Database Log")
            self.btn_log.setStyleSheet("background-color: #DC143C; color: white; font-weight: bold; padding: 5px;")
            self.is_logging = True
        else:
            self.is_logging = False
            if self.db_conn:
                self.db_conn.commit()
                self.db_conn.close()
            self.btn_log.setText("🔴 Start SQLite Log")
            self.btn_log.setStyleSheet("background-color: #2E8B57; color: white; font-weight: bold; padding: 5px;")

    # ================= DATA RENDERING =================
    def update_gui(self, data):
        self.rover_mesh.resetTransform()
        self.rover_mesh.translate(-3, -5, 0)
        self.rover_mesh.rotate(data.get('roll', 0), 0, 1, 0)  
        self.rover_mesh.rotate(data.get('pitch', 0), 1, 0, 0) 

        r_lid = "OPEN" if data.get('r_lid') else "CLOSED"
        d_lid = "OPEN" if data.get('d_lid') else "CLOSED"
        
        line1 = f"ROCK LID: {r_lid:<8} | DEEP LID: {d_lid:<8} | MQ4 STATE: {data.get('mq4_s', 'N/A')}\n"
        line2 = f"PITCH: {data.get('pitch', 0):>5.1f}°   | ROLL: {data.get('roll', 0):>5.1f}°   | WIND: {data.get('wind', 0):>5.2f} km/h @ {data.get('wdir', 0)}°\n"
        line3 = f"ROCK MASS: {data.get('rock_c', 0):>5.1f}g | DEEP MASS: {data.get('deep_c', 0):>5.1f}g \n"
        line4 = f"CO2: {data.get('co2', 0):>4} ppm   | TEMP: {data.get('t', 0):>4.1f}°C    | HUM: {data.get('h', 0):>4.1f}% | MQ4 RAW: {data.get('mq4_r', 0)}"
        self.hud_label.setText(line1 + line2 + line3 + line4)

        current_time = time.time() - self.t_start
        self.times.append(current_time)
        self.data_pitch.append(data.get('pitch', 0))
        self.data_roll.append(data.get('roll', 0))
        self.data_rock_c.append(data.get('rock_c', 0))
        self.data_deep_c.append(data.get('deep_c', 0))
        self.data_wind.append(max(0, data.get('wind', 0))) 
        self.data_co2.append(data.get('co2', 0))
        self.data_temp.append(data.get('t', 0))
        self.data_hum.append(data.get('h', 0))
        self.data_mq4.append(data.get('mq4_r', 0))

        self.curve_pitch.setData(list(self.times), list(self.data_pitch))
        self.curve_roll.setData(list(self.times), list(self.data_roll))
        self.curve_rock.setData(list(self.times), list(self.data_rock_c))
        self.curve_deep.setData(list(self.times), list(self.data_deep_c))
        self.curve_wind.setData(list(self.times), list(self.data_wind))
        self.curve_temp.setData(list(self.times), list(self.data_temp))
        self.curve_hum.setData(list(self.times), list(self.data_hum))
        self.curve_co2.setData(list(self.times), list(self.data_co2))
        self.curve_mq4.setData(list(self.times), list(self.data_mq4))

        if self.is_logging and self.db_cursor:
            data['timestamp'] = round(current_time, 3)
            if not self.db_initialized:
                cols = ", ".join([f"{k} REAL" if isinstance(v, (int, float)) else f"{k} TEXT" for k, v in data.items()])
                self.db_cursor.execute(f"CREATE TABLE IF NOT EXISTS telemetry ({cols})")
                self.db_initialized = True
            
            placeholders = ", ".join(["?"] * len(data))
            cols = ", ".join(data.keys())
            vals = tuple(data.values())
            self.db_cursor.execute(f"INSERT INTO telemetry ({cols}) VALUES ({placeholders})", vals)
            
            if len(self.times) % 10 == 0:
                self.db_conn.commit()

    def closeEvent(self, event):
        if self.active_engine:
            self.active_engine.stop()
            self.active_engine.wait(1000) 
        if self.db_conn:
            self.db_conn.commit()
            self.db_conn.close()
            
        if ROS_AVAILABLE and rclpy.ok():
            rclpy.shutdown()
            
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RoverDashboard()
    window.show()
    sys.exit(app.exec_())