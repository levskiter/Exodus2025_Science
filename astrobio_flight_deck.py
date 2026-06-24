"""
==============================================================================
TAUVER ASTROBIOLOGY FLIGHT DECK V1.4
Architecture: PyQt5 + Dual-Engine (rclpy/Serial) + Asynchronous Queues
Capability: Auto-switching, Dynamic pH Visualizer, Pump Command Deck
Upgrades: Graph Auto-Range snapping, Ghost-line purging, Reset View added.
==============================================================================
"""

import sys
import json
import time
import serial
import queue
import math
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QPushButton, QLabel, QSplitter, QComboBox, QGroupBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

# --- DYNAMIC ROS 2 IMPORTS ---
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Int32, String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("SYS WARN: 'rclpy' not found. Defaulting to Local Serial.")

# ================= CONFIGURATION =================
DEFAULT_COM_PORT = 'COM4' # Update to your Astro Arduino port if on Windows
BAUD_RATE = 115200
# =================================================

class SerialThread(QThread):
    data_received = pyqtSignal(dict)
    
    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None
        self.cmd_queue = queue.Queue()

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            time.sleep(2) 
            while self.running:
                while not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write((cmd + '\n').encode('utf-8'))
                        print(f"TX (SERIAL) -> {cmd}") 

                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            data = json.loads(line)
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            pass
        except Exception as e:
            print(f"Serial Error: {e}")

    def send_command(self, cmd):
        self.cmd_queue.put(cmd)

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

if ROS_AVAILABLE:
    class AstroSubscriberNode(Node):
        def __init__(self, parent_thread):
            super().__init__('astro_gui_node')
            self.parent_thread = parent_thread
            self.state_cache = {'ph': 7.0, 'ph_v': 2.5, 'pump': 0}

            self.create_subscription(Float32, '/astro/sensors/ph', lambda msg: self._update('ph', msg.data), 10)
            self.create_subscription(Float32, '/astro/sensors/ph_volts', lambda msg: self._update('ph_v', msg.data), 10)
            self.create_subscription(Int32,   '/astro/actuators/pump', lambda msg: self._update('pump', msg.data), 10)
            self.cmd_pub = self.create_publisher(String, '/astro/manual_cmds', 10)
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
            self.cmd_queue = queue.Queue()
            
        def run(self):
            self.running = True
            if not rclpy.ok(): rclpy.init()
            self.node = AstroSubscriberNode(self)
            try: 
                while self.running and rclpy.ok():
                    while not self.cmd_queue.empty():
                        cmd = self.cmd_queue.get_nowait()
                        if self.node:
                            self.node.transmit_command(cmd)
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception as e: 
                print(f"ROS 2 Error: {e}")
            finally:
                if self.node: self.node.destroy_node()
                
        def send_command(self, cmd):
            self.cmd_queue.put(cmd)
            
        def stop(self):
            self.running = False

class AstroDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TAUVER Astro Bio Console V1.4")
        self.resize(1300, 800)
        self.setStyleSheet("background-color: #0d0d0d; color: #e0e0e0;")
        self.active_engine = None 
        
        # --- Time-Domain Data Arrays ---
        self.graph_active = False
        self.avg_active = False
        self.time_data = []
        self.ph_data = []
        self.sample_accumulator = []
        self.start_time = time.time()

        self.init_ui()
        
        self.combo_engine.blockSignals(True)
        if ROS_AVAILABLE: self.combo_engine.setCurrentText("ROS 2 (Wi-Fi Distributed)")
        else: self.combo_engine.setCurrentText("LOCAL SERIAL (USB)"); self.combo_engine.setEnabled(False)
        self.combo_engine.blockSignals(False)
        
        self.switch_engine()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # TOP BAR
        top_bar = QHBoxLayout()
        self.combo_engine = QComboBox()
        self.combo_engine.addItems(["LOCAL SERIAL (USB)", "ROS 2 (Wi-Fi Distributed)"])
        self.combo_engine.setStyleSheet("background-color: #2b2b2b; color: #00ffcc; font-weight: bold; padding: 5px;")
        self.combo_engine.currentIndexChanged.connect(self.switch_engine)
        self.status_indicator = QLabel("⭕ OFFLINE")
        
        top_bar.addWidget(QLabel("<b>DATA LINK:</b>"))
        top_bar.addWidget(self.combo_engine)
        top_bar.addWidget(self.status_indicator)
        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, stretch=1)

        # ================= LEFT PANEL: VISUALIZER & GRAPH =================
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 10, 0)
        
        # 1. LIVE HUD
        hud_box = QGroupBox("LIVE METROLOGY")
        hud_box.setStyleSheet("color: #a6a6a6; font-weight: bold;")
        hud_layout = QVBoxLayout()
        
        self.ph_display = QLabel("7.00")
        self.ph_display.setAlignment(Qt.AlignCenter)
        self.ph_display.setFont(QFont("Consolas", 50, QFont.Bold))
        self.ph_display.setStyleSheet("background-color: #00ff00; color: black; border-radius: 15px; border: 3px solid white;")
        hud_layout.addWidget(self.ph_display)

        self.lbl_volts = QLabel("Probe Voltage: 2.500 V")
        self.lbl_volts.setAlignment(Qt.AlignCenter)
        self.lbl_volts.setFont(QFont("Consolas", 12))
        hud_layout.addWidget(self.lbl_volts)
        hud_box.setLayout(hud_layout)
        left_layout.addWidget(hud_box)

        # 2. TEMPORAL DRIFT GRAPH
        pg.setConfigOption('background', '#0a0a0a')
        pg.setConfigOption('foreground', '#D3D3D3')
        self.ph_plot = pg.PlotWidget(title="Temporal pH Drift")
        self.ph_plot.setLabel('left', 'pH Level')
        self.ph_plot.setLabel('bottom', 'Time (s)')
        self.ph_plot.showGrid(x=True, y=True, alpha=0.3)
        self.ph_plot.setYRange(0, 14)
        self.ph_line = self.ph_plot.plot(pen=pg.mkPen('#00ffcc', width=2))
        left_layout.addWidget(self.ph_plot, stretch=2)

        # 3. STATISTICAL ANALYSIS DECK
        stats_box = QGroupBox("STATISTICAL ANALYSIS ENGINE")
        stats_box.setStyleSheet("color: #00ffff; border: 1px solid #333;")
        stats_layout = QGridLayout()

        self.btn_toggle_graph = QPushButton("📈 START GRAPH")
        self.btn_toggle_graph.setStyleSheet("background-color: #1a1a1a; color: #00ffcc; font-weight: bold; padding: 10px;")
        self.btn_toggle_graph.clicked.connect(self.toggle_graph)

        self.btn_reset_graph = QPushButton("🔄 RESET VIEW")
        self.btn_reset_graph.setStyleSheet("background-color: #333333; color: white; font-weight: bold; padding: 10px;")
        self.btn_reset_graph.clicked.connect(self.reset_graph_view)

        self.btn_start_avg = QPushButton("📊 BEGIN SAMPLE WINDOW")
        self.btn_start_avg.setStyleSheet("background-color: #005580; color: white; font-weight: bold; padding: 10px;")
        self.btn_start_avg.clicked.connect(self.start_sample)

        self.btn_latch_avg = QPushButton("⏹ LATCH & REJECT NOISE")
        self.btn_latch_avg.setStyleSheet("background-color: #802b00; color: white; font-weight: bold; padding: 10px;")
        self.btn_latch_avg.clicked.connect(self.latch_sample)

        self.lbl_stat_result = QLabel("STANDBY: Awaiting sequence...")
        self.lbl_stat_result.setFont(QFont("Consolas", 11, QFont.Bold))
        self.lbl_stat_result.setStyleSheet("background-color: #050505; color: #ffbf00; padding: 10px; border: 1px dashed #ffbf00;")
        self.lbl_stat_result.setAlignment(Qt.AlignCenter)

        stats_layout.addWidget(self.btn_toggle_graph, 0, 0)
        stats_layout.addWidget(self.btn_reset_graph, 0, 1)
        stats_layout.addWidget(self.btn_start_avg, 1, 0)
        stats_layout.addWidget(self.btn_latch_avg, 1, 1)
        stats_layout.addWidget(self.lbl_stat_result, 2, 0, 1, 2)
        
        stats_box.setLayout(stats_layout)
        left_layout.addWidget(stats_box)

        splitter.addWidget(left_panel)

        # ================= RIGHT PANEL: COMMAND DECK =================
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        lbl_cmd_title = QLabel("<center><b>ASTRO COMMAND DECK</b></center>")
        lbl_cmd_title.setStyleSheet("font-size: 18px; color: #ffbf00; background: #1a1a1a; padding: 10px;")
        right_layout.addWidget(lbl_cmd_title)

        grid = QGridLayout()

        # FWD PUMP CONTROLS
        btn_pump_fwd_short = QPushButton("💧 INJECT 10ml (FWD)")
        btn_pump_fwd_short.setStyleSheet("background-color: #0066cc; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_pump_fwd_short.clicked.connect(lambda: self.route_command("PUMP_1000"))

        btn_pump_fwd_med = QPushButton("💧 INJECT 60ml (FWD)")
        btn_pump_fwd_med.setStyleSheet("background-color: #004d99; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_pump_fwd_med.clicked.connect(lambda: self.route_command("PUMP_5000"))

        # REV PUMP CONTROLS
        btn_pump_rev_short = QPushButton("⏪ PURGE 10ml (REV)")
        btn_pump_rev_short.setStyleSheet("background-color: #999900; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_pump_rev_short.clicked.connect(lambda: self.route_command("PUMP_REV_1000"))

        btn_pump_rev_med = QPushButton("⏪ PURGE 60ml (REV)")
        btn_pump_rev_med.setStyleSheet("background-color: #666600; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_pump_rev_med.clicked.connect(lambda: self.route_command("PUMP_REV_5000"))

        btn_pump_stop = QPushButton("🛑 EMERGENCY PUMP STOP")
        btn_pump_stop.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
        btn_pump_stop.clicked.connect(lambda: self.route_command("PUMP_STOP"))

        # CALIBRATION CONTROLS
        btn_cal_4 = QPushButton("🧪 CAL: pH 4.0")
        btn_cal_4.setStyleSheet("background-color: #cc6600; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_cal_4.clicked.connect(lambda: self.route_command("CAL_4"))

        btn_cal_7 = QPushButton("🧪 CAL: pH 7.0")
        btn_cal_7.setStyleSheet("background-color: #009933; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_cal_7.clicked.connect(lambda: self.route_command("CAL_7"))

        btn_cal_10 = QPushButton("🧪 CAL: pH 10.0")
        btn_cal_10.setStyleSheet("background-color: #0000cc; color: white; font-weight: bold; padding: 15px; font-size: 13px;")
        btn_cal_10.clicked.connect(lambda: self.route_command("CAL_10"))

        # GRID LAYOUT ASSEMBLY
        grid.addWidget(QLabel("<b>PUMP INJECTION (FORWARD):</b>"), 0, 0, 1, 2)
        grid.addWidget(btn_pump_fwd_short, 1, 0)
        grid.addWidget(btn_pump_fwd_med, 1, 1)
        
        grid.addWidget(QLabel("<b>PUMP PURGE (REVERSE):</b>"), 2, 0, 1, 2)
        grid.addWidget(btn_pump_rev_short, 3, 0)
        grid.addWidget(btn_pump_rev_med, 3, 1)

        grid.addWidget(btn_pump_stop, 4, 0, 1, 2)
        
        grid.addWidget(QLabel("<b>ON-SITE CALIBRATION:</b>"), 5, 0, 1, 2)
        grid.addWidget(btn_cal_4, 6, 0)
        grid.addWidget(btn_cal_7, 6, 1)
        grid.addWidget(btn_cal_10, 7, 0, 1, 2)

        right_layout.addLayout(grid)
        right_layout.addStretch()

        self.lbl_pump_status = QLabel("PUMP STATE: IDLE")
        self.lbl_pump_status.setStyleSheet("background: #050505; color: #a6a6a6; padding: 15px; font-size: 16px; font-weight: bold; border: 1px solid #333;")
        right_layout.addWidget(self.lbl_pump_status)

        splitter.addWidget(right_panel)
        splitter.setSizes([600, 400])

    # ================= LOGIC AND STATS ENGINE =================
    
    def toggle_graph(self):
        self.graph_active = not self.graph_active
        if self.graph_active:
            self.start_time = time.time()
            self.time_data.clear()
            self.ph_data.clear()
            self.ph_line.setData([], []) # Instantly purge ghost lines
            
            # Snap AutoRange back to active trace while keeping Y strictly 0-14
            self.ph_plot.enableAutoRange(x=True, y=False)
            self.ph_plot.setYRange(0, 14)

            self.btn_toggle_graph.setText("📉 STOP GRAPH")
            self.btn_toggle_graph.setStyleSheet("background-color: #404040; color: #ff3333; font-weight: bold; padding: 10px;")
        else:
            self.btn_toggle_graph.setText("📈 START GRAPH")
            self.btn_toggle_graph.setStyleSheet("background-color: #1a1a1a; color: #00ffcc; font-weight: bold; padding: 10px;")

    def reset_graph_view(self):
        """ Force pyqtgraph to re-enable AutoRange tracking after manual panning/zooming """
        self.ph_plot.enableAutoRange(x=True, y=False)
        self.ph_plot.setYRange(0, 14)

    def start_sample(self):
        self.sample_accumulator.clear()
        self.avg_active = True
        self.btn_start_avg.setStyleSheet("background-color: #00e600; color: black; font-weight: bold; padding: 10px;")
        self.lbl_stat_result.setText("ACQUIRING DATA [n=0]")
        self.lbl_stat_result.setStyleSheet("background-color: #003300; color: #00ff00; padding: 10px;")

    def latch_sample(self):
        self.avg_active = False
        self.btn_start_avg.setStyleSheet("background-color: #005580; color: white; font-weight: bold; padding: 10px;")
        
        n = len(self.sample_accumulator)
        if n < 5:
            self.lbl_stat_result.setText("ERR: Insufficient data points (n<5).")
            self.lbl_stat_result.setStyleSheet("background-color: #330000; color: #ff0000; padding: 10px;")
            return

        # 1. Calculate Initial Mean & Std Dev
        mean_raw = sum(self.sample_accumulator) / n
        variance = sum((x - mean_raw) ** 2 for x in self.sample_accumulator) / n
        std_dev = math.sqrt(variance)

        # 2. Chauvenet's Criterion (2-Sigma Anomaly Rejection)
        filtered_data = [x for x in self.sample_accumulator if abs(x - mean_raw) <= (2 * std_dev)]
        n_filtered = len(filtered_data)
        
        if n_filtered < 2: 
            filtered_data = self.sample_accumulator
            n_filtered = n

        # 3. Calculate Final Purified Statistics
        mean_final = sum(filtered_data) / n_filtered
        var_final = sum((x - mean_final) ** 2 for x in filtered_data) / (n_filtered - 1) 
        std_final = math.sqrt(var_final)
        
        # Standard Error of the Mean (SE) = std_dev / sqrt(n)
        se = std_final / math.sqrt(n_filtered)
        rejected = n - n_filtered

        # 4. Display the scientific result
        report = f"LATCHED: pH {mean_final:.2f} ± {se:.3f} | Rejected: {rejected} spikes"
        self.lbl_stat_result.setText(report)
        self.lbl_stat_result.setStyleSheet("background-color: #001a33; color: #00e6e6; padding: 10px; border: 1px solid #00e6e6;")

    # ================= GUI ENGINE =================
    
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

    def get_ph_color(self, ph):
        if ph < 3.0: return "#ff0000"   
        if ph < 5.0: return "#ff6600"   
        if ph < 6.5: return "#ffcc00"   
        if ph <= 7.5: return "#00cc00"  
        if ph < 9.0: return "#0099ff"   
        if ph < 11.0: return "#0000ff"  
        return "#6600cc"                

    def update_gui(self, data):
        ph_val = data.get('ph', 7.0)
        ph_volts = data.get('ph_v', 0.0)
        pump_state = data.get('pump', 0)

        color_hex = self.get_ph_color(ph_val)
        self.ph_display.setText(f"{ph_val:.2f}")
        text_color = "black" if (5.0 <= ph_val <= 8.5) else "white"
        self.ph_display.setStyleSheet(f"background-color: {color_hex}; color: {text_color}; border-radius: 20px; border: 5px solid #333;")
        self.lbl_volts.setText(f"Probe Voltage: {ph_volts:.3f} V")

        # Temporal Graph Update
        if self.graph_active:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            self.ph_data.append(ph_val)
            if len(self.time_data) > 200: # Rolling window
                self.time_data.pop(0)
                self.ph_data.pop(0)
            self.ph_line.setData(list(self.time_data), list(self.ph_data))

        # Statistical Accumulator Update
        if self.avg_active:
            self.sample_accumulator.append(ph_val)
            self.lbl_stat_result.setText(f"ACQUIRING DATA [n={len(self.sample_accumulator)}]")

        if pump_state == 1:
            self.lbl_pump_status.setText("PUMP STATE: 🟢 INJECTING (FWD)...")
            self.lbl_pump_status.setStyleSheet("background: #003300; color: #00ff00; padding: 15px; font-size: 16px; font-weight: bold; border: 1px solid #00ff00;")
        elif pump_state == -1:
            self.lbl_pump_status.setText("PUMP STATE: 🟡 PURGING (REV)...")
            self.lbl_pump_status.setStyleSheet("background: #333300; color: #ffff00; padding: 15px; font-size: 16px; font-weight: bold; border: 1px solid #ffff00;")
        else:
            self.lbl_pump_status.setText("PUMP STATE: ⚪ IDLE")
            self.lbl_pump_status.setStyleSheet("background: #050505; color: #a6a6a6; padding: 15px; font-size: 16px; font-weight: bold; border: 1px solid #333;")

    def closeEvent(self, event):
        if self.active_engine:
            self.active_engine.stop()
            self.active_engine.wait(1000)
        if ROS_AVAILABLE and rclpy.ok():
            rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = AstroDashboard()
    window.show()
    sys.exit(app.exec_())