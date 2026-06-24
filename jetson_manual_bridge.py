#!/usr/bin/env python3
"""
==============================================================================
TAUVER: MANUAL ROS 2 SERIAL TRANSLATOR NODE (HEADLESS ENGINE)
Architecture: Jetson (Ubuntu / ROS 2) -> WSL2 Testing
Role: The Radio Tower. Reads Arduino USB, broadcasts DDS over Wi-Fi.
==============================================================================
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
import serial
import json
import time

class JetsonManualBridge(Node):
    def __init__(self):
        super().__init__('jetson_manual_bridge')

        # --- SUBSCRIBER: Listening to the Wi-Fi for Commands ---
        self.cmd_sub = self.create_subscription(String, '/science/manual_cmds', self.ros_command_callback, 10)

        # --- PUBLISHERS: Broadcasting to the Wi-Fi ---
        self.pub_rock_mass = self.create_publisher(Float32, '/science/sensors/rock_mass', 10)
        self.pub_deep_mass = self.create_publisher(Float32, '/science/sensors/deep_mass', 10)
        self.pub_co2       = self.create_publisher(Int32,   '/science/sensors/co2', 10)
        self.pub_temp      = self.create_publisher(Float32, '/science/sensors/temp', 10)
        self.pub_hum       = self.create_publisher(Float32, '/science/sensors/humidity', 10)
        self.pub_mq4       = self.create_publisher(Int32,   '/science/sensors/methane_raw', 10)
        self.pub_wind_s    = self.create_publisher(Float32, '/science/sensors/wind_speed', 10)
        self.pub_wind_d    = self.create_publisher(Int32,   '/science/sensors/wind_direction', 10)
        self.pub_pitch     = self.create_publisher(Float32, '/science/sensors/pitch', 10)
        self.pub_roll      = self.create_publisher(Float32, '/science/sensors/roll', 10)
        self.pub_rock_lid  = self.create_publisher(Bool,    '/science/actuators/rock_lid', 10)
        self.pub_deep_lid  = self.create_publisher(Bool,    '/science/actuators/deep_lid', 10)

        # --- HARDWARE: Serial Connection ---
        self.serial_port = '/dev/ttyACM0' # Make sure this matches your Arduino (ACM0 or USB0)
        self.baud_rate = 115200
        
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"HARDWARE: Successfully connected to Arduino on {self.serial_port}")
            time.sleep(2) # Wait for Arduino to reboot after serial connection
        except serial.SerialException as e:
            self.get_logger().error(f"HARDWARE FAILURE: Could not connect to {self.serial_port}.")
            raise SystemExit

        # --- EVENT LOOP: Poll the USB port at 50Hz ---
        self.polling_timer = self.create_timer(0.02, self.poll_arduino_telemetry)

    def ros_command_callback(self, msg):
        """ Catches commands from the GUI over Wi-Fi and fires them down the USB wire. """
        command_str = msg.data.strip()
        if command_str:
            self.get_logger().info(f"TRANSMITTING COMMAND: Writing '{command_str}' to USB.")
            serial_packet = f"{command_str}\n".encode('utf-8')
            self.arduino.write(serial_packet)
            self.arduino.flush()

    def poll_arduino_telemetry(self):
        """ Catches JSON strings from the USB wire and broadcasts them over Wi-Fi. """
        if self.arduino.in_waiting > 0:
            try:
                raw_bytes = self.arduino.readline()
                raw_line = raw_bytes.decode('utf-8').strip()
                
                # Strict JSON validation shield
                if raw_line.startswith('{') and raw_line.endswith('}'):
                    data = json.loads(raw_line)
                    
                    # Publish all extracted values to their dedicated ROS 2 Topics
                    self.publish_float(self.pub_rock_mass, data.get('rock_c', 0.0))
                    self.publish_float(self.pub_deep_mass, data.get('deep_c', 0.0))
                    self.publish_int(self.pub_co2, data.get('co2', 0))
                    self.publish_float(self.pub_temp, data.get('temp', 0.0))
                    self.publish_float(self.pub_hum, data.get('hum', 0.0))
                    self.publish_int(self.pub_mq4, data.get('mq4_r', 0))
                    self.publish_float(self.pub_wind_s, data.get('wind_s', 0.0))
                    self.publish_int(self.pub_wind_d, data.get('wind_d', 0))
                    self.publish_float(self.pub_pitch, data.get('pitch', 0.0))
                    self.publish_float(self.pub_roll, data.get('roll', 0.0))
                    self.publish_bool(self.pub_rock_lid, bool(data.get('r_lid', 0)))
                    self.publish_bool(self.pub_deep_lid, bool(data.get('d_lid', 0)))
            
            # The Fault Tolerance Shield: Catch dropped USB packets silently
            except UnicodeDecodeError:
                pass
            except json.JSONDecodeError as e:
                # We can comment out these warnings once we trust the system, 
                # but leaving them in is great for debugging the WSL2 tunnel!
                self.get_logger().warning(f"PARSING CRASH (Dropped Packet): {e}")
                self.get_logger().warning(f"THE RAW STRING: {raw_line}")
            except Exception as e:
                self.get_logger().error(f"RUNTIME ERROR: {e}")

    # --- HELPER FUNCTIONS ---
    def publish_float(self, publisher, val):
        msg = Float32()
        msg.data = float(val)
        publisher.publish(msg)

    def publish_int(self, publisher, val):
        msg = Int32()
        msg.data = int(val)
        publisher.publish(msg)

    def publish_bool(self, publisher, val):
        msg = Bool()
        msg.data = bool(val)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JetsonManualBridge()
    try:
        rclpy.spin(node) # Enter the infinite loop
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Manual Science Bridge Node.")
    finally:
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()