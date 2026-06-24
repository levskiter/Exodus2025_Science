#!/usr/bin/env python3
"""
==============================================================================
TAUVER: ASTROBIOLOGY ROS 2 TRANSLATOR NODE
Architecture: Jetson (Ubuntu / ROS 2) -> WSL2 Testing
Role: The Radio Tower. Reads Astro Arduino USB, broadcasts DDS over Wi-Fi.
==============================================================================
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
import serial
import json
import time

class AstroManualBridge(Node):
    def __init__(self):
        super().__init__('astro_manual_bridge')

        # --- SUBSCRIBER: Listening to the Wi-Fi for Commands ---
        self.cmd_sub = self.create_subscription(String, '/astro/manual_cmds', self.ros_command_callback, 10)

        # --- PUBLISHERS: Broadcasting to the Wi-Fi ---
        self.pub_ph = self.create_publisher(Float32, '/astro/sensors/ph', 10)
        self.pub_ph_v = self.create_publisher(Float32, '/astro/sensors/ph_volts', 10)
        self.pub_pump = self.create_publisher(Int32, '/astro/actuators/pump', 10)

        # --- HARDWARE: Serial Connection ---
        self.serial_port = '/dev/ttyACM1' # Usually ACM1 if the main board is ACM0
        self.baud_rate = 115200
        
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"WET-LAB HARDWARE: Successfully connected to Astro Arduino on {self.serial_port}")
            time.sleep(2) 
        except serial.SerialException as e:
            self.get_logger().error(f"HARDWARE FAILURE: Could not connect to {self.serial_port}.")
            raise SystemExit

        # --- EVENT LOOP: Poll the USB port at 20Hz ---
        self.polling_timer = self.create_timer(0.05, self.poll_arduino_telemetry)

    def ros_command_callback(self, msg):
        command_str = msg.data.strip()
        if command_str:
            self.get_logger().info(f"INJECTING COMMAND: '{command_str}' to USB.")
            serial_packet = f"{command_str}\n".encode('utf-8')
            self.arduino.write(serial_packet)
            self.arduino.flush()

    def poll_arduino_telemetry(self):
        if self.arduino.in_waiting > 0:
            try:
                raw_line = self.arduino.readline().decode('utf-8').strip()
                
                # Strict JSON validation shield
                if raw_line.startswith('{') and raw_line.endswith('}'):
                    data = json.loads(raw_line)
                    
                    self.publish_float(self.pub_ph, data.get('ph', 0.0))
                    self.publish_float(self.pub_ph_v, data.get('ph_v', 0.0))
                    self.publish_int(self.pub_pump, data.get('pump', 0))
            
            except UnicodeDecodeError: pass
            except json.JSONDecodeError as e:
                self.get_logger().warning(f"ASTRO PARSING CRASH: {raw_line}")
            except Exception as e:
                self.get_logger().error(f"RUNTIME ERROR: {e}")

    def publish_float(self, publisher, val):
        msg = Float32(); msg.data = float(val); publisher.publish(msg)

    def publish_int(self, publisher, val):
        msg = Int32(); msg.data = int(val); publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AstroManualBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()