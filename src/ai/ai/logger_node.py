import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
from datetime import datetime

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.log_file_path = 'vision_log.csv'
        self.csv_header = ['timestamp', 'full_result_string']
        self.subscription = self.create_subscription(String, '/vision_results', self.results_callback, 10)
        self.setup_csv_file()
        self.get_logger().info(f"✅ Logger node started. Saving data to {self.log_file_path}")

    def setup_csv_file(self):
        write_header = not os.path.exists(self.log_file_path)
        with open(self.log_file_path, 'a+', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if write_header:
                writer.writerow(self.csv_header)

    def results_callback(self, msg):
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            with open(self.log_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([timestamp, msg.data])
        except Exception as e:
            self.get_logger().error(f"Failed to write to log file: {e}")

def main(args=None):
    rclpy.init(args=args)
    logger_node = LoggerNode()
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()