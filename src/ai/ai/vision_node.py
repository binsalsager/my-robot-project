import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import face_recognition
import numpy as np
from deepface import DeepFace
import pickle
import os
import threading

class VisionNode(Node):
    """
    A comprehensive vision node for ROS 2 that performs face detection, 
    recognition, and emotion analysis in a single efficient process.
    """
    def __init__(self):
        super().__init__('vision_node')
        
        # --- Subscribers and Publishers ---
        self.subscription = self.create_subscription(Image, '/video_feed', self.video_callback, 10)
        self.status_publisher = self.create_publisher(Bool, '/person_detected_status', 10)
        self.results_publisher = self.create_publisher(String, '/vision_results', 10)
        
        self.bridge = CvBridge()
        
        # --- Load Face Encodings ---
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_encodings()

        # --- State and Timing ---
        self.person_present_last_state = False
        
        # --- Preload the DeepFace model in a separate thread ---
        threading.Thread(target=self.preload_deepface, daemon=True).start()
        
        self.get_logger().info("✅ Vision Node has started successfully.")

    def load_encodings(self):
        """Loads face encodings from the pickle file."""
        # Assumes the encodings file is in the workspace root when launched
        encodings_path = 'encodings.pkl'
        self.get_logger().info(f"Loading known face encodings from {encodings_path}...")
        try:
            with open(encodings_path, 'rb') as f:
                data = pickle.load(f)
                self.known_face_encodings = data['encodings']
                self.known_face_names = data['names']
            self.get_logger().info(f"Loaded {len(self.known_face_names)} encodings.")
        except FileNotFoundError:
            self.get_logger().error(f"FATAL: Encodings file not found at '{encodings_path}'. Please run create_encodings.py first and move the file to your workspace root.")
        except Exception as e:
            self.get_logger().error(f"Error loading encodings file: {e}")
            
    def preload_deepface(self):
        """Preloads the DeepFace model to prevent delays on first detection."""
        try:
            self.get_logger().info("Preloading DeepFace model (this may take a moment)...")
            dummy_frame = np.zeros((100, 100, 3), dtype=np.uint8)
            DeepFace.analyze(dummy_frame, actions=['emotion'], enforce_detection=False)
            self.get_logger().info("DeepFace model preloaded successfully.")
        except Exception as e:
            self.get_logger().warn(f"Could not preload DeepFace model: {e}")

    def video_callback(self, msg):
        """Main callback for processing video frames."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        rgb_small_frame = cv2.cvtColor(cv2.resize(frame, (0, 0), fx=0.5, fy=0.5), cv2.COLOR_BGR2RGB)

        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        person_currently_present = bool(face_locations)
        if person_currently_present != self.person_present_last_state:
            self.person_present_last_state = person_currently_present
            status_msg = Bool()
            status_msg.data = person_currently_present
            self.status_publisher.publish(status_msg)

        results = []
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Person"
            
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            if len(face_distances) > 0:
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
            
            emotion = "N/A"
            try:
                face_roi = rgb_small_frame[top:bottom, left:right]
                analysis = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False)
                emotion = analysis[0]['dominant_emotion']
            except:
                pass 

            results.append(f"Name: {name}, Emotion: {emotion}")

        if results:
            result_msg = String()
            result_msg.data = " | ".join(results)
            self.results_publisher.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()