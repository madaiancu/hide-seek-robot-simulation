import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import face_recognition
import cv2
import numpy as np
import tf_transformations  # pentru a extrage unghiul din odometrie
from playsound import playsound  # import pentru sunet
import pyttsx3
import time  # import pentru măsurarea timpului

class TurtleBotFaceNav(Node):
    def __init__(self):
        super().__init__('face_nav')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        self.target_detected = False
        self.reached_target = False

        self.voice_engine = pyttsx3.init()
        self.voice_engine.say("Mă uit după ținte!")
        self.voice_engine.runAndWait()

        # Define target positions for each person (x, y)
        self.targets = {
            "Caluseri Abel": (2.0, -6.0),   # Blue cube
            "Iancu Madalina": (0.0, -4.0), # Green cube
            "Roberta Chiton": (-2.0, -6.0) # Red cube
        }

        # Load sample pictures and learn how to recognize them
        self.known_face_encodings = []
        self.known_face_names = []

        abel_image = face_recognition.load_image_file("pozaAbel.jpeg")
        abel_face_encoding = face_recognition.face_encodings(abel_image)[0]
        self.known_face_encodings.append(abel_face_encoding)
        self.known_face_names.append("Caluseri Abel")

        madalina_image = face_recognition.load_image_file("pozaMadalina.jpeg")
        madalina_face_encoding = face_recognition.face_encodings(madalina_image)[0]
        self.known_face_encodings.append(madalina_face_encoding)
        self.known_face_names.append("Iancu Madalina")

        roberta_image = face_recognition.load_image_file("pozaRoberta.png")
        roberta_face_encoding = face_recognition.face_encodings(roberta_image)[0]
        self.known_face_encodings.append(roberta_face_encoding)
        self.known_face_names.append("Roberta Chiton")

        # Open webcam
        self.video_capture = cv2.VideoCapture(0)

        # Current position of TurtleBot
        self.current_position = (0.0, 0.0)
        self.current_theta = 0.0  # orientarea robotului

        # Subscribe to odometry to track position
        self.create_subscription(Odometry, '/odom', self.update_position, 10)

        # Timpul de început pentru deplasarea către țintă
        self.start_time = None

        # Variabila pentru a salva fața detectată ultima dată
        self.last_detected_face = None

        # Set pentru a ține evidența fețelor pentru care s-a redat sunetul
        self.faces_sound_played = set()

        # Timp pentru a afișa periodic mesajul de căutare
        self.last_search_message_time = time.time()

    def update_position(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def navigate_to(self, target_position):
        target_x, target_y = target_position
        current_x, current_y = self.current_position
        distance = np.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        angle_to_target = np.arctan2(target_y - current_y, target_x - current_x)
        twist = Twist()

        if distance > 1.0:
            angular_error = angle_to_target - self.current_theta
            angular_error = np.arctan2(np.sin(angular_error), np.cos(angular_error))
            if abs(angular_error) > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = 1.0 * np.sign(angular_error)
            else:
                twist.linear.x = 0.5
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

            # Verificăm dacă sunetul a fost redat deja pentru această față
            if self.last_detected_face not in self.faces_sound_played:
                playsound("found_sound.mp3")
                self.get_logger().info("Am ajuns la țintă!")
                self.faces_sound_played.add(self.last_detected_face)

            if self.start_time:
                elapsed_time = time.time() - self.start_time
                self.get_logger().info(f"Am ajuns la țintă în {elapsed_time:.2f} secunde!")
                self.start_time = None
        self.publisher.publish(twist)

    def move_turtlebot(self):
        ret, frame = self.video_capture.read()
        if not ret:
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        face_detected = False

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
                self.get_logger().info(f"Detected: {name}")
                self.get_logger().info("Încă te caut! Rămâi pe loc, vin la tine!")
                if name != self.last_detected_face:
                    self.last_detected_face = name
                    self.start_time = time.time()
                if name in self.targets:
                    self.navigate_to(self.targets[name])
                    face_detected = True
                break

        if not face_detected:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.publisher.publish(twist)

            # Afișează mesaj periodic dacă încă caută
            current_time = time.time()
            if current_time - self.last_search_message_time > 5.0:
                self.get_logger().info("Poti fugi, dar nu te poti ascunde!")
                self.last_search_message_time = current_time

    def destroy_node(self):
        self.video_capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    face_nav_node = TurtleBotFaceNav()
    try:
        rclpy.spin(face_nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        face_nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
