#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import signal
import sys

class CatvehicleController:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_frame = None
        self.recording = False
        self.video_writer = None
        self.last_video_path = None
        
        self.vel_pub = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber("/catvehicle/camera_front/image_raw_front", Image, self.image_callback)
        
        self.vel_msg = Twist()
        
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30
        self.video_dir = 'videos'
        os.makedirs(self.video_dir, exist_ok=True)
        self.playing_video = False

    def image_callback(self, data):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.recording and self.video_writer is not None:
                resized_frame = cv2.resize(self.latest_frame, (self.frame_width, self.frame_height))
                self.video_writer.write(resized_frame)
        except Exception as e:
            rospy.logerr("Erro na conversão da imagem: %s" % e)

    def start_recording(self):
        if not self.recording:
            video_path = os.path.join(self.video_dir, f'recording_{rospy.get_time()}.avi')
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.video_writer = cv2.VideoWriter(video_path, fourcc, self.fps, 
                                                (self.frame_width, self.frame_height))
            self.recording = True
            self.last_video_path = video_path
            rospy.loginfo(f"🎥 Iniciando gravação em {video_path}")

    def stop_recording(self):
        if self.recording:
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            rospy.loginfo("🛑 Gravação finalizada.")

    def process_keys(self):
        key = cv2.waitKey(1) & 0xFF
        key_pressed = False

        # Reset das velocidades
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        key_map = {
            ord('w'): (2.0, 0.0),
            ord('s'): (-2.0, 0.0),
            ord('a'): (2.0, 1.0),
            ord('d'): (2.0, -1.0),
        }

        if key in key_map:
            self.vel_msg.linear.x, self.vel_msg.angular.z = key_map[key]
            key_pressed = True
        elif key == ord('r'):
            if self.recording:
                self.stop_recording()
            else:
                self.start_recording()
        elif key == ord('p'):
            self.play_last_video()
        elif key == ord('q'):
            return None

        if key_pressed:
            self.vel_pub.publish(self.vel_msg)

        return key_pressed

    def play_last_video(self):
        if not self.last_video_path or not os.path.exists(self.last_video_path):
            rospy.logwarn("Nenhum vídeo encontrado para reproduzir.")
            return

        rospy.loginfo(f"🎬 Reproduzindo: {self.last_video_path}")
        self.playing_video = True

        cap = cv2.VideoCapture(self.last_video_path)
        if not cap.isOpened():
            rospy.logerr("Erro ao abrir o vídeo.")
            self.playing_video = False
            return

        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow('Reproducao', frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break

        cap.release()
        cv2.destroyAllWindows()
        self.playing_video = False
        rospy.loginfo("✅ Reprodução finalizada.")

def shutdown_handler(sig, frame):
    print("Encerrando com segurança...")
    if controller.recording:
        controller.stop_recording()
    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('drive_rede_node', anonymous=True)
    controller = CatvehicleController()
    signal.signal(signal.SIGINT, shutdown_handler)

    while not rospy.is_shutdown():
        if controller.latest_frame is not None and not controller.playing_video:
            frame = cv2.resize(controller.latest_frame, (controller.frame_width, controller.frame_height))
            status_text = "🟢 Gravando..." if controller.recording else "⏸ Parado"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Camera View", frame)
        manual_override = controller.process_keys()
        if manual_override is None:
            break

    cv2.destroyAllWindows()
