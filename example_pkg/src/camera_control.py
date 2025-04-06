#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os

class CatvehicleController:
    def __init__(self):
       
        self.bridge = CvBridge()
        self.latest_frame = None
        self.recording = False
        self.video_writer = None
        
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
            print(f"🎥 Iniciando gravação em {video_path}")

    def stop_recording(self):
        if self.recording:
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            print("🛑 Gravação finalizada.")
    def process_keys(self):
        key = cv2.waitKey(1) & 0xFF
        
        # Reset das velocidades
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        key_pressed = False  # 🔹 Flag para saber se houve controle manual

        if key == ord('w'):  # Frente
            self.vel_msg.linear.x = 2.0
            self.vel_msg.angular.z = 0.0
            key_pressed = True
        elif key == ord('s'):  # Ré
            self.vel_msg.linear.x = -2.0
            self.vel_msg.angular.z = 0.0
            key_pressed = True
        elif key == ord('d'):  # Esquerda
            self.vel_msg.linear.x = 2.0
            self.vel_msg.angular.z = -1.0
            key_pressed = True
        elif key == ord('a'):  # Direita
            self.vel_msg.linear.x = 2.0
            self.vel_msg.angular.z = 1.0
            key_pressed = True

        elif key == ord('r'):  # Iniciar/Parar gravação
            if self.recording:
                self.stop_recording()
            else:
                self.start_recording()
        elif key == ord('p'):  # Reproduzir último vídeo
            self.play_last_video()
        elif key == ord('q'):
            return None  # 🔻 Sinaliza que é para sair

        self.vel_pub.publish(self.vel_msg)
        return key_pressed  # 🔸 True se houve controle manual, False se não

    def play_last_video(self):
        video_files = sorted([f for f in os.listdir(self.video_dir) if f.endswith('.avi')])
        if not video_files:
            print("Nenhum vídeo encontrado para reproduzir.")
            return

        last_video = os.path.join(self.video_dir, video_files[-1])
        print(f"🎬 Reproduzindo: {last_video}")
        self.playing_video = True

        cap = cv2.VideoCapture(last_video)
        if not cap.isOpened():
            print("Erro ao abrir o vídeo.")
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
        print("✅ Reprodução finalizada.")
