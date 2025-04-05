#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
latest_frame = None  # Variável global para armazenar o último frame recebido

def callback(data):
    global latest_frame
    latest_frame = bridge.imgmsg_to_cv2(data, "bgr8")  # Converte a imagem ROS para OpenCV

def receive():
    rospy.Subscriber("/catvehicle/camera_front/image_raw_front", Image, callback)

def get_latest_frame():
    global latest_frame
    return latest_frame  # Retorna o último frame recebido

def record_video():
    rospy.init_node("receiveImage", anonymous=True)  # Inicializa o nó ROS
    receive()  # Começa a escutar o tópico

    # Definição do vídeo
    frame_width = 640
    frame_height = 480
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Codec de vídeo
    out = cv2.VideoWriter('output.avi', fourcc, fps, (frame_width, frame_height))

    print("🎥 Gravando... Pressione 'q' para parar.")

    while not rospy.is_shutdown():
        frame = get_latest_frame()
        if frame is not None:
            frame = cv2.resize(frame, (frame_width, frame_height))  # Ajusta tamanho do frame
            out.write(frame)  # Salva frame no vídeo
            cv2.imshow("Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Pressione 'q' para parar
            print("🛑 Gravação finalizada.")
            break

    out.release()  # Libera o arquivo de vídeo
    cv2.destroyAllWindows()

if __name__ == "__main__":
    record_video()
