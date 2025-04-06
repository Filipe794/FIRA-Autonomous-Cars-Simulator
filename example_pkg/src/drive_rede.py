#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import processamento_angulo.lane_detection as ldm
import simulator_camera
import processamento_angulo.utils as utils
import video as video
import cv2
import rede_neural.segmentacao as model

def convert_range(value, old_min=-100, old_max=100, new_min=-1.0, new_max=1.0):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min

def drive(img, velocity_publisher):
    intialTrackBarVals = [102, 80, 20, 214]
    # Gera a janela de trackbar
    utils.initializeTrackbars(intialTrackBarVals)
    
    curve = ldm.getLaneCurve(img)
    
    # Convertendo os valores
    converted_data = convert_range(curve)
    print(converted_data)

    # Criando e publicando a mensagem de velocidade
    vel_msg = Twist()
    vel_msg.linear.x = 1.0  # Velocidade fixa para teste
    vel_msg.angular.z = converted_data  # Ajuste baseado na curva detectada
    velocity_publisher.publish(vel_msg)
    
    return curve

if __name__ == "__main__":
    try:
        rospy.init_node("drive_control", anonymous=True)  # 🔹 Inicializa o nó antes de usar Rate
        velocity_publisher = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=20)
        rate = rospy.Rate(10)  # 🔹 Agora `rospy.init_node()` já foi chamado

        while not rospy.is_shutdown():  # 🔹 Usa a condição correta do ROS
            # Receber frame da câmera do ROS
            frame = simulator_camera.get_frame_receiver()
            
            if frame is not None:
                # mask = model.draw_segmented_area(frame)
                # mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
                # curve = drive(mask, velocity_publisher)
                # cv2.putText(mask, str(curve), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # cv2.imshow("win", mask)
                cv2.imshow("win2", frame)
                cv2.waitKey(2)
                
    except rospy.ROSInterruptException:
        pass