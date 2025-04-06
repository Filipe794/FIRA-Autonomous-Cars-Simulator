#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import processamento_angulo.lane_detection as ldm
import processamento_angulo.utils as utils
import camera_control as camera_control
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
    rospy.init_node('driver_node', anonymous=True)
    controller = camera_control.CatvehicleController()

    print("=== Direção Inteligente com Controle Manual ===")
    print("  W: Frente | S: Ré | A: Direita | D: Esquerda")
    print("  R: Iniciar/Parar gravação | P: Reproduzir vídeo")
    print("  Q: Sair")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        frame = controller.latest_frame
        manual_override = controller.process_keys()

        if manual_override is None:
            break  # 'q' foi pressionado

        if not manual_override:
            # Nenhuma tecla de controle foi pressionada → usar direção automática
            if frame is not None and not controller.playing_video:
                mask = model.draw_segmented_area(frame)
                mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                curve = drive(mask, controller.vel_pub)
                curve = drive(frame, controller.vel_pub)
                cv2.putText(frame, str(curve), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Direcao Automatica", frame)

        rate.sleep()

    cv2.destroyAllWindows()
