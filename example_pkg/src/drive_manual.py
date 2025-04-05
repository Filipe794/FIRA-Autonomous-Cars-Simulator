#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import termios, tty, sys

def get_key():
    """Captura a tecla pressionada pelo usuário."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def drive():
    rospy.init_node("drive", anonymous=True)
    velocity_publisher = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
    vel_msg = Twist()

    print("Use W, A, S, D para mover. Pressione Q para sair.")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w':  # Frente
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 0.0
        elif key == 's':  # Ré
            vel_msg.linear.x = -2.0
            vel_msg.angular.z = 0.0
        elif key == 'a':  # Esquerda
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 1.0
        elif key == 'd':  # Direita
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = -1.0
        elif key == 'q':  # Sair
            break
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        velocity_publisher.publish(vel_msg)

if __name__ == "__main__":
    try:
        drive()
    except rospy.ROSInterruptException:
        pass