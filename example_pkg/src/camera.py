import cv2
import numpy as np
import imageReceive

def get_frames(video_path):
    cap = cv2.VideoCapture(video_path)
    frames = []

    while True:
        success, frame = cap.read()
        if not success:
            break
        frame = cv2.resize(frame, (480, 240))
        frames.append(frame)

    cap.release()
    return frames

def get_frame_receiver():
    imageReceive.receive()
    frame = imageReceive.get_latest_frame()
    return frame