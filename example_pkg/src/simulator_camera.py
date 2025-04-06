import cv2
import numpy as np
import video as video

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
    video.receive()
    frame = video.get_latest_frame()
    return frame

while True:
    frame = get_frame_receiver()
    if frame is not None:
        cv2.imshow("Teste", )