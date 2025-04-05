import cv2
import numpy as np
from ultralytics import YOLO

# Carregar o modelo YOLO de segmentação (YOLOv8-Seg)
model = YOLO("/home/filipe/catkin_ws/src/FIRA-Autonomous-Cars-Simulator/example_pkg/src/rede_neural/best.pt")  # Use um modelo treinado para segmentação


def segmentation(frame):
    results = model(frame)

    # Criar uma máscara preta do mesmo tamanho da imagem
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    # Percorrer as segmentações e adicionar na máscara
    for result in results:
        if result.masks is not None:
            for segment in result.masks.xy:  # Obtém os pontos das máscaras
                segment = np.array(segment, dtype=np.int32)
                cv2.fillPoly(mask, [segment], 255)  # Preenche a área detectada com branco

    # Criar um kernel para operação morfológica (fechamento)
    kernel = np.ones((4, 4), np.uint8)

    # Aplicar fechamento morfológico para preencher os espaços pretos dentro da máscara
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)