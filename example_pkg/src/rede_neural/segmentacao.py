import cv2
import numpy as np
from ultralytics import YOLO

# Carregar o modelo YOLO de segmentação
model = YOLO("rede_neural/best.pt")

def segment_frame_area(frame, y_start=500):
    """
    Recorta a área de interesse do frame e aplica a segmentação com o modelo YOLO.
    Retorna os resultados da segmentação e a coordenada de início y.
    """
    frame = cv2.resize(frame, (900, 750))
    cropped = frame[y_start:640, :]
    results = model(cropped)
    return results, y_start, frame.shape[:2]

def apply_segmentation_mask(results, frame_shape, y_offset):
    """
    Cria a máscara com base nos resultados da segmentação, aplicando deslocamento na vertical.
    """
    height, width = frame_shape
    mask = np.zeros((height, width), dtype=np.uint8)

    for result in results:
        if result.masks is not None:
            for segment in result.masks.xy:
                segment = np.array(segment, dtype=np.int32)
                segment[:, 1] += y_offset  # Reposiciona verticalmente
                cv2.fillPoly(mask, [segment], 255)

    kernel = np.ones((4, 4), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

def draw_segmented_area(frame):
    results, y_start, shape = segment_frame_area(frame)
    cv2.imshow("Segmented", results[0].plot())
    mask = apply_segmentation_mask(results, shape, y_start)
    return mask