import cv2
import numpy as np
from ultralytics import YOLO

# Carregar o modelo YOLO de segmentação
model = YOLO("best.pt", verbose=False)

# Abrir a câmera (ou substitua por um vídeo se quiser testar com vídeo)
cap = cv2.VideoCapture(0)  # Use 0 para webcam, ou substitua por "edit.mp4" para vídeo
corte = 400 

if not cap.isOpened():
    print("Erro ao abrir a câmera ou vídeo.")
    exit()

while True:
    ret, frame = cap.read()
    # frame = cv2.imread("image3.png")

    if not ret:
        print("Fim do vídeo ou erro ao capturar frame.")
        break

    frame = cv2.resize(frame, (900, 750))
   
    
    # adaptar para o espaço
    frames = (frame.copy())[400:,:]

    # Fazer a segmentação
    results = model(frames)

    # Criar uma máscara preta do mesmo tamanho do frame
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    # Percorrer as segmentações e adicionar na máscara
    for result in results:
        if result.masks is not None:
            for segment in result.masks.xy:
                segment = np.array(segment, dtype=np.int32)
                segment[:, 1] += 400
                cv2.fillPoly(mask, [segment], 255)  # Preenche a área detectada com branco

    # Criar um kernel para operação morfológica (fechamento)
    kernel = np.ones((4, 4), np.uint8)

    # Aplicar fechamento morfológico para preencher os espaços pretos dentro da máscara
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Converter a máscara para 3 canais (BGR)
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Definir uma cor para a máscara segmentada (azul com transparência)
    color_mask = np.zeros_like(frame)
    color_mask[:, :] = (255, 0, 0)  # Azul
    masked_segment = cv2.bitwise_and(color_mask, mask_bgr)  # Aplica a cor apenas na máscara

    # Sobrepor a máscara na imagem original com transparência
    # segmented_frame = cv2.addWeighted(frame, 0.7, masked_segment, 0.5, 0)

    # Mostrar máscara final
    # cv2.imshow("Frame original", segmented_frame)
    # cv2.imshow("corte", frames)
    cv2.imshow("Máscara de Segmentação", masked_segment)

    # Encerrar com tecla 'q'
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break

# cap.release()
cv2.destroyAllWindows()
