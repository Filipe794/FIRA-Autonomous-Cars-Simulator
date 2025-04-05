# import cv2
# import numpy as np
# from ultralytics import YOLO

# # Carregar o modelo YOLO de segmentação
# model = YOLO("/home/filipe/catkin_ws/src/FIRA-Autonomous-Cars-Simulator/example_pkg/src/rede_neural/best.pt")

# # Caminho do vídeo de entrada
# video_path = "/home/filipe/catkin_ws/src/FIRA-Autonomous-Cars-Simulator/example_pkg/src/output.mp4"
# cap = cv2.VideoCapture(video_path)

# # Verificar se o vídeo abriu corretamente
# if not cap.isOpened():
#     print(f"❌ Erro ao abrir o vídeo: {video_path}")
#     exit()

# # Obter informações do vídeo original
# frame_width = int(cap.get(3))
# frame_height = int(cap.get(4))
# fps = int(cap.get(cv2.CAP_PROP_FPS))

# # Definir o codec e criar o objeto VideoWriter para salvar o vídeo
# output_path = "/home/filipe/catkin_ws/src/FIRA-Autonomous-Cars-Simulator/example_pkg/src/processed_video.mp4"
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec para MP4
# out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

# print("🎥 Processando vídeo...")

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("✅ Fim do vídeo ou erro ao ler o frame.")
#         break

#     # Fazer a segmentação
#     results = model(frame)

#     # Criar uma máscara preta do mesmo tamanho do frame
#     mask = np.zeros(frame.shape[:2], dtype=np.uint8)

#     # Adicionar segmentações à máscara
#     for result in results:
#         if result.masks is not None:
#             for segment in result.masks.xy:
#                 segment = np.array(segment, dtype=np.int32)
#                 cv2.fillPoly(mask, [segment], 255)  # Preenche com branco

#     # Aplicar operação morfológica para suavizar a máscara
#     kernel = np.ones((4, 4), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#     # Converter a máscara para RGB e combiná-la com o frame original
#     mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#     processed_frame = cv2.addWeighted(frame, 0.7, mask_rgb, 0.3, 0)

#     # Escrever o frame processado no vídeo de saída
#     out.write(processed_frame)

# # Liberar recursos
# cap.release()
# out.release()
# print(f"✅ Vídeo processado salvo em: {output_path}")

import cv2
import numpy as np
from ultralytics import YOLO

# Carregar o modelo YOLO de segmentação
model = YOLO("example_pkg/src/rede_neural/best.pt")

# Carregar o vídeo
video_path = "example_pkg/src/output.mp4"  # Altere para o caminho do seu vídeo
cap = cv2.VideoCapture(video_path)

# Obter FPS e tamanho do vídeo para salvar o resultado
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Criar um arquivo de saída (opcional)
output_path = "output.avi"
fourcc = cv2.VideoWriter_fourcc(*"XVID")
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

while cap.isOpened():
    ret, frame = cap.read()  # Captura um frame do vídeo
    if not ret:
        break  # Sai do loop se o vídeo terminou

    # Processa a imagem com o modelo YOLO
    results = model(frame)

    # Itera sobre as detecções
    for result in results:
        masks = result.masks  # Máscaras de segmentação
        boxes = result.boxes  # Caixas delimitadoras
        names = result.names  # Nomes das classes

        if masks is not None:
            for mask in masks.xy:
                mask = np.array(mask, dtype=np.int32)
                cv2.polylines(frame, [mask], isClosed=True, color=(0, 255, 0), thickness=2)  # Desenhar contorno
                cv2.fillPoly(frame, [mask], color=(0, 255, 0, 50))  # Preencher com transparência

        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Coordenadas da caixa
                conf = box.conf[0].item()  # Confiança
                class_id = int(box.cls[0])  # Classe detectada
                label = f"{names[class_id]} {conf:.2f}"  # Rótulo da detecção

                # Desenhar a caixa e rótulo
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Escrever no vídeo de saída
    out.write(frame)

    # Exibir a imagem processada
    # cv2.imshow("Detecção YOLO", frame)

    # Pressionar 'q' para sair
    # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break

cap.release()
out.release()
cv2.destroyAllWindows()
