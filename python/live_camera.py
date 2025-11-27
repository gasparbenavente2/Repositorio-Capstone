import cv2
from vision import get_diff_y

# Inicializar la cámara (0 suele ser la cámara predeterminada/USB)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: No se pudo abrir la cámara")
    exit()

print("Presiona 'q' para salir")

while True:
    # Leer un frame
    ret, frame = cap.read()

    if not ret:
        print("Error: No se pudo recibir el frame (fin del stream?). Saliendo ...")
        break

    # Calcular diff_y y obtener detalles
    result = get_diff_y(frame)

    # Mostrar información en el frame
    if result is not None:
        diff_y = result['diff_y']
        circle_center = result['circle_center']
        img_center = result['img_center']
        radius = result['radius']
        ratio = result['ratio']

        # Dibujar círculo detectado (Verde)
        cv2.circle(frame, circle_center, radius, (0, 255, 0), 2)
        
        # Dibujar centro del círculo (Punto Rojo)
        cv2.circle(frame, circle_center, 5, (0, 0, 255), -1)
        
        # Dibujar centro de la imagen (Punto Azul)
        cv2.circle(frame, img_center, 5, (255, 0, 0), -1)
        
        # Dibujar línea entre centros (Amarillo)
        cv2.line(frame, img_center, circle_center, (0, 255, 255), 2)

        text = f"Diff Y: {diff_y} | Ratio: {ratio*100:.2f}%"
        color = (0, 255, 0) # Verde si se detecta
    else:
        text = "Diff Y: N/A"
        color = (0, 0, 255) # Rojo si no se detecta

    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # Mostrar el frame resultante
    cv2.imshow('Camara en Vivo', frame)

    # Salir si se presiona 'q'
    if cv2.waitKey(1) == ord('q'):
        break

# Liberar la cámara y cerrar ventanas
cap.release()
cv2.destroyAllWindows()
