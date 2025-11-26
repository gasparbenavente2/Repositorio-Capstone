import serial
import cv2
import time
import params as p
import robot

# puerto serial
ser = serial.Serial(
    p.port,
    p.baud_rate,
    timeout=0.1)


if __name__ == "__main__":
    print("Main start")
    robot = robot.Robot(ser)

    old_time = time.time()
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print("Arduino:", response)

        current_time = time.time()
        if current_time >= old_time + p.dt:
            old_time = time.time()
            # msg = "AGOTO 000.00 000.00 000.00;"
            # msg = msg.encode("utf-8")
            # ser.write(msg)
            # print(f"Enviado: {msg}")
            # time.sleep(0.004)

            # time.sleep(0.10)
            # try:
            #     response = ser.readline().decode('utf-8').strip()
            #     if response:
            #         print("Arduino:", response)
            # except UnicodeDecodeError:
            #     pass

            if robot.estado == 'rest':
                s = input("Press s to start: ")
                if s == 's':
                    robot.estado = 'homing'

            elif robot.estado == 'homing':
                if response == 'HOMING OK':
                    robot.estado = 'find_target'

            elif robot.estado == 'find_target':
                pass
            elif robot.estado == 'aprox':
                pass
            elif robot.estado == 'correct':
                pass
            elif robot.estado == 'insert':
                pass
            elif robot.estado == 'trigger':
                pass
            elif robot.estado == 'exit':
                pass