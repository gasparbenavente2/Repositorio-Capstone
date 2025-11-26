import serial
import cv2
import time
import params as p
import robot
import vision
import pandas as pd

# puerto serial
ser = serial.Serial(
    p.port,
    p.baud_rate,
    timeout=0.1)


if __name__ == "__main__":
    print("Main start")
    robot = robot.Robot(ser)
    cap = cv2.VideoCapture(0)

    old_time = time.time()
    while True:
        if ser.in_waiting > 0:
            response = ser.readline()
            print(response)
            response = response.decode().strip()
            print("Arduino:", response)

        current_time = time.time()
        if current_time >= old_time + p.dt:
            old_time = time.time()
            # msg = "AGOTO 000.00 000.00 000;"
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
                target_counter = robot.min_angle
                s = input("Press s to start: ")
                if s == 's':
                    robot.estado = 'homing'

            elif robot.estado == 'homing':
                robot.home()
                skip_homing = False

                if response == 'AHOME' or skip_homing:
                    robot.estado = 'aprox'

            elif robot.estado == 'find_target':
                target_counter += 1

                if target_counter < robot.max_angle + 1:
                    robot.goto(0, 0, target_counter)

                    ret, img = cap.read()
                    
                    if ret:
                        result = vision.get_diff_y(img)
                        
                        if result:
                            diff_y = result['diff_y']
                            robot.diff_list.append([target_counter, diff_y])
                else:
                    print("Target counter: ", target_counter)
                    df = pd.DataFrame(robot.diff_list)
                    df.to_csv('diff_list.csv', index=False)
                    robot.estado = 'rest'

            elif robot.estado == 'aprox':
                robot.goto(-20, 20, 100)
            elif robot.estado == 'correct':
                pass
            elif robot.estado == 'insert':
                pass
            elif robot.estado == 'trigger':
                pass
            elif robot.estado == 'exit':
                pass