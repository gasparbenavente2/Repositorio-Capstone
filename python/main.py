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
                target_angle = robot.min_angle
                s = input("Press s to start: ")
                if s == 's':
                    robot.estado = 'homing'

            elif robot.estado == 'homing':
                robot.home()
                skip_homing = True

                if response == 'AHOME' or skip_homing:
                    print("Response home")
                    robot.estado = 'find_target'

            elif robot.estado == 'find_target':
                if target_angle < robot.max_angle + 1:
                    robot.goto(p.homing_angle_1, p.homing_angle_2, target_angle)

                    ret, img = cap.read()
                    
                    if ret:
                        result = vision.get_diff_y(img)
                        
                        if result:
                            diff_y = result['diff_y']
                            robot.diff_list.append([target_angle, diff_y])
                    target_angle += 1
                else:
                    df = pd.DataFrame(robot.diff_list, columns=['angle', 'diff_y'])
                    df.to_csv('diff_list.csv', index=False)

                    df = df.sort_values(by='diff_y', ascending=True)
                    robot.best_angle = df.iloc[0]['angle']
                    robot.pos_target = robot.target_geometry()
                    robot.pos_aprox = robot.aprox_geometry()

                    print('Listo con target')
                    robot.estado = 'aprox'

            elif robot.estado == 'aprox':
                # q1_aprox, q2_aprox = robot.inv_kinematics(robot.pos_aprox)
                # robot.goto(q1_aprox, q2_aprox, -30)
                pass

            elif robot.estado == 'correct':
                pass
            elif robot.estado == 'insert':
                pass
            elif robot.estado == 'trigger':
                pass
            elif robot.estado == 'exit':
                pass
            elif robot.estado == 'test':
                robot.goto(p.homing_angle_1-20, p.homing_angle_2+20, 100)