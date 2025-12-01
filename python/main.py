import serial
import cv2
import time
import params as p
import robot 
import vision
import pandas as pd
import numpy as np

# puerto serial
ser = serial.Serial(
    p.port,
    p.baud_rate,
    timeout=0.1)

message_log = [] # Todos los strings enviados y que llegan

if __name__ == "__main__":
    print("Main start")
    robot = robot.Robot(ser, message_log)
    q1 = np.deg2rad(p.homing_angle_1)
    q2 = np.deg2rad(p.homing_angle_2) - np.pi
    q3 = - q1 - q2
    robot.q = np.array([[q1], [q2], [q3]])
    init_find = True
    cap = cv2.VideoCapture(0)

    old_time = time.time()
    # robot.goto(p.homing_angle_1, p.homing_angle_2, robot.min_angle)
    while True:
        if ser.in_waiting > 0:
            response = ser.readline()
            # print(response)
            response = response.decode().strip()
            message_log.append(response)
            print("Arduino:", response)

            if response == 'AHOME':
                print("Response home")
                robot.estado = 'rest'
                pass
            if response == "ATGOTO" and robot.estado in ['goto', 'aprox']:
                # robot.estado = "rest"
                pass

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
                s = input("Press s to start, g for goto, h for home, f for find target, a for aprox: ")
                if s == 's':
                    robot.estado = 'homing'
                elif s == "g":
                    robot.estado = 'goto'
                elif s == "h":
                    robot.estado = 'homing'
                elif s == 'f':
                    robot.estado = 'find_target'
                elif s == 'a':
                    robot.estado = 'aprox'

            elif robot.estado == 'homing':
                robot.home()
                skip_homing = False

                pos = robot.forward_kinematics(robot.q)
                print(pos)

            elif robot.estado == 'find_target':
                if init_find:
                    find_time = time.time()
                    robot.goto(p.homing_angle_1, p.homing_angle_2, target_angle)

                init_find = False
                robot.best_angle = None
                if time.time() - find_time > 2:
                    if robot.best_angle:
                        print('Listo con target')
                        robot.estado = 'rest'
                        robot.pos_target = robot.infer_target()
                        print('altura hoyo:', robot.pos_target[1][0])
                        robot.pos_aprox = robot.aprox_geometry()
                    else:

                        if target_angle < robot.max_angle + 1:
                            robot.goto(p.homing_angle_1, p.homing_angle_2, target_angle)

                            ret, img = cap.read()
                            
                            if ret:
                                result = vision.get_diff_y(img)
                                
                                if result:
                                    diff_y = result['diff_y']
                                    robot.diff_list.append([target_angle, np.abs(diff_y)])
                            target_angle += 1
                        else:
                            df = pd.DataFrame(robot.diff_list, columns=['angle', 'diff_y'])
                            df = df.sort_values(by='diff_y', ascending=True)
                            df.to_csv('diff_list.csv', index=False)

                            robot.best_angle = int(df.iloc[0]['angle']) - 0     # -1
                            robot.pos_target = robot.infer_target()
                            print('altura hoyo desde suelo:', np.round(robot.pos_target[1][0], 3) + p.height_eje1)
                            robot.pos_aprox = robot.aprox_geometry()

                            robot.goto(p.homing_angle_1, p.homing_angle_2, robot.best_angle)

                            print('Listo con target')
                            robot.estado = 'rest'
            
            elif robot.estado == 'goto':
                robot.goto(p.homing_angle_1, p.homing_angle_2 + 30, 0)
                

            elif robot.estado == 'aprox':
                q = robot.inverse_kinematics(robot.pos_aprox)
                q1, q2, q3 = np.rad2deg(q[0][0]), np.rad2deg(q[1][0]), int(np.rad2deg(q[2][0]))
                q1 = (q1 + 180) % 360 - 180
                q1 = np.round(q1, 2)
                q2 += 180
                q2 = (q2 + 180) % 360 - 180
                q2 = np.round(q2, 2)
                q3 = q1+q2-180+q3
                q3 = (q3 + 180) % 360 - 180
                q3 = int(q3)
                print(q1, q2, q3)
                p_final = robot.forward_kinematics(q)
                print('x:', np.round(p_final[0][0], 3), 'y:', np.round(p_final[1][0] + p.height_eje1, 3))
                input('confirm: ')
                q1 += 5         # correccion M1
                q3 += 5         # correccion M1
                robot.goto(q1, q2, q3)
                robot.estado = 'correct'

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

            # if len(message_log) > 0: 
            #     print(message_log[-1])