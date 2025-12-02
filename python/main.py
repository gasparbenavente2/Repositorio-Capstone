import serial
import cv2
import time
import params as p
import robot 
import vision
import pandas as pd
import numpy as np
from vision import corregir

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
    target_angle = robot.min_angle
    aprox_done = False
    correct_done = False
    enchufar_done = False

    old_time = time.time()
    while True:
        if ser.in_waiting > 0:
            response = ser.readline()
            response = response.decode().strip()
            message_log.append(response)
            print("Arduino:", response)

            if response == 'AHOME':
                print("Response home")
                if p.manual:
                    robot.estado = 'rest'
                else:
                    robot.estado = 'find_target'
            if "ATGOTO" in response and robot.estado == 'standby':
                robot.estado = 'rest'
            if not p.manual:
                if "ATGOTO" in response and robot.estado == 'aprox':
                    robot.estado = 'correct'
                elif "ATGOTO" in response and robot.estado == 'correct':
                    robot.estado = 'enchufar'
                elif "ATGOTO" in response and robot.estado == 'enchufar':
                    robot.estado = 'trigger'

        current_time = time.time()
        if current_time >= old_time + p.dt:
            old_time = time.time()

            if robot.estado == 'rest':
                s = input("Press s to start, g for goto, h for home, f for find target, a for aprox, c for correct, e for enchufar, t for trigger: ")
                if s == "g":
                    robot.estado = 'goto'
                elif s == "h":
                    robot.estado = 'homing'
                elif s == 'f':
                    robot.estado = 'find_target'
                elif s == 'a':
                    robot.estado = 'aprox'
                elif s == 'c':
                    robot.estado = 'correct'
                elif s == 'e':
                    robot.estado = 'enchufar'
                elif s == 't':
                    robot.estado = 'trigger'

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
                if time.time() - find_time > 4:
                    if robot.best_angle:
                        print('Listo con target')
                        robot.estado = 'rest'
                        robot.pos_target = robot.infer_target()
                        print('altura hoyo:', robot.pos_target[1][0])
                        robot.pos_aprox = robot.aprox_geometry()
                    else:

                        if target_angle < robot.max_angle + 1:
                            robot.goto_servo(p.homing_angle_1, p.homing_angle_2, target_angle)

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
                            robot.altura_hoyo_suelo = np.round(robot.pos_target[1][0], 3) + p.height_eje1
                            print('altura hoyo desde suelo:', robot.altura_hoyo_suelo)
                            robot.pos_aprox = robot.aprox_geometry()

                            robot.goto(p.homing_angle_1, p.homing_angle_2, robot.best_angle)

                            print('Listo con target')
                            if p.manual:
                                robot.estado = 'rest'
                            else:
                                robot.estado = 'aprox'
            
            elif robot.estado == 'goto':
                robot.goto(p.homing_angle_1 + 0, p.homing_angle_2 + 20, 0) 
                robot.estado = 'standby'

            elif robot.estado == 'aprox' and not aprox_done:
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
                # input('confirm: ')
                q1 += 5         # correccion M1
                q3 += 5         # correccion M1
                robot.goto(q1, q2, q3)
                aprox_done = True
                if p.manual:
                    robot.estado = 'standby'

            elif robot.estado == 'correct' and not correct_done:
                ret, img = cap.read()
                print('correct')
                            
                if ret:
                    top_y = corregir(img)

                    delta = robot.correccion(top_y)

                    robot.pos_correct = robot.pos_aprox
                    robot.pos_correct[1][0] += delta
                    print('nueva pos:', robot.pos_correct)
                    print('mover:', delta, 'cm')
                    # input('confirmar:')
                    q = robot.inverse_kinematics(robot.pos_correct)
                    q1, q2, q3 = np.rad2deg(q[0][0]), np.rad2deg(q[1][0]), int(np.rad2deg(q[2][0]))
                    q1 = (q1 + 180) % 360 - 180
                    q1 = np.round(q1, 2)
                    q2 += 180
                    q2 = (q2 + 180) % 360 - 180
                    q2 = np.round(q2, 2)
                    q3 = q1+q2-180+q3
                    q3 = (q3 + 180) % 360 - 180
                    q3 = int(q3)
                    q1 += 5         # correccion M1
                    q3 += 5         # correccion M1

                    robot.goto(q1, q2, q3)
                    correct_done = True
                    if p.manual:
                        robot.estado = 'standby'
                else:
                    print('no funciona camara')

            elif robot.estado == 'enchufar' and not enchufar_done:
                q1, q2, q3 = robot.enchufar()
                # input('confirmar:')

                robot.goto(q1, q2, q3)

                enchufar_done = True

                if p.manual:
                    robot.estado = 'standby'
                
            elif robot.estado == 'trigger':
                robot.gatillo()
                robot.estado = 'rest'
                
            elif robot.estado == 'exit':
                pass
            elif robot.estado == 'test':
                robot.goto(p.homing_angle_1-20, p.homing_angle_2+20, 100)
            
            elif robot.estado == 'standby':
                pass
            print("estado", robot.estado)

            # if len(message_log) > 0: 
            #     print(message_log[-1])