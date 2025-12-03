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
    cap = cv2.VideoCapture(0)
    target_angle = robot.min_angle

    init_find = True
    init_aprox = True
    init_correct = True
    init_enchufar = True
    init_trigger = True
    init_exit = True
    init_clear = True


    aprox_done = False
    correct_done = False
    enchufar_done = False
    exit_done = False
    clear_done = False

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
                    if p.partir_clear:
                        robot.estado = 'clear'
                    else:
                        robot.estado = 'find_target'

            if "ATGOTO" in response and robot.estado == 'standby':
                robot.estado = 'rest'
            if not p.manual:
                if "ATGOTO" in response and robot.estado == 'aprox':
                    robot.estado = 'correct'
                elif "ATGOTO" in response and robot.estado == 'clear':
                    input("Iniciar rutina?")
                    robot.estado = 'find_target'
                elif "ATGOTO" in response and robot.estado == 'correct':
                    robot.estado = 'enchufar'
                elif "ATGOTO" in response and robot.estado == 'enchufar':
                    robot.estado = 'trigger'
                elif "ATGOTO" in response and robot.estado == 'exit':
                    robot.estado = 'homing'

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
            
            elif robot.estado == 'clear' and not clear_done:
                "Robot se extiende completamente para requerimiento de 0.5 metros al auto"
                if init_clear:
                    clear_init_time = time.time()
                    robot.goto_servo(p.homing_angle_1, p.homing_angle_2, 95)
                    init_clear = False
                
                if time.time() - clear_init_time > 1.0:
                    robot.goto(p.homing_angle_1, p.homing_angle_2 + 95, 95)
                    clear_done = True

            elif robot.estado == 'find_target':
                if init_find:
                    find_time = time.time()
                    robot.goto(p.homing_angle_1, p.homing_angle_2, target_angle)

                init_find = False
                robot.best_angle = None
                if time.time() - find_time > 5:
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

                            # input(f'Target encontrado a altura {robot.altura_hoyo_suelo}. Continuar?')
                            if p.manual:
                                robot.estado = 'rest'
                            else:
                                robot.estado = 'aprox'
            
            elif robot.estado == 'goto':
                robot.goto(p.homing_angle_1 + 0, p.homing_angle_2 + 20, 0) 
                robot.estado = 'standby'

            elif robot.estado == 'aprox' and not aprox_done:
                if init_aprox:
                    aprox_time = time.time()
                    init_aprox = False

                if time.time() - aprox_time > 0:
                    q = robot.inverse_kinematics(robot.pos_aprox)
                    q1, q2, q3 = robot.anguloreal2anguloarduino(q)
                    # print(q1, q2, q3)
                    # p_final = robot.forward_kinematics(q)
                    # print('x:', np.round(p_final[0][0], 3), 'y:', np.round(p_final[1][0] + p.height_eje1, 3))
                    # input('confirm: ')
                    robot.goto(q1, q2, q3)
                    aprox_done = True
                    if p.manual:
                        robot.estado = 'standby'

            elif robot.estado == 'correct' and not correct_done:
                if init_correct:
                    correct_time = time.time()
                    init_correct = False
                
                if time.time() - correct_time > 0.5:
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
                        robot.q_pos_correct = q
                        q1, q2, q3 = robot.anguloreal2anguloarduino(q)

                        robot.goto(q1, q2, q3)
                        correct_done = True
                        if p.manual:
                            robot.estado = 'standby'
                    else:
                        print('no funciona camara')

            elif robot.estado == 'enchufar' and not enchufar_done:
                if init_enchufar:
                    enchufar_time = time.time()
                    init_enchufar = False

                if time.time() - enchufar_time > 0.5:
                    q1, q2, q3 = robot.enchufar()
                    robot.q_pos_enchufar = np.array([[q1], [q2], [q3]])
                    # input('confirmar:')

                    robot.goto(q1, q2, q3)

                    enchufar_done = True

                    if p.manual:
                        robot.estado = 'standby'
                
            elif robot.estado == 'trigger':
                if init_trigger:
                    trigger_time = time.time()
                    init_trigger = False

                if time.time() - trigger_time > 0.5:
                    robot.gatillo()
                    if p.manual:
                        robot.estado = 'standby'
                    else:
                        robot.estado = 'exit'
                
            elif robot.estado == 'exit' and not exit_done:
                if init_exit:
                    exit_time = time.time()
                    init_exit = False
                
                if time.time() - exit_time > 4:
                    #q = robot.q_pos_correct
                    #q1, q2, q3 = robot.anguloreal2anguloarduino(q)
                    q1_exit = robot.q_pos_enchufar[0][0]
                    q2_exit = robot.q_pos_enchufar[1][0]
                    q3_exit = robot.q_pos_enchufar[2][0]
                    robot.goto(q1_exit + 5, q2 - 4, q3)
                    exit_done = True
            
            elif robot.estado == 'test':
                robot.goto(p.homing_angle_1-20, p.homing_angle_2+20, 100)
            
            elif robot.estado == 'standby':
                pass
            print("estado", robot.estado)

            # if len(message_log) > 0: 
            #     print(message_log[-1])