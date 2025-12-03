import numpy as np
import params as p
import time
import pygame

class Robot:
    def __init__(self, serial_port, message_log):
        self.l1 = p.l1
        self.l2 = p.l2
        self.r1 = p.r1
        self.r2 = p.r2  # VER calculos_imagenes/cinematica.JPG para mas info
        self.r3 = p.r3
        self.r2_cam = p.r2_cam
        self.r3_cam = p.r3_cam

        self.q = np.array([[np.deg2rad(p.homing_angle_1)], [np.deg2rad(p.homing_angle_2) - np.pi], [np.deg2rad(0)]])  # theta_1, theta_2, theta_3 ver imagen
        self.pos_e_brazo = np.array([[0], [0]]) # Posicion absoluta extremo brazo
        self.pos_e_pistola = np.array([[0], [0]])   # Posicion absoluta extremo pistola

        # Rangos de operación validos
        self.q_min = np.array([[np.deg2rad(20)], [np.deg2rad(p.homing_angle_2 - 180)], [-50]])
        self.q_max= np.array([[np.deg2rad(90)], [np.deg2rad(30)], [50]])

        self.serial = serial_port
        self.estado = 'home'            # rest, homing, find_target, aprox, correct, insert, trigger, exit

        # find_target
        self.diff_list = []
        self.min_angle = p.min_search_angle
        self.max_angle = p.max_search_angle
        self.best_angle = None # grados respecto a horizontal
        self.pos_target = None
        self.altura_hoyo_suelo = None

        # aprox
        self.pos_aprox = None

        # correct
        self.pos_correct = None

        # enchufar
        self.pos_enchufe = None

        self.message_log = message_log

        self.q_pos_correct = None

        self.q_pos_enchufar = None

        self.update_pos()
    
    def goto(self, q1, q2, q3):
        ''' Angulos en grados, q2 medido desde abajo. Lenguaje robot'''
        q1 = np.clip(q1, 0, p.homing_angle_1)
        q2 = np.clip(q2, p.homing_angle_2, 150)
        q3 = np.clip(q3, -50, 50)

        formatted_q1 = f"{q1:06.2f}"  # total width 6: 3 digits + dot + 2 decimals
        formatted_q2 = f"{q2:06.2f}"
        formatted_q3 = f"{q3:+04d}"

        msg = f"AGOTO {formatted_q1} {formatted_q2} {formatted_q3};"
        msg = msg.encode("utf-8")
        print(f"Enviado: {msg}")
        self.message_log.append(f"Enviado: {msg}")
        self.serial.write(msg)

    def goto_servo(self, q1, q2, q3):
        ''' Angulos en grados, q2 medido desde abajo. Lenguaje robot'''
        q1 = np.clip(q1, 0, p.homing_angle_1)
        q2 = np.clip(q2, p.homing_angle_2, 150)
        q3 = np.clip(q3, -50, 50)

        formatted_q1 = f"{q1:06.2f}"  # total width 6: 3 digits + dot + 2 decimals
        formatted_q2 = f"{q2:06.2f}"
        formatted_q3 = f"{q3:+04d}"

        msg = f"ASRVO {formatted_q1} {formatted_q2} {formatted_q3};"
        msg = msg.encode("utf-8")
        # print(f"Enviado: {msg}")
        self.message_log.append(f"Enviado: {msg}")
        self.serial.write(msg)
    
    def anguloreal2anguloarduino(self, q):
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

        return (q1, q2, q3)

    def home(self):
        msg = "AHOME;"
        msg = msg.encode("utf-8")
        self.serial.write(msg)
    
    def gatillo(self):
        msg = "ATRGR;"
        msg = msg.encode("utf-8")
        self.serial.write(msg)

    def infer_target(self):
        """ encuentra posicion de target. best angle en grados respecto horizontal """
        q1 = np.deg2rad(p.homing_angle_1)
        q2 = np.deg2rad(p.homing_angle_2) - np.pi
        q3 = np.deg2rad(self.best_angle) - q1 - q2
        p1 = self.forward_kinematics_camara(np.array([[q1], [q2], [q3]]))

        print('pos x camara:', p1[0][0], 'pos y camara:', p1[1][0])

        px = p1[0][0]
        py = p1[1][0]

        y = np.tan(np.deg2rad(self.best_angle)) * (p.d_tot - px)
        height_hoyo = py + y
        print(height_hoyo + p.height_eje1)

        return np.array([[p.d_tot], [height_hoyo + p.bias_angulo_camara]])          # correccion camara

    def aprox_geometry(self):
        """ encuentra posicion de aproximacion segun target """
        return np.array([[self.pos_target[0][0] - 0.05], [self.pos_target[1][0] - 0.03], [np.deg2rad(-30)]])
    
    def correccion(self, top_y):
        """calcula cm a partir de diferencia en pixeles"""
        return (-top_y/100 + 5.8)/100

    def enchufar(self, ):
        deltax, deltay = self.altura_hoyo_suelo/8 + 1.5/100, -0.01
        print('delta x', deltax)
        self.pos_enchufe = self.pos_correct

        self.pos_enchufe[0][0] += deltax
        self.pos_enchufe[1][0] += deltay

        q = self.inverse_kinematics(self.pos_enchufe)
        q1, q2, q3 = self.anguloreal2anguloarduino(q)

        return (q1, q2, q3)
    
    def forward_kinematics_legacy(self, q:np.array):
        """Calcula cinematica directa de extremo de brazo y retorna la pos absoluta. 
        Adaptado de Unidad 1 Fundamentos de robotica"""
        q1 = q[0][0]
        q2 = q[1][0] # OJO este es theta_2 PRIMA

        R1 = np.array(
            [[np.cos(q1), -1 * np.sin(q1)],
             [np.sin(q1), np.cos(q1)]])

        R2 =  np.array(
            [[np.cos(q1 + q2), -1 * np.sin(q1 + q2)],
             [np.sin(q1 + q2), np.cos(q1 + q2)]])
        
        p1 = np.array([[self.l1], [0]])    #marco F1
        p2 = np.array([[self.l2], [0]])    #marco F2
        
        p1_f0 = np.dot(R1, p1)
        p2_f0 = np.dot(R2, p2) + p1_f0

        return [p1_f0, p2_f0]
    
    def forward_kinematics(self, q:np.array) -> np.array:
        """Entra q1, q2, q3, sale x, y, phi (angulo abs pistola)"""

        q1 = q[0][0]
        q2 = q[1][0] # OJO este es theta_2 PRIMA
        q3 = q[2][0]

        l1, l2 = self.l1, self.l2
        r1, r2, r3 = self.r1, self.r2, self.r3

        c1, s1 = np.cos(q1), np.sin(q1)
        c12, s12 = np.cos(q1 + q2), np.sin(q1 + q2)
        c123, s123 = np.cos(q1 + q2 + q3), np.sin(q1 + q2 + q3)

        px = l1*c1 + l2*c12 + r3*c123 - r1*s12 - r2*s123
        py = r1*c12 + r2*c123 + l1*s1 + l2*s12 + r3*s123
        phi = q1 + q2 + q3

        # phi = ((phi + 2 * np.pi) % (2 * np.pi)) - 2 * np.pi

        return np.array([[px], [py], [phi]])
    
    def forward_kinematics_camara(self, q:np.array) -> np.array:
        """Entra q1, q2, q3, sale x, y, phi (angulo abs pistola)"""

        q1 = q[0][0]
        q2 = q[1][0] # OJO este es theta_2 PRIMA
        q3 = q[2][0]

        l1, l2 = self.l1, self.l2
        r1, r2, r3 = self.r1, self.r2_cam, self.r3_cam

        c1, s1 = np.cos(q1), np.sin(q1)
        c12, s12 = np.cos(q1 + q2), np.sin(q1 + q2)
        c123, s123 = np.cos(q1 + q2 + q3), np.sin(q1 + q2 + q3)

        px = l1*c1 + l2*c12 + r3*c123 - r1*s12 - r2*s123
        py = r1*c12 + r2*c123 + l1*s1 + l2*s12 + r3*s123
        phi = q1 + q2 + q3

        # phi = ((phi + 2 * np.pi) % (2 * np.pi)) - 2 * np.pi

        return np.array([[px], [py], [phi]])

        
    def update_pos(self):
        pass
        self.p1 = self.forward_kinematics(self.q)[0]
        self.p2 = self.forward_kinematics(self.q)[1]
    
    def jacobiano(self, q: np.array):
        """Recibe los ángulos q1, q2 y q3(en grados) y retorna la matriz jacobiana de la funcion fkine. Para
        calculo de cinemática inversa"""

        q1 = q[0][0]
        q2 = q[1][0] 
        q3 = q[2][0]

        l1, l2 = self.l1, self.l2
        r1, r2, r3 = self.r1, self.r2, self.r3

        c1, s1 = np.cos(q1), np.sin(q1)
        c12, s12 = np.cos(q1 + q2), np.sin(q1 + q2)
        c123, s123 = np.cos(q1 + q2 + q3), np.sin(q1 + q2 + q3)

        J00 = -(r1*c12) - r2*c123 - l1*s1 - l2*s12 - r3*s123
        J01 = -(r1*c12) - r2*c123 - l2*s12 - r3*s123
        J02 = -(r2*c123) - r3*s123

        J10 = l1*c1 + l2*c12 + r3*c123 - r1*s12 - r2*s123
        J11 =  l2*c12 + r3*c123 - r1*s12 - r2*s123
        J12 = r3*c123 - r2*s123

        J20 = 1
        J21 = 1
        J22 = 1

        return np.array([[J00, J01, J02],
                        [J10, J11, J12],
                        [J20, J21, J22]])

    
    
    def inverse_kinematics(self, p_target: np.array):
        start_time = time.time()
        iter_lim = 1000
        precision = 1e-6
        # qk = self.
        qk = np.array([[np.deg2rad(p.homing_angle_1)], [np.deg2rad(p.homing_angle_2) - np.pi], [np.deg2rad(-30)]])
        k = 0   # num of iterations
        stop_flag = False

        while stop_flag is not True:
            pk = self.forward_kinematics(qk)  # pos con config actual 
            # print(pk)
            p_delta = p_target - pk # error de posicion
            p_delta[2][0] = (p_delta[2][0] + np.pi) % (2*np.pi) - np.pi

            J = self.jacobiano(qk)
            # print(np.dot(np.linalg.pinv(J), p_delta))
            p_delta = (iter_lim - k)/iter_lim * p_delta
            qknew = (qk + np.dot(np.linalg.pinv(J), p_delta)) % (np.pi * 2)# calculo de nuevo intento de config
            # qknew = np.clip(qknew, self.q_min, self.q_max)
            pknew = self.forward_kinematics(qknew)# calculo de nueva pos con nueva config

            error = np.linalg.norm(p_target[:2, :] - pknew[:2, :]) +  np.abs(p_target[2, :] - ((pknew[2, :] + np.pi) % (2*np.pi) - np.pi))
            # error = np.linalg.norm(p_target - pknew)
            #print(error)
            # print(p_target - pknew)
            # print(f"p_target: {p_target}, pknew: {pknew}, error: {error}")
            if error < precision:
                stop_flag = True
                end_time = time.time()
                print(f'Configuarcion encontrada en {k} iteraciones en {end_time - start_time} segundos')
                self.q = qknew 
                # self.update_pos()
                return qknew
            elif k >= iter_lim:
                print(f"más de {iter_lim} iteraciones")
                self.q = qknew
                self.update_pos()
                return qknew
            else:
                # print(f'Iteración {k}.')
                # print(f'Posición actual: {pknew[0][0]}, {pknew[1][0]}')
                # print(f'Ángulo actual: {np.degrees(qknew[0][0])}°, {np.degrees(qknew[1][0])}°')
                # print('-'*70)
                k += 1
                qk = np.copy(qknew)
            pass
    


    def inverse_kinematics_pistola(self, p_target: np.array):
        """ toma en cuenta pistola"""
        theta4 = np.deg2rad(23) + self.q[2][0]
        unit_theta4 = np.array([[np.cos(theta4)], [np.sin(theta4)]])
        pistola_offset = 0.2061 * unit_theta4

        q_robot = self.q[:2]

        for _ in range(4):
            theta_tip = q_robot[0][0] + q_robot[1][0]
            perp = np.array([[-np.sin(theta_tip)], [np.cos(theta_tip)]])
            axis_offset = 0.03 * perp
            p_wrist = p_target - axis_offset - pistola_offset
            q_robot = self.inverse_kinematics(p_wrist)

        q_pistol = q_robot.copy()
        q_pistol[1][0] += np.pi/2

        return q_pistol

    def correccion_theta2(self, q2_desired: float) -> float:
        """Este medoto recibe el angulo real al que debería ir el brazo
        y retorna el angulo que se debería comandar al encoder dada la flección (best fit line)"""
        # theta_real = a * theta_encoder + b
        # theta_enc_cmd = (theta_deseado - b) / a

        a, b = p.lup_a, p.lup_b

        return round((q2_desired - b) / a, 2)

    def draw(self, q:np.array):
        # initialize pygame and create window (uses params if available)
        pygame.init()
        width = getattr(p, "WIDTH", 800)
        height = getattr(p, "HEIGHT", 600)
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption(getattr(p, "WINDOW_TITLE", "Robot Simulator"))
        clock = pygame.time.Clock()
        fps = getattr(p, "FPS", 30)
        bg = getattr(p, "BG_COLOR", (255, 255, 255))

        try:
            running = True
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False

                # forward kinematics
                p1 = self.forward_kinematics_legacy(q)[0]
                p2 = self.forward_kinematics_legacy(q)[1]
                p_pistol = self.forward_kinematics(q)

                # extract coordinates (handles column vectors)
                x0, y0 = 0.0, 0.0
                x1, y1 = float(p1[0][0]), float(p1[1][0])
                x2, y2 = float(p2[0][0]), float(p2[1][0])
                xp, yp = float(p_pistol[0][0]), float(p_pistol[1][0])

                # scale (pixels per meter) — allow override in params
                scale = getattr(p, "SCALE", getattr(p, "pixels_per_meter", 300))

                # origin at center of surface, flip y for screen coords
                cx, cy = width // 2, height // 2

                def to_screen(x, y):
                    return (int(cx + x * scale), int(cy - y * scale))

                o_pt = to_screen(x0, y0)
                p1_pt = to_screen(x1, y1)
                p2_pt = to_screen(x2, y2)
                pistol_pt = to_screen(xp, yp)

                # clear
                screen.fill(bg)

                # draw links
                pygame.draw.line(screen, (0, 0, 0), o_pt, p1_pt, 4)    # base -> joint1
                pygame.draw.line(screen, (0, 0, 255), p1_pt, p2_pt, 4) # joint1 -> end effector
                pygame.draw.line(screen, (255, 0, 255), p2_pt, pistol_pt, 2) # end effector -> pistol tip

                # draw joints/endpoint
                pygame.draw.circle(screen, (200, 0, 0), o_pt, 6)   # base
                pygame.draw.circle(screen, (0, 200, 0), p1_pt, 6)  # joint1
                pygame.draw.circle(screen, (0, 0, 200), p2_pt, 6)  # end effector
                pygame.draw.circle(screen, (255, 0, 0), pistol_pt, 4) # pistol tip

                # draw vertical line 0.5 meters from the origin from height 0 to 1
                line_start = to_screen(0.5, 0.1)
                line_end = to_screen(0.5, 0.4)
                pygame.draw.line(screen, (128, 128, 128), line_start, line_end, 2)

                pygame.display.flip()
                clock.tick(fps)
        finally:
            pygame.quit()
    
    

        

if __name__ == "__main__":
    robot = Robot(None, None)

    # q1 = np.deg2rad(p.homing_angle_1)
    # q2 = np.deg2rad(p.homing_angle_2) - np.pi
    # q3 = np.deg2rad(-30) - q1 - q2

    # print(f"q1: {np.rad2deg(q1)}")
    # print(f"q2: {np.rad2deg(q2)}")
    # print(f"q3: {np.rad2deg(q3)}")

    # fk_test = robot.forward_kinematics(np.array([[q1],[q2],[q3]]))
    # print(fk_test)

    # x = fk_test[0][0] + 0
    # y = fk_test[1][0] - 0
    # phi = np.deg2rad(-30)


    # test_inv = robot.inverse_kinematics(np.array([[x], [y], [phi]]))
    # print(test_inv)
    # print(robot.forward_kinematics(test_inv))

    # robot.draw(test_inv)

    # jacob = robot.jacobiano(robot.q)
    #print(jacob.shape)

    robot.infer_target()