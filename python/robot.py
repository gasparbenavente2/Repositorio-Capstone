import numpy as np
import params as p
import time
import pygame

class Robot:
    def __init__(self, serial_port):
        self.l1 = p.l1
        self.l2 = p.l2
        self.l_p_1 = p.l_p_1
        self.l_p_2 = p.l_p_2    # VER calculos_imagenes/cinematica.JPG para mas info

        self.q = np.array([[p.homing_angle_1], [p.homing_angle_2 - np.pi], [np.pi/2]])  # theta_1, theta_2, theta_3 ver imagen
        self.pos_e_brazo = np.array([[0], [0]]) # Posicion absoluta extremo brazo
        self.pos_e_pistola = np.array([[0], [0]])   # Posicion absoluta extremo pistola

        self.serial = serial_port
        self.estado = 'rest'            # rest, homing, find_target, aprox, correct, insert, trigger, exit
        self.update_pos()

    
    def forward_kinematics(self, q:np.array):
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
    
    def update_pos(self):
        self.p1 = self.forward_kinematics(self.q)[0]
        self.p2 = self.forward_kinematics(self.q)[1]
    
    def jacobiano(self, q: np.array):
        """Recibe los ángulos q1 y q2 (en grados) y retorna la matriz jacobiana de la funcion fkine. Para
        calculo de cinemática inversa"""
        q1 = q[0][0]
        q2 = q[1][0]
        l1 = self.l1
        l2 = self.l2
        sinq1 = np.sin(q1)
        cosq1 = np.cos(q1)
        sinq12 = np.sin(q1 + q2)
        cosq12 = np.cos(q1 + q2)

        jq = np.array([
            [(-l1 * sinq1 - l2 * sinq12), -l2 * sinq12],
            [(l1 * cosq1 + l2 * cosq12), l2 * cosq12]
        ])
        return jq
    
    
    def inverse_kinematics(self, p_target: np.array):
        start_time = time.time()
        iter_lim = 10000
        precision = 1e-6
        # qk = np.array([[np.arctan2(p_target[1][0], p_target[0][0])], [-1]])   # q0. Punto de partida del algoritmo
        qk = self.q[:2]
        k = 0   # num of iterations
        stop_flag = False

        while stop_flag is not True:
            pk = self.forward_kinematics(qk)[1]  # pos con config actual 
            p_delta = p_target - pk # error de posicion

            J = self.jacobiano(qk)
            # print(np.dot(np.linalg.pinv(J), p_delta))
            p_delta = (iter_lim - k)/iter_lim * p_delta
            qknew = (qk + np.dot(np.linalg.pinv(J), p_delta)) % (np.pi * 2)# calculo de nuevo intento de config
            pknew = self.forward_kinematics(qknew)[1]    # calculo de nueva pos con nueva config

            error = np.linalg.norm(p_target - pknew)
            if error < precision:
                stop_flag = True
                end_time = time.time()
                # print(f'Configuarcion encontrada en {k} iteraciones en {end_time - start_time} segundos')
                self.q = qknew 
                self.update_pos()
                return qknew
            elif k >= iter_lim:
                # print(f"más de {iter_lim} iteraciones")
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
                p1 = self.forward_kinematics(q)[0]
                p2 = self.forward_kinematics(q)[1]

                # extract coordinates (handles column vectors)
                x0, y0 = 0.0, 0.0
                x1, y1 = float(p1[0][0]), float(p1[1][0])
                x2, y2 = float(p2[0][0]), float(p2[1][0])

                # scale (pixels per meter) — allow override in params
                scale = getattr(p, "SCALE", getattr(p, "pixels_per_meter", 300))

                # origin at center of surface, flip y for screen coords
                cx, cy = width // 2, height // 2

                def to_screen(x, y):
                    return (int(cx + x * scale), int(cy - y * scale))

                o_pt = to_screen(x0, y0)
                p1_pt = to_screen(x1, y1)
                p2_pt = to_screen(x2, y2)

                # clear
                screen.fill(bg)

                # draw links
                pygame.draw.line(screen, (0, 0, 0), o_pt, p1_pt, 4)    # base -> joint1
                pygame.draw.line(screen, (0, 0, 255), p1_pt, p2_pt, 4) # joint1 -> end effector

                # draw joints/endpoint
                pygame.draw.circle(screen, (200, 0, 0), o_pt, 6)   # base
                pygame.draw.circle(screen, (0, 200, 0), p1_pt, 6)  # joint1
                pygame.draw.circle(screen, (0, 0, 200), p2_pt, 6)  # end effector

                # draw vertical line 0.5 meters from the origin from height 0 to 1
                line_start = to_screen(0.5, 0.1)
                line_end = to_screen(0.5, 0.4)
                pygame.draw.line(screen, (128, 128, 128), line_start, line_end, 2)

                pygame.display.flip()
                clock.tick(fps)
        finally:
            pygame.quit()
        

if __name__ == "__main__":
    robot = Robot()
    # q_test = robot.inverse_kinematics(np.array([[0.5],[0.1]]))
    # q_test = robot.inverse_kinematics(np.array([[0.5],[0.4]]))


    # q_test = np.array([[np.pi / 4], [-np.pi / 2]])
    #print(robot.forward_kinematics(q_test))
    # robot.draw(robot.q)
    print(np.rad2deg(p.homing_angle_2) + 40)
    print(robot.correccion_theta2(np.rad2deg(p.homing_angle_2) + 40))