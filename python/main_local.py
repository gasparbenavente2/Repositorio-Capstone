import cv2
import time
import params as p
import robot
import vision

ser = None


if __name__ == "__main__":
    print("Main start")
    robot = robot.Robot(ser)
    robot.estado = 'find_target'
    cap = cv2.VideoCapture(0)

    old_time = time.time()
    while True:
        current_time = time.time()
        if current_time >= old_time + p.dt:
            old_time = time.time()
            
            if robot.estado == 'rest':
                s = input("Press s to start: ")
                if s == 's':
                    robot.estado = 'homing'

            elif robot.estado == 'homing':
                if response == 'HOMING OK':
                    robot.estado = 'find_target'

            elif robot.estado == 'find_target':
                for angle in range(robot.min_angle, robot.max_angle + 1):
                    ret, img = cap.read()
                    
                    if ret:
                        result = vision.get_diff_y(img)
                        
                        if result:
                            diff_y = result['diff_y']
                            robot.diff_list.append([angle, diff_y])
                            print(f"Angle: {angle}, Diff Y: {diff_y}")

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