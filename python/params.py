import numpy as np

dt = 1 / 2 # Vel loop principal


###### Comunicacion serial ######
## ls /dev/tty.*
port = "/dev/tty.usbmodem1301"
#port = "COM3"
baud_rate = 115200


###### Cinematica ######
l1 = 0.44   # MEDIR
l2 = 0.275    # MEDIR
r1 = 0.023 # MEDIR
r2 = 0.067 # MEDIR
r3 = 0.185
r2_cam = r2 + 0.03
r3_cam = r3 - 0.11

d_tot = 0.71 # eje a hoyo
height_eje1 = 0.238
# Lookup table
lup_a = 0.8934 # pendiente
lup_b = 6.2105 # intercept

# angulos homing: angulos absolutos a los que queda el brazo al hacer homing
homing_angle_1 = 81.80  # desde horizontal hacia arriba
homing_angle_2 = 55.00  # desde eslabon 1 a eslabon 2   

# modo manual
manual = False