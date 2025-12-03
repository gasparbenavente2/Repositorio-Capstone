import numpy as np

dt = 1 / 2 # Vel loop principal


###### Comunicacion serial ######
## ls /dev/tty.*
port = "/dev/tty.usbmodem11301"
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
homing_angle_1 = 81.10  # desde horizontal hacia arriba
homing_angle_2 = 60.80  # desde eslabon 1 a eslabon 2   

# modo manual
manual = False
partir_clear = True


# Busqueda Hoyo
max_search_angle = 0
min_search_angle = -55
bias_angulo_camara = 0.03   # Error sistematico en altura encontrada gracias a diferencia angulo camara