import numpy as np

dt = 1 / 2 # Vel loop principal


###### Comunicacion serial ######
## ls /dev/tty.*
port = "/dev/tty.usbmodem1201"
#port = "COM3"
baud_rate = 115200


###### Cinematica ######
l1 = 0.44   # MEDIR
l2 = 0.275    # MEDIR
r1 = 0.03 # MEDIR
r2 = 0.0804 # MEDIR
r3 = 0.1896

# Lookup table
lup_a = 0.8934 # pendiente
lup_b = 6.2105 # intercept

# angulos homing: angulos absolutos a los que queda el brazo al hacer homing
homing_angle_1 = 86.36  # desde horizontal hacia arriba
homing_angle_2 = 62.10  # desde eslabon 1 a eslabon 2   