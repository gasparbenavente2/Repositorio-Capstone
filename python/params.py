import numpy as np

dt = 1 / 1 # Vel loop principal


# Comunicacion seriañ
port = "COM3"
baud_rate = 115200


# Cinematica
l1 = 0.4   # MEDIR
l2 = 0.3    # MEDIR
l_p_1 = 0.1 # MEDIR
l_p_2 = 0.1 # MEDIR


# angulos homing: angulos absolutos a los que queda el brazo al hacer homing
homing_angle_1 = np.deg2rad(80.03)  # desde horizontal hacia arriba
homing_angle_2 = np.deg2rad(64.95)  # desde eslabon 1 a eslabon 2   