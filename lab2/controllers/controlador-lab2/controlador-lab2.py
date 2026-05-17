"""controlador-lab2 controller."""

from controller import Robot
from math import sin, cos

# Inicializacion
robot: Robot = Robot()
timestep = int(robot.getBasicTimeStep())
motor_rueda_izq = robot.getDevice("left wheel motor")
motor_rueda_izq.setPosition(float("inf"))
motor_rueda_izq.setVelocity(3)
motor_rueda_der = robot.getDevice("right wheel motor")
motor_rueda_der.setPosition(float("inf"))
motor_rueda_der.setVelocity(3)
dist_front_izq = robot.getDevice("ps7")
dist_front_izq.enable(timestep)
dist_front_der = robot.getDevice("ps0")
dist_front_der.enable(timestep)
dist_lat_izq = robot.getDevice("ps5")
dist_lat_izq.enable(timestep)
dist_lat_der = robot.getDevice("ps2")
dist_lat_der.enable(timestep)
encoder_rueda_izq = robot.getDevice("left wheel sensor")
encoder_rueda_izq.enable(timestep)
encoder_rueda_der = robot.getDevice("right wheel sensor")
encoder_rueda_der.enable(timestep)

# En metros
radio_ruedas: float = 0.0205
distancia_ejes: float = 0.052
velocidad_max: float = 6.28

pos: dict = {
    "theta": 0.0,
    "actual": 0.0,
    "x": 0.0,
    "y": 0.0,
    "encoder_izq_previo": 0.0,
    "encoder_der_previo": 0.0,
}

def calcular_avance_ruedas(pos: dict, pos_actual_izq: float, pos_actual_der: float) -> None:
    delta_izq: float = pos_actual_izq - pos["encoder_izq_previo"]
    delta_der: float = pos_actual_der - pos["encoder_der_previo"]
    avance_rueda_izq: float = delta_izq*radio_ruedas
    avance_rueda_der: float = delta_der*radio_ruedas
    avance: float = (avance_rueda_izq + avance_rueda_der) / 2
    delta_theta: float = (avance_rueda_der - avance_rueda_izq) / distancia_ejes

    pos["actual"] += avance
    pos["theta"] += delta_theta
    pos["x"] += avance*cos(pos["theta"])
    pos["y"] += avance*sin(pos["theta"])

while robot.step(timestep) != -1:
    print(pos)
    pos_encoder_izq: float = encoder_rueda_izq.getValue()
    pos_encoder_der: float = encoder_rueda_der.getValue()
    
    calcular_avance_ruedas(pos, pos_encoder_izq, pos_encoder_der)

    pos["encoder_izq_previo"] = pos_encoder_izq
    pos["encoder_der_previo"] = pos_encoder_der