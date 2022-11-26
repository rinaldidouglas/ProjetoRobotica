from controller import Robot
from controller import DistanceSensor
from controller import Motor

MAX_SPEED = 6.28
TIME_STEP = 64

robot = Robot()

# Inicializando sensores [imagem com sua representação no relatório]

ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

# Atribuindo cada motor e velocidade para o e-puck

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

while robot.step(TIME_STEP) != -1:

    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # Possibilita os sensores a detectarem os obstáculos presentes no mundo

    right_obstacle = psValues[2] > 80.0 or psValues[5] > 80.0
    right_too_close = psValues[1] > 80.0
    front_obstacle = psValues[7] > 80.0 or psValues[0] > 80.0

    # Modificador de velocidade de acordo com os obstáculos encontrados

    leftSpeed  = 0.7 * MAX_SPEED
    rightSpeed = 0.7 * MAX_SPEED
    
    if front_obstacle:

        # Vira para ESQUERDA

        leftSpeed  = -0.7 * MAX_SPEED
        rightSpeed = 0.7 * MAX_SPEED
    
    else:

        if right_obstacle:

            # Anda para FRENTE

            leftSpeed  = 1.0 * MAX_SPEED
            rightSpeed = 1.0 * MAX_SPEED
        
        else:

            # Vira para DIREITA

            leftSpeed  = 1.0 * MAX_SPEED
            rightSpeed = 0.4 * MAX_SPEED
        
        if right_too_close:

            # Caso esteja muito próximo à parede, o robô irá evitá-la

            leftSpeed  = 0.4 * MAX_SPEED
            rightSpeed = 1.0 * MAX_SPEED
 
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)