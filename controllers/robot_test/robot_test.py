from controller import Robot, Keyboard

# 1. Initialiseer de robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Haal de motoren op (namen moeten matchen met je PROTO/URDF)
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')

# 3. Stel de motoren in op 'Velocity Mode'
# Door de positie op oneindig (inf) te zetten, kunnen de wielen blijven draaien
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Start met snelheid 0 (stilstand)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 4. Activeer het toetsenbord
keyboard = Keyboard()
keyboard.enable(timestep)

# Snelheid instelling (max is 15 volgens je URDF)
MAX_SPEED = 5.0 

print("Controller gestart! Gebruik de pijltjestoetsen om te rijden.")

# 5. De Main Loop
while robot.step(timestep) != -1:
    # Lees welke toets wordt ingedrukt
    key = keyboard.getKey()
    
    left_speed = 0.0
    right_speed = 0.0
    
    # Besturing logica
    if key == Keyboard.UP:
        # Vooruit: beide wielen vooruit
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        
    elif key == Keyboard.DOWN:
        # Achteruit: beide wielen achteruit
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED
        
    elif key == Keyboard.LEFT:
        # Links draaien: links achteruit, rechts vooruit
        left_speed = -MAX_SPEED * 0.5
        right_speed = MAX_SPEED * 0.5
        
    elif key == Keyboard.RIGHT:
        # Rechts draaien: links vooruit, rechts achteruit
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
        
    # Stuur de snelheden naar de motoren
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)