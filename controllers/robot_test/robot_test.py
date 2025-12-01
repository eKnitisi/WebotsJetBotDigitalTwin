from controller import Robot, Keyboard

# 1. Initialiseer de robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Haal de motoren op
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')

# Zet motoren in velocity mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ==========================================
# 3. CAMERA INSTELLEN (Nieuw!)
# ==========================================
# De naam 'camera' moet overeenkomen met de naam in je PROTO bestand
camera = robot.getDevice('camera')

# Zet de camera aan. Hij maakt elke 'timestep' een nieuwe foto.
camera.enable(timestep)

# ==========================================

# 4. Toetsenbord en variabelen
keyboard = Keyboard()
keyboard.enable(timestep)
MAX_SPEED = 5.0 

print("Klaar! Camera is actief. Gebruik pijltjestoetsen om te rijden.")

# 5. De Main Loop
while robot.step(timestep) != -1:
    # Hier kun je eventueel de camera-data ophalen om er iets mee te doen:
    # image = camera.getImage()
    
    key = keyboard.getKey()
    left_speed = 0.0
    right_speed = 0.0
    
    if key == Keyboard.UP:
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
    elif key == Keyboard.DOWN:
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED
    elif key == Keyboard.LEFT:
        left_speed = -MAX_SPEED * 0.5
        right_speed = MAX_SPEED * 0.5
    elif key == Keyboard.RIGHT:
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)