from controller import Robot

# 1. Initialiseer Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Motoren instellen
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ==========================================
# 3. LIDAR & CAMERA INSTELLEN
# ==========================================
# Camera aanzetten (voor het geval je die ook wilt zien)
camera = robot.getDevice('camera')
camera.enable(timestep)

# Lidar aanzetten
lidar = robot.getDevice('lidar')
lidar.enable(timestep)

# Optioneel: Zet PointCloud aan (helpt soms bij visualisatie)
lidar.enablePointCloud()

# ==========================================

print("Lidar Test Gestart! Robot rijdt autonoom.")

MAX_SPEED = 5.0

while robot.step(timestep) != -1:
    # 4. LEES LIDAR DATA
    # getRangeImage() geeft een lijst terug met 720 getallen (afstanden)
    # index 0 is recht vooruit (omdat Lidar naar +X kijkt)
    range_image = lidar.getRangeImage()
    
    # Veiligheidscheck: soms is de lijst nog leeg bij de eerste stap
    if not range_image:
        continue

    # We kijken naar wat er RECHT VOOR ons is (index 0)
    # We pakken ook een paar graden links en rechts voor de zekerheid
    # (Bijv. het gemiddelde van de eerste 10 en laatste 10 punten)
    front_distance = range_image[0] 
    
    # Debug: Print de afstand in de console
    # "inf" betekent oneindig (geen obstakel gezien)
    print(f"Afstand tot muur: {front_distance:.2f} meter")

    # 5. OBSTAKEL LOGICA
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED
    
    # Als er iets dichterbij is dan 30 cm (0.3 meter)
    if front_distance < 0.3:
        print("Obstakel! Draaien...")
        # Draai naar rechts
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
    else:
        # Gewoon rechtdoor
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        
    # Stuur motoren aan
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)