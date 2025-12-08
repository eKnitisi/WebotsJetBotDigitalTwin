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
# 3. SENSOREN INSTELLEN
# ==========================================
camera = robot.getDevice('camera')
camera.enable(timestep)

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

print("Robot gestart met verbeterde detectie!")

# Instellingen
MAX_SPEED = 5.0
CRUISING_SPEED = 4.0   # Iets langzamer rijden is veiliger
OBSTACLE_DIST = 0.6    # Reageer iets eerder (op 60 cm)
SCAN_ANGLE = 30        # Hoeveel meetpunten links en rechts we checken

while robot.step(timestep) != -1:
    # 4. LEES LIDAR DATA
    range_image = lidar.getRangeImage()
    
    if not range_image:
        continue

    # --- VERBETERING: KIJKHOEK (VISION CONE) ---
    # In plaats van alleen index [0], pakken we een 'slice' van de data.
    # Webots Lidar: index 0 is vooruit. De lijst loopt rond.
    # Het einde van de lijst ([-x:]) is links-voor, het begin ([:x]) is rechts-voor.
    
    # We plakken de laatste 30 punten en de eerste 30 punten aan elkaar
    # Dit geeft een blikveld van ongeveer 60 punten breed recht vooruit.
    right_side = range_image[:SCAN_ANGLE]      # 0 tot 30
    left_side = range_image[-SCAN_ANGLE:]      # 690 tot 720 (bijv)
    front_sector = right_side + left_side

    # Zoek de MINIMALE afstand in die hele sector
    # Als alles 'inf' is, is de min() ook 'inf', dat is prima.
    min_front_distance = min(front_sector)

    # Debug info
    # print(f"Dichtstbijzijnde object voor: {min_front_distance:.2f} m")

    # 5. OBSTAKEL LOGICA
    left_speed = CRUISING_SPEED
    right_speed = CRUISING_SPEED
    
    # Check of de afstand in onze hele kegel te klein is
    if min_front_distance < OBSTACLE_DIST:
        print(f"Obstakel gedetecteerd op {min_front_distance:.2f}m! Draaien...")
        
        # Draai agressief om botsing te voorkomen
        # Een wiel achteruit en een vooruit zorgt voor draaien op de plaats
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
        
    else:
        # Gewoon rechtdoor
        left_speed = CRUISING_SPEED
        right_speed = CRUISING_SPEED
        
    # Stuur motoren aan
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)