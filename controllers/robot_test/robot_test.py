from controller import Robot
import math
import numpy as np
import cv2

# ==========================================
# 1. INITIALISATIE
# ==========================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motoren
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Encoders
left_ps = robot.getDevice('left_wheel_sensor')
right_ps = robot.getDevice('right_wheel_sensor')
left_ps.enable(timestep)
right_ps.enable(timestep)

# Lidar
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

# ==========================================
# 2. CONFIGURATIE (BEREKEND UIT JE SCREENSHOT)
# ==========================================
# Anchor was -0.07 -> Dus totale breedte is 0.07 * 2 = 0.14
# Anchor hoogte was 0.0325 -> Dus straal is 0.0325
WHEEL_RADIUS = 0.0325  
AXLE_LENGTH = 0.14     

# Kaart instellingen
MAP_SIZE = 600         
MAP_SCALE = 30.0       # 30 pixels per meter
# Zwarte achtergrond
slam_map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)

# Navigatie
MAX_SPEED = 5.0
CRUISING_SPEED = 4.0
OBSTACLE_DIST = 0.6
SCAN_ANGLE = 30

# Startpositie
robot_x = 0.0
robot_z = 0.0
robot_theta = 0.0

print("SLAM Gestart met exacte kalibratie (0.14m as-breedte)")

# ==========================================
# 3. FUNCTIES
# ==========================================

def world_to_grid(x, z):
    """Zet meters om naar pixels, met (0,0) in het midden."""
    center_u = MAP_SIZE // 2
    center_v = MAP_SIZE // 2
    
    u = int(x * MAP_SCALE + center_u)
    v = int(z * MAP_SCALE + center_v)
    return u, MAP_SIZE - v # Flip Y-as

def update_visuals(lidar_data):
    """Tekent de kaart met Raytracing (gummen)."""
    display_img = slam_map.copy()

    # A. Waar is de robot nu?
    ru, rv = world_to_grid(robot_x, robot_z)
    
    # Teken robot op het scherm (Blauw)
    cv2.circle(display_img, (ru, rv), 4, (255, 0, 0), -1) 

    # B. LIDAR DATA VERWERKEN (3D naar 2D fix)
    width = lidar.getHorizontalResolution()
    total_points = len(lidar_data)
    
    if total_points > width:
        # Pak de middelste laag
        num_layers = total_points // width
        layer_index = num_layers // 2 
        start_idx = int(layer_index * width)
        end_idx = int(start_idx + width)
        current_layer_data = lidar_data[start_idx : end_idx]
    else:
        current_layer_data = lidar_data

    # C. Kaart Tekenen met RAYTRACING
    fov = lidar.getFov()
    angle_step = fov / (width - 1)
    start_angle = -fov / 2
    
    for i, dist in enumerate(current_layer_data):
        # 1. Bepaal hoever we moeten tekenen
        if math.isinf(dist) or dist > 5.0:
            draw_dist = 5.0 
            is_wall = False
        else:
            draw_dist = dist
            is_wall = True
            
        # 2. Bereken coördinaten
        alpha = start_angle + i * angle_step
        world_angle = robot_theta + alpha
        
        ox = robot_x + math.cos(world_angle) * draw_dist
        oz = robot_z + math.sin(world_angle) * draw_dist
        
        # Pixel coördinaten
        u, v = world_to_grid(ox, oz)
        
        if 0 <= u < MAP_SIZE and 0 <= v < MAP_SIZE:
            # STAP 1: DE GUM (Raytracing)
            # Maak de ruimte leeg (zwart) tussen robot en muur
            cv2.line(slam_map, (ru, rv), (u, v), (0, 0, 0), 1)
            
            # STAP 2: DE PEN (Muur tekenen)
            if is_wall:
                cv2.circle(slam_map, (u, v), 2, (255, 255, 255), -1)

    # Kopieer de permanente map naar het display plaatje
    np.copyto(display_img, slam_map)
    
    # Teken robot nogmaals vers op het plaatje
    cv2.circle(display_img, (ru, rv), 4, (255, 0, 0), -1)

    cv2.imshow("SLAM Map", display_img)
    cv2.waitKey(1)

# ==========================================
# 4. EERSTE LEZING (NULSTELLING)
# ==========================================
robot.step(timestep)
old_ps_left = left_ps.getValue()
old_ps_right = right_ps.getValue()
print("Klaar voor start!")

# ==========================================
# 5. HOOFD LOOP
# ==========================================
step_counter = 0

while robot.step(timestep) != -1:
    step_counter += 1
    
    # --- A. ODOMETRIE ---
    ps_left_val = left_ps.getValue()
    ps_right_val = right_ps.getValue()
    
    diff_left = ps_left_val - old_ps_left
    diff_right = ps_right_val - old_ps_right
    
    old_ps_left = ps_left_val
    old_ps_right = ps_right_val
    
    dist_left = diff_left * WHEEL_RADIUS
    dist_right = diff_right * WHEEL_RADIUS
    
    linear_dist = (dist_left + dist_right) / 2.0
    d_theta = (dist_right - dist_left) / AXLE_LENGTH
    
    robot_theta += d_theta
    robot_theta = math.atan2(math.sin(robot_theta), math.cos(robot_theta))
    
    robot_x += linear_dist * math.cos(robot_theta)
    robot_z += linear_dist * math.sin(robot_theta)
    
    # --- B. KAART ---
    range_image = lidar.getRangeImage()
    if range_image and step_counter % 3 == 0:
        update_visuals(range_image)
        
    # --- C. NAVIGATIE ---
    width = lidar.getHorizontalResolution()
    
    right_sector = range_image[:SCAN_ANGLE]      
    left_sector = range_image[-SCAN_ANGLE:]
    front_sector = right_sector + left_sector
    
    valid_dists = [d for d in front_sector if not math.isinf(d)]
    if valid_dists:
        min_front_distance = min(valid_dists)
    else:
        min_front_distance = float('inf')

    left_speed = CRUISING_SPEED
    right_speed = CRUISING_SPEED
    
    if min_front_distance < OBSTACLE_DIST:
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

cv2.destroyAllWindows()