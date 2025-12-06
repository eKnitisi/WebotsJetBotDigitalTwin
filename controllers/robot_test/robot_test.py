"""my_controller controller."""

from controller import Robot, Camera, Motor
import struct

def run_robot(robot):
    # ==========================================================
    # 1. INITIALISATIE
    # ==========================================================
    timestep = int(robot.getBasicTimeStep())
    
    # Motors ophalen (namen komen uit jouw .proto)
    left_motor = robot.getDevice('left_wheel_motor')
    right_motor = robot.getDevice('right_wheel_motor')
    
    # Motors instellen voor snelheid (velocity control)
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Camera ophalen en aanzetten
    camera = robot.getDevice('camera')
    camera.enable(timestep)
    
    # Camera specificaties (uit je .proto: 1280x720)
    # LET OP: 1280x720 is zwaar om elke pixel te scannen in Python.
    # We scannen daarom met stappen (overslaan van pixels).
    cam_width = camera.getWidth()
    cam_height = camera.getHeight()
    
    # Instellingen voor beweging
    MAX_SPEED = 8.0
    BASE_SPEED = 4.0
    
    print("JetBot Red Ball Tracker gestart...")

    # ==========================================================
    # 2. CONTROL LOOP
    # ==========================================================
    while robot.step(timestep) != -1:
        
        # Haal het beeld op (Raw bytes: B, G, R, A volgorde meestal in Webots Python)
        image = camera.getImage()
        
        red_sum_x = 0
        red_count = 0
        
        # We scannen niet het hele beeld, maar een strook in het midden
        # om CPU te besparen (van 40% tot 60% van de hoogte)
        start_y = int(cam_height * 0.4)
        end_y = int(cam_height * 0.6)
        
        # We gebruiken een 'step' van 5 pixels om het sneller te maken
        for y in range(start_y, end_y, 5):
            for x in range(0, cam_width, 5):
                # Webots image bytes processing
                # Formaat is BGRA (Blue, Green, Red, Alpha) per pixel
                # index berekenen in de 1D byte array
                # pixel_index = (y * cam_width + x) * 4 
                
                # Snel ophalen via camera image layer methode is vaak makkelijker,
                # maar hier doen we direct byte parsing voor pure Python.
                # Webots Python API < R2022b geeft bytes, nieuwere soms lijst.
                # We gebruiken hier de veiligste methode voor bytes:
                
                base_index = (y * cam_width + x) * 4
                
                # Waarden ophalen (Let op: Webots geeft vaak BGRA)
                blue  = image[base_index]
                green = image[base_index + 1]
                red   = image[base_index + 2]
                
                # EENVOUDIGE KLEURDETECTIE (Thresholding)
                # Is het rood dominant?
                if red > 130 and red > (green + 30) and red > (blue + 30):
                    red_sum_x += x
                    red_count += 1
        
        # ==========================================================
        # 3. LOGICA & STURING (P-Controller)
        # ==========================================================
        
        left_speed = 0.0
        right_speed = 0.0
        
        if red_count > 0:
            # Bal gevonden! Bereken het middelpunt
            center_x = red_sum_x / red_count
            
            # Bereken afwijking van het midden van het scherm
            # (0 is links, width is rechts. width/2 is midden)
            image_center = cam_width / 2
            error = center_x - image_center
            
            # Stuurcorrectie (Proportional control)
            # Als error positief is (bal rechts), moeten we naar rechts sturen
            # Kp is de gevoeligheid
            Kp = 0.015 
            steering = error * Kp
            
            # Pas snelheden aan
            left_speed = BASE_SPEED + steering
            right_speed = BASE_SPEED - steering
            
            # Begrens de snelheden
            left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
            right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
            
            # (Optioneel) Stop als de bal heel dichtbij is
            # Als heel veel pixels rood zijn, staan we ervoor
            if red_count > (cam_width * (end_y - start_y) / 25) * 0.3:
                 left_speed = 0
                 right_speed = 0
                 
        else:
            # Geen bal gezien? Draai om je as om te zoeken
            left_speed = -2.0
            right_speed = 2.0
            
        # Stuur commando's naar motoren
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)