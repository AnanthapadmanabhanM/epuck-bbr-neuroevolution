from controller import Robot

class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 6.28 # Maximum speed of the e-puck
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Enable Distance Sensors (ps0 to ps7) for Obstacles
        self.distance_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.time_step)
            self.distance_sensors.append(sensor)
    
        # Enable Light Sensors (ls0 to ls7) for Phototaxis
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.time_step)
            self.light_sensors.append(sensor)

    def run_robot(self):        
        while self.robot.step(self.time_step) != -1:
            # --- 1. READ SENSORS ---
            
            # Read Distance Sensors (Values > 80 mean an object is close)
            ps_values = []
            for i in range(8):
                ps_values.append(self.distance_sensors[i].getValue())

            # Read Light Sensors (Lower value = Brighter light)
            ls_values = []
            for i in range(8):
                ls_values.append(self.light_sensors[i].getValue())

            # --- 2. CONTROL LOGIC ---
            
            left_speed = 0
            right_speed = 0
            
            # Check for obstacles at the front or sides
            front_obstacle = (ps_values[0] > 80 or ps_values[7] > 80)
            left_obstacle = (ps_values[5] > 80 or ps_values[6] > 80)
            right_obstacle = (ps_values[1] > 80 or ps_values[2] > 80)

            # PRIORITY 1: Obstacle Avoidance (BBR Approach)
            if front_obstacle or left_obstacle or right_obstacle:
                print("Obstacle! Avoiding...")
                # If obstacle is on the left, turn right. Otherwise, turn left.
                if left_obstacle:
                    left_speed = 0.5 * self.max_speed
                    right_speed = -0.5 * self.max_speed
                else:
                    left_speed = -0.5 * self.max_speed
                    right_speed = 0.5 * self.max_speed
            
            # PRIORITY 2: Phototaxis (Move to Light)
            else:
                # Find the sensor with the lowest value (brightest light)
                min_ls_value = min(ls_values)
                min_ls_index = ls_values.index(min_ls_value)
                
                # If the value is high (> 3500), it's dark (no light detected)
                if min_ls_value > 3500:
                    # Spin around to search for light
                    left_speed = -0.5 * self.max_speed
                    right_speed = 0.5 * self.max_speed
                else:
                    # Light found! Move towards it.
                    # Index 0, 7 = Front
                    # Index 1, 2, 3 = Right
                    # Index 4, 5, 6 = Left
                    
                    if min_ls_index == 0 or min_ls_index == 7:
                        # Drive Forward
                        left_speed = self.max_speed
                        right_speed = self.max_speed
                    elif min_ls_index in [1, 2, 3]:
                        # Turn Right
                        left_speed = self.max_speed
                        right_speed = 0.2 * self.max_speed
                    elif min_ls_index in [4, 5, 6]:
                        # Turn Left
                        left_speed = 0.2 * self.max_speed
                        right_speed = self.max_speed

            # --- 3. ACTUATE MOTORS ---
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()