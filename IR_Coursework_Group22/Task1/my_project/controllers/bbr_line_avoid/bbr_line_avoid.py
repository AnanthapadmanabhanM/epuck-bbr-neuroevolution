from controller import Robot

class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = 32  # ms
        self.max_speed = 1.0
 
        # Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0.0
        self.velocity_right = 0.0
    
        # Proximity Sensors ps0..ps7
        self.proximity_sensors = []
        for i in range(8):
            sensor = self.robot.getDevice('ps' + str(i))
            sensor.enable(self.time_step)
            self.proximity_sensors.append(sensor)
       
        # Ground Sensors gs0..gs2
        self.left_ir = self.robot.getDevice('gs0')
        self.center_ir = self.robot.getDevice('gs1')
        self.right_ir = self.robot.getDevice('gs2')
        self.left_ir.enable(self.time_step)
        self.center_ir.enable(self.time_step)
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []          # current normalised sensor values
        self.inputsPrevious = []  # previous cycle values
        
        # Line-follow search flag
        self.flag_turn = 0  

        # Obstacle-avoidance FSM
        self.avoid_state = "NONE"   # "NONE", "TURN_OUT", "GO_AROUND"
        self.avoid_dir = 0          # +1 = around LEFT, -1 = around RIGHT
        self.avoid_counter = 0

        # Noise-robust front obstacle detection
        self.front_obstacle_counter = 0
        self.FRONT_CONFIRM_STEPS = 3

        # After rejoining line, keep going forward (no instant spin)
        self.rejoin_lock_steps = 0

        # simple heading estimation from wheel speeds
        self.heading = 0.0
        self.heading_entry = 0.0
        self.HEADING_GAIN = 0.02   # tune if needed

        # NEW: remember previous side distance for gap control
        self.side_inner_prev = 0.0

    def clip_value(self, value, min_max):
        if value > min_max:
            return min_max
        elif value < -min_max:
            return -min_max
        return value

    # helper to keep heading roughly bounded and update it
    def update_heading(self):
        # integrate difference in wheel speeds
        self.heading += self.HEADING_GAIN * (self.velocity_right - self.velocity_left)
        # wrap roughly into [-2*pi, 2*pi]
        if self.heading > 6.283:
            self.heading -= 6.283
        elif self.heading < -6.283:
            self.heading += 6.283

    # ---------- OBSTACLE AVOIDANCE FSM USING PROXIMITY + GROUND ----------

    def obstacle_avoidance_step(self):
        if self.avoid_state == "NONE":
            return False

        # ==== CONSTANTS TO TUNE ====
        SIDE_SEE_TH    = 0.05   # side sensor above this = "seeing obstacle"
        LINE_DARK      = 0.35   # any gs < LINE_DARK => on black line

        MAX_TURN_STEPS   = 35   # safety limit for TURN_OUT
        MAX_AROUND_STEPS = 500  # safety limit for GO_AROUND

        # max heading deviation allowed when rejoining line
        MAX_HEADING_DIFF = 2.0   # ~ about 100–120 degrees equivalent
        # ===========================

        # ground sensors
        left_gs   = self.inputs[0]
        center_gs = self.inputs[1]
        right_gs  = self.inputs[2]

        # proximity sensors
        ps0 = self.inputs[3]
        ps1 = self.inputs[4]
        ps2 = self.inputs[5]
        ps3 = self.inputs[6]
        ps4 = self.inputs[7]
        ps5 = self.inputs[8]
        ps6 = self.inputs[9]
        ps7 = self.inputs[10]

        side_right = max(ps1, ps2)
        side_left  = max(ps5, ps6)

        # choose which side should “see” the box
        if self.avoid_dir == +1:     # we go around on LEFT, box on RIGHT
            side_inner = side_right
        else:                        # we go around on RIGHT, box on LEFT
            side_inner = side_left

        # ---------- STATE: TURN_OUT ----------
        if self.avoid_state == "TURN_OUT":
            base = 0.6
            turn = 0.4

            if self.avoid_dir == +1:
                # box on right -> arc left
                self.velocity_left  = base - turn
                self.velocity_right = base + turn
            else:
                # box on left -> arc right
                self.velocity_left  = base + turn
                self.velocity_right = base - turn

            self.avoid_counter += 1

            if self.avoid_counter >= MAX_TURN_STEPS:
                self.avoid_state = "GO_AROUND"
                self.avoid_counter = 0
                # initialise side reference when entering GO_AROUND
                self.side_inner_prev = side_inner

        # ---------- STATE: GO_AROUND (SIDE-FOLLOW) ----------
        elif self.avoid_state == "GO_AROUND":
            base = 0.6
            turn = 0.35

            # 1) If any ground sensor sees the black line again,
            #    only accept it if heading is still similar to entry.
            if min(left_gs, center_gs, right_gs) < LINE_DARK:
                heading_diff = abs(self.heading - self.heading_entry)

                if heading_diff < MAX_HEADING_DIFF:
                    # orientation OK -> rejoin line
                    if self.avoid_dir == +1:
                        # box on right -> turn slightly LEFT towards line
                        self.velocity_left  = base - turn
                        self.velocity_right = base + turn
                    else:
                        # box on left -> turn slightly RIGHT towards line
                        self.velocity_left  = base + turn
                        self.velocity_right = base - turn

                    # exit avoidance
                    self.avoid_state = "NONE"
                    self.avoid_dir = 0
                    self.flag_turn = 0
                    self.avoid_counter = 0

                    # keep some steps of “forward-only” line following
                    self.rejoin_lock_steps = 25
                else:
                    # heading too different -> probably the wrong direction,
                    # ignore this line detection and continue around the obstacle
                    # small forward bias
                    self.velocity_left  = base
                    self.velocity_right = base

            else:
                # 2) Wall-follow rule while we haven't seen an acceptable line yet
                if side_inner > SIDE_SEE_TH:
                    # maintain a small gap instead of just going straight
                    delta = side_inner - self.side_inner_prev
                    deadband = 0.01      # small tolerance
                    steer = 3.0         # steering strength

                    if delta > deadband:
                        # wall getting CLOSER (IR stronger) -> steer AWAY
                        if self.avoid_dir == +1:   # box on right
                            self.velocity_left  = base - steer
                            self.velocity_right = base + steer
                        else:                      # box on left
                            self.velocity_left  = base + steer
                            self.velocity_right = base - steer

                    elif delta < -deadband:
                        # wall getting FURTHER (IR weaker but still visible)
                        # -> move slightly TOWARDS that side to keep a gap
                        if self.avoid_dir == +1:   # box on right
                            self.velocity_left  = base + steer
                            self.velocity_right = base - steer
                        else:                      # box on left
                            self.velocity_left  = base - steer
                            self.velocity_right = base + steer

                    else:
                        # distance roughly constant -> go straight
                        self.velocity_left  = base
                        self.velocity_right = base

                else:
                    # wall/box lost -> turn TOWARDS that side until we see it again
                    if self.avoid_dir == +1:   # box on right
                        # turn right
                        self.velocity_left  = base + turn
                        self.velocity_right = base - turn
                    else:                      # box on left
                        # turn left
                        self.velocity_left  = base - turn
                        self.velocity_right = base + turn

                self.avoid_counter += 1
                if self.avoid_counter > MAX_AROUND_STEPS:
                    # safety: give up after long time and return to line mode
                    self.avoid_state = "NONE"
                    self.avoid_dir = 0
                    self.flag_turn = 0
                    self.avoid_counter = 0

                # update side reference each GO_AROUND step
                self.side_inner_prev = side_inner

        # update heading based on chosen velocities
        self.update_heading()

        # apply speeds for this step
        self.left_motor.setVelocity(self.clip_value(self.velocity_left,  1.0))
        self.right_motor.setVelocity(self.clip_value(self.velocity_right, 1.0))
        return True

    # ---------- MAIN BEHAVIOUR ARBITRATION ----------

    def sense_compute_and_actuate(self):
        # If we are currently avoiding an obstacle, that behaviour has priority
        if self.avoid_state != "NONE":
            if self.obstacle_avoidance_step():
                return

        # Normal mode: line following + detecting *new* obstacles
        if len(self.inputs) > 0 and len(self.inputsPrevious) > 0:

            # unpack proximity sensors
            ps0 = self.inputs[3]
            ps1 = self.inputs[4]
            ps2 = self.inputs[5]
            ps3 = self.inputs[6]
            ps4 = self.inputs[7]
            ps5 = self.inputs[8]
            ps6 = self.inputs[9]
            ps7 = self.inputs[10]

            # group front sensors
            front_right = max(ps0, ps1)
            front_left  = max(ps6, ps7)

            # small threshold = early detection, but we add confirmation to kill noise
            FRONT_TRIGGER = 0.05

            # noise-robust obstacle trigger using confirmation counter
            front_max = max(front_right, front_left)
            if front_max > FRONT_TRIGGER:
                self.front_obstacle_counter += 1
            elif front_max < FRONT_TRIGGER * 0.5:
                # clearly no obstacle -> reset
                self.front_obstacle_counter = 0

            # --- detect new obstacle in front (enter avoidance) ---
            if self.front_obstacle_counter >= self.FRONT_CONFIRM_STEPS:
                # store heading at the moment we start avoidance
                self.heading_entry = self.heading

                # choose side with more free space to go around
                if front_left >= front_right:
                    # obstacle more on left -> go around on RIGHT
                    self.avoid_dir = -1
                else:
                    # obstacle more on right -> go around on LEFT
                    self.avoid_dir = +1

                self.avoid_state = "TURN_OUT"
                self.avoid_counter = 0
                self.front_obstacle_counter = 0   # reset for next time
                self.obstacle_avoidance_step()
                return

            # ---------- LINE FOLLOWING ----------
            left_gs, center_gs, right_gs = self.inputs[0], self.inputs[1], self.inputs[2]

            # after rejoining from avoidance, lock direction for a few steps:
            if self.rejoin_lock_steps > 0:
                BASE_SPEED = 2.0
                SLOW_SPEED = 0.3

                if left_gs < center_gs and left_gs < right_gs:
                    # line under left sensor -> turn left gently
                    self.velocity_left  = SLOW_SPEED
                    self.velocity_right = BASE_SPEED
                elif center_gs < left_gs and center_gs < right_gs:
                    # centre on line -> straight
                    self.velocity_left  = BASE_SPEED
                    self.velocity_right = BASE_SPEED
                elif right_gs < left_gs and right_gs < center_gs:
                    # line under right sensor -> turn right gently
                    self.velocity_left  = BASE_SPEED
                    self.velocity_right = SLOW_SPEED
                else:
                    # tie-breaking: tiny bias, but never spin in place
                    self.velocity_left  = 0.6
                    self.velocity_right = 0.6

                # ensure we are not in search mode while stabilising
                self.flag_turn = 0
                self.rejoin_lock_steps -= 1

            else:
                # normal line-follow + search when line lost
                if self.flag_turn:
                    self.velocity_left = -0.3
                    self.velocity_right = 0.3
                    if min(self.inputs[0:3]) < 0.35:  # found line again
                        self.flag_turn = 0
                else:
                    # detect line lost or sudden contrast change
                    if sum(self.inputs[0:3]) / 3.0 > 0.85:
                        self.flag_turn = 1
                    elif (min(self.inputs[0:3]) - min(self.inputsPrevious[0:3])) > 0.2:
                        self.flag_turn = 1
                    else:
                        BASE_SPEED = 1.4
                        SLOW_SPEED = 0.3
                        
                        # choose direction by darkest ground sensor (on the black line)
                        if left_gs < center_gs and left_gs < right_gs:
                            # line under left sensor -> turn left
                            self.velocity_left  = SLOW_SPEED
                            self.velocity_right = BASE_SPEED
                        elif center_gs < left_gs and center_gs < right_gs:
                            # centre on line -> straight
                            self.velocity_left  = BASE_SPEED
                            self.velocity_right = BASE_SPEED
                        elif right_gs < left_gs and right_gs < center_gs:
                            # line under right sensor -> turn right
                            self.velocity_left  = BASE_SPEED
                            self.velocity_right = SLOW_SPEED
                        else:
                            # tie-breaking using previous centre value
                            if self.inputsPrevious[1] < self.inputs[1]:
                                self.velocity_left  = 0.1
                                self.velocity_right = 0.7
                            else:
                                self.velocity_left  = 0.7
                                self.velocity_right = 0.1
     
        # update heading for line-follow motion
        self.update_heading()

        # apply speeds
        self.left_motor.setVelocity(self.clip_value(self.velocity_left,  1.0))
        self.right_motor.setVelocity(self.clip_value(self.velocity_right, 1.0))

    # ---------- MAIN LOOP ----------

    def run_robot(self):        
        count = 0
        inputs_avg = []
        smooth = 3   # small smoothing -> faster reaction
        
        while self.robot.step(self.time_step) != -1:
            self.inputs = []
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()

            # Normalise ground sensors
            min_gs = 0
            max_gs = 1000
            left   = max(min(left,   max_gs), min_gs)
            center = max(min(center, max_gs), min_gs)
            right  = max(min(right,  max_gs), min_gs)
            
            self.inputs.append((left   - min_gs) / (max_gs - min_gs))
            self.inputs.append((center - min_gs) / (max_gs - min_gs))
            self.inputs.append((right  - min_gs) / (max_gs - min_gs))
            
            # Normalise distance sensors ps0..ps7
            for i in range(8):
                temp = self.proximity_sensors[i].getValue()
                min_ds = 0
                max_ds = 2400
                temp = max(min(temp, max_ds), min_ds)
                self.inputs.append((temp - min_ds) / (max_ds - min_ds))
      
            if count == smooth:
                # average over last 'smooth' samples
                inputs_avg = [sum(col) / smooth for col in zip(*inputs_avg)]
                self.inputs = inputs_avg
                
                self.sense_compute_and_actuate()
                
                count = 0
                inputs_avg = []
                self.inputsPrevious = list(self.inputs)
            else:
                inputs_avg.append(self.inputs)
                count += 1
                

if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
