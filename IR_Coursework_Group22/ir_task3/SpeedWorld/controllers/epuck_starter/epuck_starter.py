from controller import Robot, Receiver, Emitter
import sys,struct,math
import numpy as np
import mlp as ntw

class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # MLP Architecture (Optimized for smoother control)
        self.number_input_layer = 11 # 8 proximity + 3 ground sensors
        self.number_hidden_layer = [12, 6] # Two hidden layers
        self.number_output_layer = 2 # Left and Right Wheel Velocities
        
        # Create a list with the number of neurons per layer
        self.number_neuros_per_layer = []
        self.number_neuros_per_layer.append(self.number_input_layer)
        self.number_neuros_per_layer.extend(self.number_hidden_layer)
        self.number_neuros_per_layer.append(self.number_output_layer)
        
        # Initialize the network
        self.network = ntw.MLP(self.number_neuros_per_layer)
        self.inputs = []
        
        # Calculate the number of weights
        self.number_weights = 0
        for n in range(1,len(self.number_neuros_per_layer)):
            if(n == 1):
                self.number_weights += (self.number_neuros_per_layer[n-1]+1)*self.number_neuros_per_layer[n]
            else:
                self.number_weights += self.number_neuros_per_layer[n-1]*self.number_neuros_per_layer[n]

        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors (gs0 is Right, gs1 is Center, gs2 is Left)
        self.left_ir = self.robot.getDevice('gs0')   # Physically Right sensor
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1') # Physically Center sensor
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')  # Physically Left sensor
        self.right_ir.enable(self.time_step)
        
        # Enable Emitter and Receiver
        self.emitter = self.robot.getDevice("emitter") 
        self.receiver = self.robot.getDevice("receiver") 
        self.receiver.enable(self.time_step)
        self.receivedData = "" 
        self.receivedDataPrevious = "" 
        self.flagMessage = False
        
        # Fitness value (initialization fitness parameters once)
        self.fitness_values = []
        self.fitness = 0

    def check_for_new_genes(self):
        if(self.flagMessage == True):
                # Split the list based on the number of layers
                part = []
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        part.append((self.number_neuros_per_layer[n-1]+1)*(self.number_neuros_per_layer[n]))
                    else:   
                        part.append(self.number_neuros_per_layer[n-1]*self.number_neuros_per_layer[n])
                
                # Set the weights of the network
                data = []
                weightsPart = []
                sum = 0
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        weightsPart.append(self.receivedData[n-1:part[n-1]])
                    elif(n == (len(self.number_neuros_per_layer)-1)):
                        weightsPart.append(self.receivedData[sum:])
                    else:
                        weightsPart.append(self.receivedData[sum:sum+part[n-1]])
                    sum += part[n-1]
                for n in range(1,len(self.number_neuros_per_layer)):  
                    if(n == 1):
                        weightsPart[n-1] = weightsPart[n-1].reshape([self.number_neuros_per_layer[n-1]+1,self.number_neuros_per_layer[n]])    
                    else:
                        weightsPart[n-1] = weightsPart[n-1].reshape([self.number_neuros_per_layer[n-1],self.number_neuros_per_layer[n]])    
                    data.append(weightsPart[n-1])                
                self.network.weights = data
                
                #Reset fitness list
                self.fitness_values = []
        
    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self):
        # MLP: 
        output = self.network.propagate_forward(self.inputs)
        self.velocity_left = output[0]
        self.velocity_right = output[1]
        
        # Multiply the motor values by 3 to increase the velocities
        self.left_motor.setVelocity(self.velocity_left*3)
        self.right_motor.setVelocity(self.velocity_right*3)

    def calculate_fitness(self):

        # Wheel speeds in [-1,1] 
        vl = float(self.velocity_left)
        vr = float(self.velocity_right)

        # === 1) Forward motion fitness ===
        avg_v = (vl + vr) / 2.0
        forwardFitness = max(0.0, avg_v) 

        # === 2) Line following fitness ===
        gs_l = self.inputs[0]
        gs_c = self.inputs[1]
        gs_r = self.inputs[2]
        side_mean = (gs_l + gs_r) / 2.0
        followLineFitness = abs(gs_c - side_mean) 

        # === 3) Obstacle avoidance fitness ===
        ds = self.inputs[3:]  
        front_indices = [0, 1, 6, 7]
        front_vals = [ds[i] for i in front_indices]
        max_front = max(front_vals)
        avoidCollisionFitness = 1.0 - max_front

        # === 4) Spinning penalty ===
        spin_diff = abs(vl - vr)
        spinningFitness = max(0.0, 1.0 - (spin_diff / 2.0))

        # === Weighted total ===
        W_F = 1.0
        W_L = 3.0
        W_A = 3.0
        W_S = 1.0

        combinedFitness = (
            W_F * forwardFitness +
            W_L * followLineFitness +
            W_A * avoidCollisionFitness +
            W_S * spinningFitness
        )

        self.fitness_values.append(combinedFitness)
        self.fitness = np.mean(self.fitness_values) 

    def handle_emitter(self):
        # Send the self.fitness value to the supervisor
        data = str(self.number_weights)
        data = "weights: " + data
        string_message = str(data)
        string_message = string_message.encode("utf-8")
        self.emitter.send(string_message)

        # Send the self.fitness value to the supervisor
        data = str(self.fitness)
        data = "fitness: " + data
        string_message = str(data)
        string_message = string_message.encode("utf-8")
        self.emitter.send(string_message)
            
    def handle_receiver(self):
        if self.receiver.getQueueLength() > 0:
            while(self.receiver.getQueueLength() > 0):
                self.receivedData = self.receiver.getString()
                
                self.receivedData = self.receivedData[1:-1]
                self.receivedData = self.receivedData.split()
                x = np.array(self.receivedData)
                self.receivedData = x.astype(float)
                self.receiver.nextPacket()
                
            # Is it a new Genotype?
            if(np.array_equal(self.receivedDataPrevious,self.receivedData) == False):
                self.flagMessage = True
            else:
                self.flagMessage = False
                
            self.receivedDataPrevious = self.receivedData 
        else:
            self.flagMessage = False

    def run_robot(self):        
        # Main Loop
        while self.robot.step(self.time_step) != -1:
            self.inputs = []
            
            # Emitter and Receiver
            self.handle_emitter()
            self.handle_receiver()
            
            # --- START CRITICAL FIX: CORRECT SENSOR MAPPING ---
            # GS0 is Right, GS1 is Center, GS2 is Left
            right_raw = self.left_ir.getValue()     # gs0
            center_raw = self.center_ir.getValue()  # gs1
            left_raw = self.right_ir.getValue()     # gs2
            
            # Rename variables for easy use in subsequent blocks
            left = left_raw
            center = center_raw
            right = right_raw
            # --- END CRITICAL FIX ---
                        
            ### Sensor Normalization (Ground)
            min_gs = 50 
            max_gs = 4096 
            
            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs
            
            # Normalize the values between 0 and 1. Order is LEFT, CENTER, RIGHT.
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))
            
            # Read Distance Sensors
            for i in range(8):
                if(i==0 or i==1 or i==2 or i==3 or i==4 or i==5 or i==6 or i==7):        
                    temp = self.proximity_sensors[i].getValue()
                    
                    ### Sensor Normalization (Distance)
                    min_ds = 0
                    max_ds = 4096 
                    
                    if(temp > max_ds): temp = max_ds
                    if(temp < min_ds): temp = min_ds
                    
                    # Normalize the values between 0 and 1
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
    
            # GA Iteration       
            self.check_for_new_genes()
            self.sense_compute_and_actuate()
            self.calculate_fitness()
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()