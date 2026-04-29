from controller import Supervisor
from controller import Keyboard
from controller import Display

import numpy,struct
import ga,os
import sys
import csv
import time
import datetime


class SupervisorGA:
    def __init__(self):
        # Simulation Parameters
        self.time_step = 32 # ms
        self.time_experiment = 90 # s
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef("Controller")
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        self.trans_field = self.robot_node.getField("translation")  
        self.rot_field = self.robot_node.getField("rotation")
        
        # Check Receiver and Emitter
        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(self.time_step)
        
        # Initialize the receiver and emitter data to null
        self.receivedData = "" 
        self.receivedWeights = "" 
        self.receivedFitness = "" 
        self.emitterData = ""
        
        ### Define here the GA Parameters
        self.num_generations = 15  
        self.num_population = 18   
        self.num_elite = 2         
        
        # size of the genotype variable
        self.num_weights = 0
        
        # Creating the initial population
        self.population = []
        
        # All Genotypes
        self.genotypes = []
        
        # Display: screen to plot the fitness values
        self.display = self.supervisor.getDevice("display")
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.prev_best_fitness = 0.0;
        self.prev_average_fitness = 0.0;
        self.display.drawText("Fitness (Best - Red)", 0,0)
        self.display.drawText("Fitness (Average - Green)", 0,10)
        
        # CSV Logging Setup
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        log_name = f"GA_log_{timestamp}.csv"
        try:
            self.csv_file = open(log_name, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Generation", "Best Fitness", "Average Fitness", "Time (sec)"])
        except IOError:
            sys.stderr.write(f"Error: Could not open {log_name} for writing.\n")
            self.csv_file = None


    def createRandomPopulation(self):
        if(self.num_weights > 0):
            pop_size = (self.num_population,self.num_weights)
            self.population = numpy.random.uniform(low=-1.0, high=1.0, size=pop_size)

    def handle_receiver(self):
        while(self.receiver.getQueueLength() > 0):
            self.receivedData = self.receiver.getString()
            typeMessage = self.receivedData[0:7]
            if(typeMessage == "weights"):
                self.receivedWeights = self.receivedData[9:len(self.receivedData)] 
                self.num_weights = int(self.receivedWeights)
            elif(typeMessage == "fitness"):  
                self.receivedFitness = float(self.receivedData[9:len(self.receivedData)])
            self.receiver.nextPacket()
        
    def handle_emitter(self):
        if(self.num_weights > 0):
            string_message = str(self.emitterData)
            string_message = string_message.encode("utf-8")
            self.emitter.send(string_message)     
        
    def run_seconds(self,seconds):
        stop = int((seconds*1000)/self.time_step)
        iterations = 0
        while self.supervisor.step(self.time_step) != -1:
            self.handle_emitter()
            self.handle_receiver()
            if(stop == iterations):
                break    
            iterations = iterations + 1
                
    def evaluate_genotype(self,genotype,generation):
        self.emitterData = str(genotype)
        
        # Reset robot position and physics
        INITIAL_TRANS = [0.47, 0.16, 0]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [0, 0, 1, 1.57]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment)
    
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness: {}".format(fitness))
        current = (generation,genotype,fitness)
        self.genotypes.append(current)  
        
        return fitness

    def run_demo(self):
        if not os.path.exists("Best.npy"):
            print("Error: 'Best.npy' not found. Run optimization first (S).")
            return

        genotype = numpy.load("Best.npy")
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [0.47, 0.16, 0]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [0, 0, 1, 1.57]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment)    
    
    def run_optimization(self):
        while(self.num_weights == 0):
            self.handle_receiver()
            self.createRandomPopulation()
        
        print("starting GA optimization ...\n")
        
        for generation in range(self.num_generations):
            print("Generation: {}".format(generation))
            current_population = []
            
            # START TIMER
            start_time = time.time()

            for population in range(self.num_population):
                genotype = self.population[population]
                fitness = self.evaluate_genotype(genotype, generation)
                current_population.append((genotype, float(fitness)))
                    
            # END TIMER
            elapsed = time.time() - start_time
                
            # After checking the fitness value of all individuals
            best = ga.getBestGenotype(current_population);
            average = ga.getAverageGenotype(current_population);
            numpy.save("Best.npy", best[0]);
            self.plot_fitness(generation, best[1], average);
            
            # TERMINAL LOGGING
            print(f"Generation {generation} time: {elapsed:.2f} seconds")

            # CSV LOG ENTRY
            if self.csv_file:
                self.csv_writer.writerow([generation, best[1], average, round(elapsed, 3)])
                self.csv_file.flush()

            # Generate the new population using genetic operators
            if (generation < self.num_generations - 1):
                self.population = ga.population_reproduce(current_population,self.num_elite);
        
        print("GA optimization terminated.\n")
        if self.csv_file:
            self.csv_file.close()
    
    
    def draw_scaled_line(self, generation, y1, y2): 
        XSCALE = int(self.width/self.num_generations);
        YSCALE = 100;
        self.display.drawLine((generation-1)*XSCALE, self.height-int(y1*YSCALE), generation*XSCALE, self.height-int(y2*YSCALE));
    
    def plot_fitness(self, generation, best_fitness, average_fitness):
        if (generation > 0):
            self.display.setColor(0xff0000);  # red
            self.draw_scaled_line(generation, self.prev_best_fitness, best_fitness);
    
            self.display.setColor(0x00ff00);  # green
            self.draw_scaled_line(generation, self.prev_average_fitness, average_fitness);
    
        self.prev_best_fitness = best_fitness;
        self.prev_average_fitness = average_fitness;
    
if __name__ == "__main__":
    gaModel = SupervisorGA()
    keyboard = Keyboard()
    keyboard.enable(50)
    
    # Interface
    print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
    while gaModel.supervisor.step(gaModel.time_step) != -1:
        resp = keyboard.getKey()
        if(resp == 83): # S key (Search)
            gaModel.run_optimization()
            print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        elif(resp == 82): # R key (Run)
            gaModel.run_demo()
            print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")