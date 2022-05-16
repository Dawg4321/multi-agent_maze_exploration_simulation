import json
import argparse
from tokenize import group
import matplotlib.pyplot as plt
import numpy as np
import os
import analysis_functions
from scipy.stats import entropy

parser = argparse.ArgumentParser() # parsing argument for target simulation directory
parser.add_argument("dir")
args = parser.parse_args()

parent_directory = args.dir

evaluation_settings_name = "Sim_Settings.json"
simulation_json_name = "Simulation.json"

file = open(parent_directory + evaluation_settings_name) # gathering json data about simulation

evaluation_settings = json.load(file) # placing simulation settings into a dictionary

num_of_robots = evaluation_settings["Number_of_Robots"] # gathering total number of robots used 
group_sizes = evaluation_settings["Group_Sizes"] # gathering size of robot groups used in each instance
number_of_mazes = evaluation_settings["Number_of_Mazes"]

turns_taken_matrix = [] # matrix to store number of turns taken for each robot test
eveness_index_matrix = [] # matrix to store eveness for each robot test
num_of_used_robots_matrix = [] # matrix to store number of robots which actually performed a scan operation
prob_cells_scanned_matrix = [] # matrix to store probabilities of various robots performing a scan operation 

for n in range(len(group_sizes)): # iterate through the various robot swarm sizes

    i = group_sizes[n]
    simulation_directory = parent_directory + str(num_of_robots) + "_group_size_" + str(i) # gathering parent directory of robot to evaluate
    
    turns_taken = [] # list to store various values for the number of turns taken by a robot
    eveness_index_list = [] # list to store various values for the eveness index of a simulation
    num_of_used_robots_list = [] # list to store percentage of robots used in scanning
    probability_of_scanning_list = [] # list to store probability of cell being scanned

    print("Calculating " + str(n + 1) + "/" + str(len(group_sizes)))
    
    for root, subdirectories, files in os.walk(simulation_directory): # get subdirectories of simulation_directory
                                                                      # subdirectories contain simulation info for specified robot size
        for subdirectory in subdirectories: # iterate through simulation directories
                directory_2_json = root + "/" + subdirectory  + "/" + simulation_json_name

                # getting turns taken from this simulation
                num_of_turns = analysis_functions.getNumberofTurns(directory_2_json) # get the number of turns taken by the simulation

                turns_taken.append(num_of_turns) # add the number of turns taken to the list
                
                # getting balance of robot's exploration of this simulation
                # based off this: https://stats.stackexchange.com/questions/239973/a-general-measure-of-data-set-imbalance
                # read this: https://www.itl.nist.gov/div898/software/dataplot/refman2/auxillar/shannon.htm
                # and this: https://en.wikipedia.org/wiki/Species_evenness
                num_of_cells_scanned, robot_ids = analysis_functions.getNumberofCellsScanned(directory_2_json) # getting array with # of cells scanned by each robot
                
                total_num_of_cells = sum(num_of_cells_scanned) # getting total number of cells scanned
                
                probability_of_scanning = np.array(num_of_cells_scanned)/total_num_of_cells # getting probability of a robot scanning a cell
                probability_of_scanning_list.append(-np.sort(-probability_of_scanning)) # adding probability of various robots scanning a cell to a list
                
                if 1 in probability_of_scanning: # if only one robot did scanning, even index should be one as only one robot did the work
                    balance_of_scanning = 1 # entropy of 1 is not possible due to ln(1) = 0
                    
                else: # if more than one robot did the work
                    balance_of_scanning = entropy(probability_of_scanning, base=2)/np.log2(len(np.nonzero(probability_of_scanning)[0])) # calculating Pielou's eveness index
                
                eveness_index_list.append(balance_of_scanning)

                # getting percentage of used robots for exploration
                num_of_used_robots = 0
                for scanned_cells in num_of_cells_scanned:
                    if scanned_cells != 0:
                        num_of_used_robots += 1
                
                num_of_used_robots_list.append(num_of_used_robots)
                

    turns_taken_matrix.append((i, turns_taken)) # append turns_taken matrix with tuple containing number of robots used for simulations and the turns taken list
    eveness_index_matrix.append((i, eveness_index_list)) # append eveness_index matrix with tuple containing number of robots used for simulations and the eveness index list
    num_of_used_robots_matrix.append((i, num_of_used_robots_list)) # append percentage_of_used_robots with tuple
    prob_cells_scanned_matrix.append(probability_of_scanning_list) # append probability of a cell being scanned to the matrix

group_sizes.reverse() # reversing group sizes as number of groups is reverse of this
num_groups = group_sizes # getting number of groups
print(group_sizes)
average_turns_taken = [] # list containing average number of turns taken to map maze at each size
std_deviation = [] # standard deviation across turns taken for a robot number

for turn_values in turns_taken_matrix:
    average_turns_taken.append(np.average(turn_values[1]))
    std_deviation.append(np.std(turn_values[1]))

subtitle = str(num_of_robots) + " robots - " + str(evaluation_settings["Maze_Size"]) + "x" + str(evaluation_settings["Maze_Size"]) + " - " + analysis_functions.getRobotTypeName(evaluation_settings["Robot_Type"]) # subtitle used in following plots

plt.figure(1) # getting average turns taken at each robot size
plt.plot(num_groups, average_turns_taken, '--bo')
plt.plot(num_groups, std_deviation, '--ro')
plt.title("Relationship between average number of steps and number of robots used\n" + subtitle, fontsize=11)
plt.ylabel("Average number of steps over " + str(number_of_mazes) + " mazes")
plt.xlabel("Number of robot groups")
plt.legend(["Average turns taken", "Standard deviation"])
plt.grid(linestyle = '--')
plt.savefig(parent_directory + "Figure_1.png")


average_used_robots = []
std_used_robots = []

for num_of_used_robots in num_of_used_robots_matrix: # getting average robot's used at each robot size
    average_used_robots.append(np.average(num_of_used_robots[1]))
    std_used_robots.append(np.std(num_of_used_robots[1]))

plt.figure(2) # plotting average robot's used at each robot size
plt.plot(num_groups, average_used_robots, '--bo')
plt.plot(num_groups, std_used_robots, '--ro')
plt.title("Relationship between number of agents which scanned a cell\n and number of robots used\n" + subtitle, fontsize=11)
plt.ylabel("Average number of robots who scanned a cell over " + str(number_of_mazes) + " mazes")
plt.xlabel("Number of robot groups")
plt.legend(["Number of robots used", "Standard deviation"])
plt.grid(linestyle = '--')
plt.savefig(parent_directory + "Figure_2.png")

average_eveness_index = []
eveness_std_deviation = []

for eveness_indexes in eveness_index_matrix: # getting average evenness at each robot size
    average_eveness_index.append(np.average(eveness_indexes[1]))
    eveness_std_deviation.append(np.std(eveness_indexes[1]))

plt.figure(3) # plotting average evenness at each robot size
plt.plot(num_groups, average_eveness_index, '--bo')
plt.plot(num_groups, eveness_std_deviation, '--ro')
plt.title("Relationship between eveness of cell scan operations\n and number of robots used\n" + subtitle, fontsize=11)
plt.ylabel("Average Pielou's Eveness Index value for robots who\n scanned a cell over " + str(number_of_mazes) + " mazes")
plt.xlabel("Number of robot groups")
plt.legend(["Average Pielou's Eveness Index", "Standard deviation"])
plt.grid(linestyle = '--')
plt.savefig(parent_directory + "Figure_3.png")

average_probability_of_scan = []

for prob_of_scan in prob_cells_scanned_matrix: # getting probability of each robot scanning at each robot size
    average_probability_of_scan.append(np.average(prob_of_scan,axis=0))

plt.figure(4)
for i in range(len(average_probability_of_scan)): # plotting probability of each robot scanning at each group size
    row = average_probability_of_scan[i]
    plt.scatter(num_groups[i]*np.ones(num_of_robots), row)

plt.title("Relationship between proportion of cells scanned by a robot\n and number of robots used\n" + subtitle, fontsize=11)
plt.ylabel("Average propotion of scan operations performed by a robot\n over  " + str(number_of_mazes) + " mazes")
plt.xlabel("Number of robot groups")
plt.grid(linestyle = '--')
plt.savefig(parent_directory + "Figure_4.png")

plt.show()