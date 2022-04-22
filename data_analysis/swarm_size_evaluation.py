import json
import matplotlib.pyplot as plt
import numpy as np
import os
import analysis_functions
from scipy.stats import entropy

parent_directory = "/home/ryan/code/git_workspace/junk/"

evaluation_settings_name = "Sim_Settings.json"
simulation_json_name = "Simulation.json"

file = open(parent_directory + evaluation_settings_name) # gathering json data about simulation

evaluation_settings = json.load(file) # placing simulation settings into a dictionary

range_of_robots = evaluation_settings["Robot_Sizes"] # gathering range of robot swarm size which were evaluated 

turns_taken_matrix = [] # matrix to store number of turns taken for each robot test
eveness_index_matrix = [] # matrix to store eveness for each robot test
num_of_used_robots_matrix = []

for i in range_of_robots: # iterate through the various robot swarm sizes

    simulation_directory = parent_directory + "sim_size_" + str(i) # gathering parent directory of robot to evaluate
    
    turns_taken = [] # list to store various values for the number of turns taken by a robot
    eveness_index_list = [] # list to store various values for the eveness index of a simulation
    num_of_used_robots_list = [] # list to store percentage of robots used in scanning

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
            
            probability_of_scanning = np.true_divide(num_of_cells_scanned, total_num_of_cells) # getting probability of a robot scanning a cell
            
            balance_of_scanning = entropy(probability_of_scanning, base=2)/np.log2(len(probability_of_scanning))
            eveness_index_list.append(balance_of_scanning)

            # getting percentage of used robots for exploration
            num_of_used_robots = 0
            for scanned_cells in num_of_cells_scanned:
                if scanned_cells != 0:
                    num_of_used_robots += 1
            
            num_of_used_robots_list.append(num_of_used_robots)
            

    turns_taken_matrix.append((i, turns_taken)) # append turns_taken matrix with tuple containing number of robots used for simulations and the turns taken list
    eveness_index_matrix.append((i, eveness_index_list)) # append eveness_index matrix with tuple containing number of robots used for simulations and the eveness index list
    num_of_used_robots_matrix.append((i, num_of_used_robots_list)) ## append percentage_of_used_robots with tuple

number_of_robots_list = [] # list to store swarm size used for each metric
average_turns_taken = [] # list containing average number of turns taken to map maze at each size
std_deviation = [] # standard deviation across turns taken for a robot number

for turn_values in turns_taken_matrix:
    number_of_robots_list.append(turn_values[0])
    average_turns_taken.append(np.average(turn_values[1]))
    std_deviation.append(np.std(turn_values[1]))

plt.figure()
plt.scatter(number_of_robots_list, average_turns_taken)
plt.scatter(number_of_robots_list, std_deviation)
plt.title("Maze mapping performance as number of agents increases")
plt.ylabel("Number of Turns")
plt.xlabel("Number of Agents")
plt.legend(["Average Turns Taken", "Standard Deviation"])
plt.show()

average_eveness_index = []
eveness_std_deviation = []

for eveness_indexes in eveness_index_matrix:
    average_eveness_index.append(np.average(eveness_indexes[1]))
    eveness_std_deviation.append(np.std(eveness_indexes[1]))

plt.figure()
plt.scatter(number_of_robots_list, average_eveness_index)
plt.scatter(number_of_robots_list, eveness_std_deviation)
plt.title("Eveness (Diversity Index) of cell scans of used robots")
plt.ylabel("Eveness Index Value")
plt.xlabel("Total Number of Agents")
plt.legend(["Eveness Index", "Standard Deviation"])
plt.show()

average_used_robots = []
std_used_robots = []

for num_of_used_robots in num_of_used_robots_matrix:
    average_used_robots.append(np.average(num_of_used_robots[1]))
    std_used_robots.append(np.std(num_of_used_robots[1]))

plt.figure()
plt.scatter(number_of_robots_list, average_used_robots)
plt.scatter(number_of_robots_list, std_used_robots)
plt.title("Average number of agents used in mapping")
plt.ylabel("Number of Agents Used")
plt.xlabel("Total Number of Agents")
plt.legend(["Number of Agents Used", "Standard Deviation"])
plt.show()


# TODO: Make Table Output of Data