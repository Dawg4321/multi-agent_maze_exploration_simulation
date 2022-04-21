from enum import unique
import json
import matplotlib.pyplot as plt
import numpy as np

def getNumberofCellsScanned(simulation_json):
    file = open(simulation_json) # gathering json data of simulation

    simulation = json.load(file) # placing json data into a dictionary

    simulation_info = simulation["Info"] # getting number of turns in simulation
    num_turns = simulation_info["Total_Turns_Taken"]
    num_robots = simulation_info["Number_of_Robots"]

    simulation_data = simulation["Simulation"]

    cells_scan_operations = [0] * num_robots
    
    robot_ids = range(1, num_robots + 1)

    for turn in range(1, num_turns): # iterate through data in various turns
        if turn != 3:
            transactions = simulation_data[turn-1]["Turn_" + str(turn)] # get the transactions which occured during the turn
            for t in transactions: # iterate through transactions in turn
                if t["Type"] == 1: # if a scan cell operation occured
                    cells_scan_operations[t["Request"]["ID"] - 1] += 1  # incrementing number of cells scanned tracked by specific robot
        else:
            scanned_cells = []

            transactions = simulation_data[turn-1]["Turn_" + str(turn)] # get the transactions which occured during the turn
            for t in transactions: # iterate through transactions in turn
                if t["Type"] == 1: # if a scan cell operation occured
                    first = t["Request"]["Current_Cell"]["x_pos"]
                    second = t["Request"]["Current_Cell"]["y_pos"]
                    if scanned_cells.count((first, second)) == 0:
                        cells_scan_operations[t["Request"]["ID"] - 1] += 1  # incrementing number of cells scanned tracked by specific robot
                        scanned_cells.append((first, second))
    
    return cells_scan_operations, robot_ids # returning the number of cells scanned by each robot



def getNumberofTurns(simulation_json): # simple function to gather the number of turns taken to complete a simulation
    file = open(simulation_json) # gathering json data of simulation

    simulation = json.load(file) # placing json data into a dictionary

    num_turns = simulation["Info"]["Total_Turns_Taken"] # getting number of turns taken in simulation

    return num_turns # returning number of turns

def getNumberofRobots(simulation_json): # gets the number of robots used in a simulation
    file = open(simulation_json) # gathering json data of simulation

    simulation = json.load(file) # placing json data into a dictionary

    num_robots = simulation["Info"]["Number_of_Robots"] # getting number of robots in simulation

    return num_robots

def analyseExploredCellsOverTurns(simulation_data, num_turns): # plots and returns the number of explored cells over turns in simulation
                                                                # pass dictionary of [Simulation] from json and number of turns taken into function
    explored_cells_over_turns = []
    cells_scanned_in_turn = 0

    for turn in range(1, num_turns):
        transactions = simulation_data[turn-1]["Turn_" + str(turn)]
        for t in transactions:
            if t["Type"] == 1:
                cells_scanned_in_turn += 1
        explored_cells_over_turns.append(cells_scanned_in_turn)

    print(explored_cells_over_turns)

    plt.figure()
    plt.title("Number of Explored Cells over Turns")
    plt.xlabel("Turns")
    plt.plot(range(1,num_turns), explored_cells_over_turns)
    plt.show()

    return explored_cells_over_turns
"""
location = "/home/ryan/code/git_workspace/multi-agent_maze_exploration_simulation/build/"

data_json = "Simulation.json"

file = open(location + data_json) # gathering json data

simulation = json.load(file) # placing json data into a dictionary

simulation_info = simulation["Info"]

simulation_data = simulation["Simulation"]

num_turns = simulation_info["Total_Turns_Taken"]

analyseExploredCellsOverTurns(simulation_data, num_turns) """


#print(transaction_data)

#df = pd.read_json(location + data_json)

#print(df)