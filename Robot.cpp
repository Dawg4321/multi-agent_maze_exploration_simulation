#include "Robot.h"

Robot::Robot(int x, int y){
    x_position = x; // initialising robots current position
    y_position = y;
}

void Robot::scanCell(GridGraph* maze){ // scans current cell for walls on all sides

    // gathering x edges within maze at robot's current position
    local_map.x_edges[y_position][x_position] = maze->x_edges[y_position][x_position];
    local_map.x_edges[y_position][x_position+1] = maze->x_edges[y_position][x_position+1];
    
    // gathering y edges within maze at robot's current position
    local_map.y_edges[y_position][x_position] = maze->y_edges[y_position][x_position];
    local_map.y_edges[y_position+1][x_position] = maze->y_edges[y_position+1][x_position];

    local_map.nodes[y_position][x_position] = 1; // setting currently scanned node to 1 to signifiy its been scanned 

    return;
}

bool Robot::move2Cell(int direction){ // function to move robot depending on location of walls within local map
                                      // ensure current cell is scanned with scanCell before calling this function
                                      // direction = 1 ^
                                      // direction = 2 v
                                      // direction = 3 <
                                      // direction = 4 >

    bool ret_value = false; // return variable
                            // used to track whether move operation was a success

    switch (direction){ // switch statement to move robot in specific direction based on know information from local map
    case 1:
        if (!local_map.y_edges[y_position][x_position]){ // if there is an edge between current node and node above
            y_position--; // move robot to node above
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 2:
        if (!local_map.y_edges[y_position+1][x_position]){ // if there is an edge between current node and node below
            y_position++; // move robot to node below
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 3:
        if (!local_map.x_edges[y_position][x_position]){ // if there is an edge between current node and node to the left
            x_position--; // move robot to node to the left
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 4:
        if (!local_map.x_edges[y_position][x_position+1]){ // if there is an edge between current node and node to the right 
            x_position++; // move robot to node to the right
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    default: // if invalid direction is passed into function
        break;
    }
    if (ret_value) // if robot position movement successfull
        std::cout << "Robot Sucessfully moved to " << x_position  << ", "<< y_position << "\n";
    return ret_value; // ret_value = true if robot moved sucessfully
                      // ret_value = false if robot failed to move
}

bool Robot::pf_FloodFill(int dest_x, int dest_y){ // TODO: implement simple flood fill pathfinding
    bool ret_value = false;

    return ret_value;    
}

void Robot::printRobotNodes(){ // function to print Robot's map of explored nodes
    printNodes(&local_map);
}

void Robot::printRobotXMap(){ // function to print Robot's X edge map 
    printXEdges(&local_map);
}
void Robot::printRobotYMap(){ // function to print Robot's Y edge map 
    printYEdges(&local_map);
}