#include "Robot.h"

Robot::Robot(int x, int y){
    x_position = x; // initialising robots current position
    y_position = y;
}

void Robot::scanCell(GridGraph* maze){ // scans current cell for walls on all sides

    // gathering x edges within maze at robot's current position
    LocalMap.x_edges[y_position][x_position] = maze->x_edges[y_position][x_position];
    LocalMap.x_edges[y_position][x_position+1] = maze->x_edges[y_position][x_position+1];
    
    // gathering y edges within maze at robot's current position
    LocalMap.y_edges[y_position][x_position] = maze->y_edges[y_position][x_position];
    LocalMap.y_edges[y_position+1][x_position] = maze->y_edges[y_position+1][x_position];

    LocalMap.nodes[y_position][x_position] = 1; // setting currently scanned node to 1 to signifiy its been scanned 

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
        if (!LocalMap.y_edges[y_position][x_position]){ // if there is an edge between current node and node above
            y_position--; // move robot to node above
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 2:
        if (!LocalMap.y_edges[y_position+1][x_position]){ // if there is an edge between current node and node below
            y_position++; // move robot to node below
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 3:
        if (!LocalMap.x_edges[y_position][x_position]){ // if there is an edge between current node and node to the left
            x_position--; // move robot to node to the left
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 4:
        if (!LocalMap.x_edges[y_position][x_position+1]){ // if there is an edge between current node and node to the right 
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

std::vector<Coordinates> Robot::getValidNeighbours(unsigned int x, unsigned  int y){

    std::vector<Coordinates> ret_value; // vector of arrays to return
    
    Coordinates buffer; // buffer vector to gather positions of neighbouring nodes

    // check if neighbour to the north is valid and connected via an edge (no wall)
    if(!LocalMap.y_edges[y][x] && LocalMap.nodes[y-1][x] == 1){
        buffer.x = x; 
        buffer.y = y - 1;
        ret_value.push_back(buffer);
    }
    // check if neighbour to the south is valid and connected via an edge (no wall)
    if(!LocalMap.y_edges[y+1][x] && LocalMap.nodes[y+1][x] == 1){
        buffer.x = x; 
        buffer.y = y + 1;
        ret_value.push_back(buffer);
    }
    // check if neighbour to the east is valid and connected via an edge (no wall)
    if(!LocalMap.x_edges[y][x] && LocalMap.nodes[y][x-1] == 1){
        buffer.x = x - 1; 
        buffer.y = y;
        ret_value.push_back(buffer);
    }
    // check if neighbour to the west is valid and connected via an edge (no wall)
    if(!LocalMap.x_edges[y+1][x] && LocalMap.nodes[y][x+1] == 1){
        buffer.x = x + 1; 
        buffer.y = y;
        ret_value.push_back(buffer);
    }

    for(int i = 0; i < ret_value.size(); i ++){
        printf("%d,%d\n",ret_value[i].x,ret_value[i].y);
    }
    return ret_value;
}

bool Robot::pf_BFS(int x_dest, int y_dest){
    
    planned_path.clear(); // clearing planned_path as new path is to be planned using breadth first search

    if(LocalMap.nodes[y_dest][x_dest] == 0) // if the destination node has not been explored
        return false;                       // can't create a path thus return false;

    else if(x_position == x_dest && y_position == y_dest) // if robot is at the destination already
        return true;                                      // no need to move thus return true

    bool ret_value = false; // return value

    std::queue<Coordinates> node_queue; // creating 

    std::map<Coordinates, Coordinates> visited_nodes;

    Coordinates curr_node;
    curr_node.x = x_position;
    curr_node.y = y_position;


    node_queue.push(curr_node);
    visited_nodes.insert({curr_node, curr_node});

    while(node_queue.size() != 0){// while nodes to explore are in the queue
        
        curr_node = node_queue.front(); // gather node from front of queue

        printf("curr node: %d,%d\n", curr_node.x, curr_node.y);

        node_queue.pop(); // removing current node from queue

        if (curr_node.x == x_dest && curr_node.y == y_dest)
            break;
        
        std::vector<Coordinates> valid_neighbours = getValidNeighbours(curr_node.x, curr_node.y);

        for(int i = 0; i < valid_neighbours.size(); i ++){
            
            if(visited_nodes.find(valid_neighbours[i]) != visited_nodes.end()){
                
            }
            else{
                node_queue.push(valid_neighbours[i]);              
                visited_nodes.insert({valid_neighbours[i], curr_node});
            }
        }
    }
    
    while(curr_node.x != x_position || curr_node.y != y_position){
        planned_path.push_back(curr_node);
        printf("hi\n");
        curr_node = visited_nodes[curr_node];
    }

    for(int i = 0; i < planned_path.size(); i ++){
        printf("%d,%d\n", planned_path[i].x,planned_path[i].y);
    }

    return ret_value;    
}

void Robot::printRobotNodes(){ // function to print Robot's map of explored nodes
    printNodes(&LocalMap);
}

void Robot::printRobotXMap(){ // function to print Robot's X edge map 
    printXEdges(&LocalMap);
}
void Robot::printRobotYMap(){ // function to print Robot's Y edge map 
    printYEdges(&LocalMap);
}