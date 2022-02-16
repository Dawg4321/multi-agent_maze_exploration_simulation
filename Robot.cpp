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

std::vector<Coordinates> Robot::getValidNeighbours(unsigned int x, unsigned  int y){ // function to gather valid neighbouring cells of a selected cell based on robot's local map

    std::vector<Coordinates> ret_value; // vector of Coordinates to return
                                        // this will contain the coordinates of valid neighbouring nodes
    
    Coordinates buffer; // buffer structor to gather positions of neighbouring nodes before pushing to vector

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
    if(!LocalMap.x_edges[y][x+1] && LocalMap.nodes[y][x+1] == 1){
        buffer.x = x + 1; 
        buffer.y = y;
        ret_value.push_back(buffer);
    }

    // printing all valid neighbouring nodes of selected node
    for(int i = 0; i < ret_value.size(); i ++){ // TODO: after debugging
        printf("%d,%d\n",ret_value[i].x,ret_value[i].y);
    }

    return ret_value; // returning vector
    // TODO: return by pointer may be desirable
}

bool Robot::pf_BFS(int x_dest, int y_dest){ // function to plan a path for robot to follow from current position to a specified destination
    
    planned_path.clear(); // clearing planned_path as new path is to be planned using breadth first search

    // initial checks to ensure path needs to be planned
    // no point using computation if robot is at destination or destination does not exist on robot's local map
    if(LocalMap.nodes[y_dest][x_dest] == 0) // if the destination node has not been explored
        return false;                       // can't create a path thus return false;

    else if(x_position == x_dest && y_position == y_dest) // if robot is at the destination already
        return true;                                      // no need to move thus return true

    bool ret_value = false; // return value

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm

    std::map<Coordinates, Coordinates> visited_nodes; // map to track explored nodes to their parent node

    // initializing first node to explore from with robot's current coordinates
    Coordinates curr_node(x_position, y_position); 

    node_queue.push(curr_node); // adding first node to explore to node queue
    visited_nodes.insert({curr_node, curr_node}); // adding first node to visited node maps using itself as parent

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        printf("curr node: %d,%d\n", curr_node.x, curr_node.y);

        node_queue.pop(); // removing node from front of the queue

        if (curr_node.x == x_dest && curr_node.y == y_dest){ // if the target node has been located
            ret_value = true; // return true as path found
            break; // break from while loop
        }
        
        std::vector<Coordinates> valid_neighbours = getValidNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i ++){ // iterate through all of the current node's neighbours to see if they have been explored

            if(visited_nodes.contains(valid_neighbours[i]) == 0){//== visited_nodes.end()){
                node_queue.push(valid_neighbours[i]);              
                visited_nodes.insert({valid_neighbours[i], curr_node});   
            }
        }
    }
    
    // TODO: implement handling if path to target location is not found 

    // as a valid path has been found from current position to target using robot's local map
    // must travel from destination back through parent nodes to reconstruct path
    
    curr_node.x = x_dest; // ensuring the current node is pointing to destination
    curr_node.y = y_dest; // this allows for the "walk back" through nodes to start to occur
    
    while(!(curr_node.x == x_position && curr_node.y == y_position)){ // while the starting node has not been found from parents

        planned_path.push_back(curr_node); // add current node to planned path

        for(auto [key, val]: visited_nodes){ // TODO: use better search for key function
            if (key == curr_node){
                curr_node = val;
                break;
            }
        }
    }

    printf("Planned Path\n"); // printing planned path
    for(int i = 0; i <  planned_path.size(); i++){ // TODO: Remove after further debugging
        printf("%d,%d\n",planned_path[i].x,planned_path[i].y);
    }

    return ret_value; 
}

void Robot::findNearestUnknownCell(){

}

void Robot::soloExplore(GridGraph* maze){
    while(1){
        scanCell(maze);

        // find nearest unseen cell

        move2Cell();
    }
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