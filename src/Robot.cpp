#include "Robot.h"

Robot::Robot(int x, int y, unsigned int xsize, unsigned int ysize){ // constructor used by child classes to allocate LocalMap
    
    x_position = x; // initialising robots current position with passed in values
    y_position = y;

    maze_xsize = xsize; // getting and storing maze size for print out and map allocation purposes
    maze_ysize = ysize; // this is not used by the proposed algorithms for exploration purposes#

    number_of_unexplored = 1; // set unknown cells to 1 as current occupied cell is unknown to robot
 
    LocalMap = new GridGraph(xsize,ysize); // allocating local map to be size of maze
    
}

Robot::~Robot(){
    delete LocalMap; // deleting LocalMap as it was dynamically allocated
}

std::vector<bool> Robot::scanCell(GridGraph* maze){ // scans current cell for walls on all sides
                                       // function assumes current cell has not been scanned yet
    std::vector<bool> ret_vector;

    number_of_unexplored--;
    
    // gathering x edges within maze at robot's current position
    LocalMap->x_edges[y_position][x_position] = maze->x_edges[y_position][x_position]; // east
    LocalMap->x_edges[y_position][x_position+1] = maze->x_edges[y_position][x_position+1]; // west
    
    // gathering y edges within maze at robot's current position
    LocalMap->y_edges[y_position][x_position] = maze->y_edges[y_position][x_position]; // north
    LocalMap->y_edges[y_position+1][x_position] = maze->y_edges[y_position+1][x_position]; // south

    // placing edges within return vector for usage by RobotMaster -> [0] = north, [1] = south, [2] = east, [3] = west
    ret_vector.push_back(maze->y_edges[y_position][x_position]); // north
    ret_vector.push_back(maze->y_edges[y_position+1][x_position]); // south
    ret_vector.push_back(maze->x_edges[y_position][x_position]); // east
    ret_vector.push_back(maze->x_edges[y_position][x_position+1]); // west
    

    // updating state of current node 
    LocalMap->nodes[y_position][x_position] = 1; // setting currently scanned node to 1 to signifiy its been scanned 

    // updating state of neighbouring nodes to unexplored if possible
    
    // checking north
    if(!LocalMap->y_edges[y_position][x_position] && LocalMap->nodes[y_position - 1][x_position] == 0){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position - 1][x_position] = 2; // if unexplored and no wall between robot and cell, set northern node to unexplored
        number_of_unexplored++;
    }
    // checking south
    if(!LocalMap->y_edges[y_position + 1][x_position] && LocalMap->nodes[y_position + 1][x_position] == 0){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position + 1][x_position] = 2; // if unexplored and no wall between robot and cell, set southern node to unexplored
        number_of_unexplored++;
    }
    // checking east
    if(!LocalMap->x_edges[y_position][x_position] && LocalMap->nodes[y_position][x_position - 1] == 0){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position][x_position - 1] = 2; // if unexplored and no wall between robot and cell, set eastern node to unexplored
        number_of_unexplored++;
    }
    // checking wast
    if(!LocalMap->x_edges[y_position][x_position + 1] && LocalMap->nodes[y_position][x_position + 1] == 0){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position][x_position + 1] = 2; // if unexplored and no wall between robot and cell, set western node to unexplored
        number_of_unexplored++;
    }
    return ret_vector;
}

bool Robot::move2Cell(int direction){ // function to move robot depending on location of walls within local map
                                      // ensure current cell is scanned with scanCell before calling this function
                                      // direction = 1 ^ north
                                      // direction = 2 v south
                                      // direction = 3 < east
                                      // direction = 4 > west

    bool ret_value = false; // return variable
                            // used to track whether move operation was a success

    switch (direction){ // switch statement to move robot in specific direction based on know information from local map
        case 1:
        {
            if (!LocalMap->y_edges[y_position][x_position]){ // if there is an edge between current node and node above
                y_position--; // move robot to node above
                ret_value = true; // ret_value = true as movement was a success
            }
            break;
        }
        case 2:
        {
            if (!LocalMap->y_edges[y_position+1][x_position]){ // if there is an edge between current node and node below
                y_position++; // move robot to node below
                ret_value = true; // ret_value = true as movement was a success
            }
            break;
        }
        case 3:
        {
            if (!LocalMap->x_edges[y_position][x_position]){ // if there is an edge between current node and node to the left
                x_position--; // move robot to node to the left
                ret_value = true; // ret_value = true as movement was a success
            }
            break;
        }
        case 4:
        {
            if (!LocalMap->x_edges[y_position][x_position+1]){ // if there is an edge between current node and node to the right 
                x_position++; // move robot to node to the right
                ret_value = true; // ret_value = true as movement was a success
            }
            break;
        }
        default: // if invalid direction is passed into function
        {
            break;
        }
    }

    return ret_value; // ret_value = true if robot moved sucessfully
                      // ret_value = false if robot failed to move
}

bool Robot::move2Cell(Coordinates destination){ // overloaded version of move2Cell using destination
                                                 // converts target destination to a direction then uses original move2Cell function

    // first must determine direction based on two values
    int dx = x_position - destination.x; // gather change in x and y directions
    int dy = y_position - destination.y;

    int direction = 0;  // variable to store determined direction 
                        // direction = 1 ^ north
                        // direction = 2 v south
                        // direction = 3 < east
                        // direction = 4 > west

    if(dy == 1 && dx == 0){ // if moving north
        direction = 1;
    }
    else if(dy == -1 && dx == 0){ // if moving south
        direction = 2;
    }
    else if(dx == 1 && dy == 0){ // if moving east
        direction = 3;
    }
    else if (dx == -1 && dy == 0){ // if moving west
        direction = 4;
    }
    else{
        throw "Error: invalid movement passed into move2Cell";
        return false; // return false as movement failed dur to invalid direction
    }

    return move2Cell(direction); // moving robot and returning
    
}

std::vector<Coordinates> Robot::getValidNeighbours(int x, int y){ // function to gather valid neighbouring cells of a selected cell based on robot's local map
    std::vector<Coordinates> neighbours; // vector of Coordinates to return
                                         // this will contain the coordinates of valid neighbouring nodes

    if(LocalMap->nodes[y][x] == 2 || LocalMap->nodes[y][x] == 3){ // if the current node is unexplored or leads to a dead end, don't get nearest neighbours
        return neighbours;
    }

    Coordinates buffer; // buffer structor to gather positions of neighbouring nodes before pushing to vector

    // check if neighbour to the north is valid and connected via an edge (no wall)
    if(!LocalMap->y_edges[y][x]){
        if (LocalMap->nodes[y-1][x] == 1 || LocalMap->nodes[y-1][x] == 2){
            buffer.x = x; 
            buffer.y = y - 1;
            neighbours.push_back(buffer);
        }
    }
    // check if neighbour to the south is valid and connected via an edge (no wall)
    if(!LocalMap->y_edges[y+1][x]){
        if (LocalMap->nodes[y+1][x] == 1 || LocalMap->nodes[y+1][x] == 2){
            buffer.x = x; 
            buffer.y = y + 1;
            neighbours.push_back(buffer);
        }
    }
    // check if neighbour to the east is valid and connected via an edge (no wall)
    if(!LocalMap->x_edges[y][x]){
        if (LocalMap->nodes[y][x-1] == 1 || LocalMap->nodes[y][x-1] == 2){
            buffer.x = x - 1; 
            buffer.y = y;
            neighbours.push_back(buffer);
        }
    }
    // check if neighbour to the west is valid and connected via an edge (no wall)
    if(!LocalMap->x_edges[y][x+1]){
        if (LocalMap->nodes[y][x+1] == 1 || LocalMap->nodes[y][x+1] == 2){
            buffer.x = x + 1; 
            buffer.y = y;
            neighbours.push_back(buffer);
        }
    }


    std::random_shuffle(neighbours.begin(), neighbours.end()); // randomizing neighbours to remove bias when selecting a neighbor at intersections

    return neighbours; // returning vector
}

bool Robot::pf_BFS(int x_dest, int y_dest){ // function to plan a path for robot to follow from current position to a specified destination
    
    planned_path.clear(); // clearing planned_path as new path is to be planned using breadth first search

    bool path_found = false; // variable to track whether a path has been found

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm

    std::map<Coordinates, Coordinates> visited_nodes; // map to track explored nodes to their parent node

    // initializing first node to explore from with robot's current coordinates
    Coordinates curr_node(x_position, y_position); 

    node_queue.push(curr_node); // adding first node to explore to node queue
    visited_nodes.insert({curr_node, curr_node}); // adding first node to visited node maps using itself as parent

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        //printf("curr node: %d,%d\n", curr_node.x, curr_node.y);

        node_queue.pop(); // removing node from front of the queue

        if (curr_node.x == x_dest && curr_node.y == y_dest){ // if the target node has been located
            path_found = true; // return true as path found
            break; // break from while loop
        }
        
        std::vector<Coordinates> valid_neighbours = getValidNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i ++){ // iterate through all of the current node's neighbours to see if they have been explored
            
            bool node_in_map = false; // variable to track whether neighbour is in map
            
            for(auto [key, val]: visited_nodes){ // iterate through visited_nodes
                if (key == valid_neighbours[i]){ // if current neighbour is in map
                    node_in_map = true;          // set node_in_map
                    break;
                }
            }
            if(!node_in_map){ // if neighbour not found in map
                node_queue.push(valid_neighbours[i]); // add to node_queue and visited nodes
                visited_nodes.insert({valid_neighbours[i], curr_node}); 
            }
        }
    }
    
    if(!path_found){ // if no path found, return false
        return path_found;
    }

    // as a valid path has been found from current position to target using robot's local map
    // must travel from destination back through parent nodes to reconstruct path
    
    curr_node.x = x_dest; // ensuring the current node is pointing to destination
    curr_node.y = y_dest; // this allows for the "walk back" through nodes to start to occur
    
    while(!(curr_node.x == x_position && curr_node.y == y_position)){ // while the starting node has not been found from parents

        planned_path.push_front(curr_node); // add current node to top of planned path "stack"

        for(auto [key, val]: visited_nodes){ // TODO: use better search for key function
            if (key == curr_node){
                curr_node = val;
                break;
            }
        }
    }

    return path_found; 
}

bool Robot::BFS_exitCondition(Coordinates* node_to_test){
    return (LocalMap->nodes[node_to_test->y][node_to_test->x] == 2);
}

bool Robot::BFS_pf2NearestUnknownCell(std::deque<Coordinates>* ret_stack){

    ret_stack->clear(); // clearing planned_path as new path is to be planned using breadth first search

    bool ret_value = false; // return value

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm

    std::map<Coordinates, Coordinates> visited_nodes; // map to track explored nodes to their parent node

    // initializing first node to explore from with robot's current coordinates
    Coordinates curr_node(x_position, y_position); 

    node_queue.push(curr_node); // adding first node to explore to node queue
    visited_nodes.insert({curr_node, curr_node}); // adding first node to visited node maps using itself as parent

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        //printf("curr node: %d,%d\n", curr_node.x, curr_node.y);
        std::vector<Coordinates>  valid_neighbours; // creating cector to store valid neigbours in following else if statement

        if (BFS_exitCondition(&curr_node)){ // if exit conditon has been met
            ret_value = true; // return true as path to unexplored node found
            break; // break from while loop
        }
        // gathering nearest neighbours and checking if the node is a dead end (e.g. one neighbour and has already been visited). 
        // if it is a dead end, mark nodes along dead end path with 3. this prevents these paths from being searched during pathfinding
        else if(valid_neighbours = getValidNeighbours(curr_node.x, curr_node.y); valid_neighbours.size() == 1 && LocalMap->nodes[curr_node.y][curr_node.x] == 1 && curr_node.x != x_position && curr_node.y != y_position){
                    Coordinates node_to_test = curr_node; // gathering dead end node before testing
                    std::vector<Coordinates> neighbours; // vector to store neighbours during branch removal
                    do{
                        LocalMap->nodes[node_to_test.y][node_to_test.x] = 3; // marking node leading to dead end as 3

                        for(auto [key, val]: visited_nodes){ // searching to find parent node (node before this node)
                            if (key == node_to_test){ // if parent node found
                                node_to_test = val; // test it on next iteration
                                break;
                            }
                        }

                        neighbours = getValidNeighbours(node_to_test.x, node_to_test.y); // check next node's neighbours to see if it still leads to dead end
                    }while(neighbours.size() == 1 && node_to_test.x != x_position && node_to_test.y != y_position); // checking if node leads to a dead end (1 valid neighbour)
                }

        node_queue.pop(); // removing node from front of the queue as new nodes must be added to queue

        for(int i = 0; i < valid_neighbours.size(); i ++){ // iterate through all of the current node's neighbours to see if they have been explored
            bool node_in_map = false; // variable to track whether neighbour is in map
            
            for(auto [key, val]: visited_nodes){ // iterate through visited_nodes
                if (key == valid_neighbours[i]){ // if current neighbour is in map
                    node_in_map = true;          // set node_in_map
                    break;    
                }
            }
            if(!node_in_map){ // if neighbour not found in map
                node_queue.push(valid_neighbours[i]); // add to node_queue and visited nodes
                visited_nodes.insert({valid_neighbours[i], curr_node});
            }
        }
    }

    if (ret_value == false){ // if ret_value == fsle, 
        BFS_noPathFound(); // function which handles if a path is not found
                           // does nothing in robot class as meant to be handled by child classes

        return ret_value; // can return false as no point reconstructing path
    }

    // as a valid path has been found from current position to target using robot's local map
    // must travel from destination back through parent nodes to reconstruct path

    while(!(curr_node.x == x_position && curr_node.y == y_position)){ // while the starting node has not been found from parents

        ret_stack->push_front(curr_node); // add current node to top of planned path "stack"

        for(auto [key, val]: visited_nodes){ // iterating through visited nodes map
            if (key == curr_node){ // if parent node to current node it found
                curr_node = val; // set parent node as current node
                break;
            }
        }
    }
    return ret_value; 
}

bool Robot::printRobotMaze(){ // function to print robot's local map of maze
                              // maze design based off what can be seen here: https://www.chegg.com/homework-help/questions-and-answers/using-c-1-write-maze-solving-program-following-functionality-note-implementation-details-a-q31826669

    std::string logos[8] = { "   ", "---", "|", " ", " R ", " . ", " X ", " * "}; // array with logos to use when printing maze
    
    if(maze_xsize == 0 || maze_ysize == 0){ // if maze has not been allocated
        throw "Critical Error: Cannot print maze as it has not been allocated";
        return false; // return false as printing failed
    }

    printf("*Robot Local Map**\n"); // printing title and maze information

    int string_pointer = 0; // integer used to determine which logo needs to be printed from logo vector

    int count = 0; // counter to determine if both the column and row edges have been printed
    int i = 0; // counter to track if the whole maze has been printed

    while(!(i >= maze_ysize && count == 1)){ // while loop to determing which row to print (i = node row number)
    
        if(count == 0){ // printing the horizontal walls of maze

            for(int j = 0; j < maze_xsize; j++){

                if(LocalMap->y_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 1; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 0; // print horizontal line
                }

                printf("+%s", logos[string_pointer].c_str());
            }
            printf("+");
        }
        else{ // printing vertical walls of the maze

            for(int j = 0; j < maze_xsize + 1; j++){

                // checking the walls between two nodes (e.g. wall?, no wall?)
                if(LocalMap->x_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 2; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 3; // print horizontal line
                }
                printf("%s", logos[string_pointer].c_str());

                // checking contents of a node (e.g. does it have a robot?)
                if (j >= maze_xsize){ // if iterating outside of valid node
                    string_pointer = 3; // print empty space as node is outside of maze
                }
                else if(j == x_position && i == y_position){ // if current node is the robot's location
                    string_pointer = 4; // print R for robot
                }
                else if(LocalMap->nodes[i][j] == 0){ // if current node is invalid (unseen and unexplored)
                    string_pointer = 6; // print I for invalid cell
                }
                else if(LocalMap->nodes[i][j] == 2){ // if current node has been seen but not explored
                    string_pointer = 7; // print * for seen node
                }
                else{ // if current cell has been seen and explored (valid)
                    string_pointer = 0; // print empty space
                }


                printf("%s",logos[string_pointer].c_str());
            }
        }
        
        if (count == 1){ // if both the row and column corresponding to i value have been printed
            count = 0; // reset count value
            i++; // increment i to access next row
        }
        else // if only the column edges corresponding to i have been printed
            count++;

        printf("\n");

    }

    return true; // return true as printing succeeded
}

void Robot::printRobotNodes(){ // function to print Robot's map of explored nodes
    printNodes(LocalMap);
}

void Robot::printRobotXMap(){ // function to print Robot's X edge map 
    printXEdges(LocalMap);
}
void Robot::printRobotYMap(){ // function to print Robot's Y edge map 
    printYEdges(LocalMap);
}

void Robot::setLocalMap(GridGraph* new_map){ // sets LocalMap
                                             // do not call this function if robot has already explored
    *LocalMap = *new_map; // setting gridgraph value

    for(int i = 0; i < LocalMap->nodes.size(); i++){ // need to account for all unexplored cells in new map
        for(int j = 0; j < LocalMap->nodes[i].size(); j++){
            if(LocalMap->nodes[i][j] == 2)
                number_of_unexplored++;
        }
    }

    return;
}

void Robot::BFS_noPathFound(){ // function which handles if a path is not found
    return;
}