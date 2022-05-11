#include "Maze.h"

void Maze::generateInterconnectedMaze(unsigned int x_size, unsigned int y_size){
    maze_xsize = x_size; // setting maze x and y size
    maze_ysize = y_size;

    MazeMap = GridGraph(maze_xsize,maze_ysize); // allocating GridGraph of specified size

    // generating an x_size by y_size sized grid of valid nodes
    for (int i = 0; i < maze_ysize; i++)
        for (int j = 0; j < maze_xsize; j++)
            MazeMap.nodes[i][j] = true;

    // generating valid connections in y direction movement
    // Map is fully interconnected thus only first and last row have '1' values 
    for (int i = 0; i < maze_ysize + 1; i++){
        MazeMap.y_edges[0][i] = true;
        MazeMap.y_edges[maze_ysize][i] = true;
    }

    for (int i = 0; i < maze_xsize + 1; i++){
        MazeMap.x_edges[i][0] = true;
        MazeMap.x_edges[i][maze_xsize] = true;
    }

    return;
}

void Maze::generate4x4SampleMaze(){
    maze_xsize = 4; // setting maze x and y size to 4x4
    maze_ysize = 4;

    MazeMap = GridGraph(maze_xsize,maze_ysize); // allocating GridGraph of specified size

    char n[4][4] = { // marking nodes for maze
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                    };

    for (int i = 0; i < sizeof(n)/sizeof(n[0]); i++) // passing nodes into graph struct
        for (int j = 0; j < sizeof(n[0])/sizeof(n[0][0]); j++)
            MazeMap.nodes[i][j] = n[i][j];
    

    char x[4][5] = { // marking x edges for maze
                        {1, 0, 1, 0, 1},
                        {1, 1, 0, 0, 1},
                        {1, 1, 0, 0, 1},
                        {1, 0, 1, 1, 1},
                    };

    // passing x edges into graph struct
    for (int i = 0; i < sizeof(x)/sizeof(x[0]); i++){ // outer loop (iterate through rows)
        for (int j = 0; j < sizeof(x[0])/sizeof(x[0][0]); j++){ // inner loop (iterate through columns in row)
            if(x[i][j] == 1){ // if there is a wall, set mazemap to have a wall
                MazeMap.x_edges[i][j] = true;
            }
            else{ // if there is no wall, set mazemap to have no wall
                MazeMap.x_edges[i][j] = false;
            }
        }
    }

    char y[5][4] = { // marking y edges for maze
                        {1, 1, 1, 1}, 
                        {0, 0, 1, 0},
                        {0, 0, 1, 1},
                        {0, 1, 0, 0},
                        {1, 1, 1, 1}
                    };

    // passing y edges into graph struct
    for (int i = 0; i < sizeof(y)/sizeof(y[0]); i++){ // outer loop (iterate through rows)
        for (int j = 0; j < sizeof(y[0])/sizeof(y[0][0]); j++){ // inner loop (iterate through columns in row)
            if(y[i][j] == 1){ // if there is a wall, set mazemap to have a wall
                MazeMap.y_edges[i][j] = true;
            }
            else{ // if there is no wall, set mazemap to have no wall
                MazeMap.y_edges[i][j] = false;
            }
        }
    }
    return;
}

void Maze::generate8x8SampleMaze(){
    maze_xsize = 8; // setting maze x and y size to 4x4
    maze_ysize = 8;

    MazeMap = GridGraph(maze_xsize,maze_ysize); // allocating GridGraph of specified size

    char n[8][8] = { // marking nodes for maze
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, 1, 1, 1, 1, 1, 1, 1}
                    };

    for (int i = 0; i < sizeof(n)/sizeof(n[0]); i++) // passing nodes into graph struct
        for (int j = 0; j < sizeof(n[0])/sizeof(n[0][0]); j++)
            MazeMap.nodes[i][j] = n[i][j];
    

    char x[8][9] = { // marking x edges for maze
                        {1, 0, 0, 0, 0, 0, 1, 0, 1},
                        {1, 0, 0, 0, 1, 1, 0, 0, 1},
                        {1, 0, 0, 1, 1, 1, 1, 0, 1},
                        {1, 0, 1, 0, 1, 0, 1, 0, 1},
                        {1, 1, 0, 1, 1, 0, 1, 0, 1},
                        {1, 0, 1, 0, 1, 1, 0, 1, 1},
                        {1, 1, 1, 0, 0, 1, 1, 0, 1},
                        {1, 1, 0, 0, 0, 1, 0, 0, 1},
                    };

    // passing x edges into graph struct
    for (int i = 0; i < sizeof(x)/sizeof(x[0]); i++){ // outer loop (iterate through rows)
        for (int j = 0; j < sizeof(x[0])/sizeof(x[0][0]); j++){ // inner loop (iterate through columns in row)
            if(x[i][j] == 1){ // if there is a wall, set mazemap to have a wall
                MazeMap.x_edges[i][j] = true;
            }
            else{ // if there is no wall, set mazemap to have no wall
                MazeMap.x_edges[i][j] = false;
            }
        }
    }

    char y[9][8] = { // marking y edges for maze
                        {1, 1, 1, 1, 1, 1, 1, 1}, 
                        {0, 1, 1, 1, 1, 0, 1, 0},
                        {1, 1, 1, 0, 0, 0, 1, 0},
                        {0, 1, 0, 0, 0, 0, 0, 1},
                        {0, 1, 1, 0, 1, 0, 1, 0},
                        {1, 0, 0, 0, 0, 0, 1, 1},
                        {0, 0, 1, 1, 0, 1, 0, 0},
                        {0, 0, 1, 1, 1, 0, 1, 0},
                        {1, 1, 1, 1, 1, 1, 1, 1}
                    };

    // passing y edges into graph struct
    for (int i = 0; i < sizeof(y)/sizeof(y[0]); i++){ // outer loop (iterate through rows)
        for (int j = 0; j < sizeof(y[0])/sizeof(y[0][0]); j++){ // inner loop (iterate through columns in row)
            if(y[i][j] == 1){ // if there is a wall, set mazemap to have a wall
                MazeMap.y_edges[i][j] = true;
            }
            else{ // if there is no wall, set mazemap to have no wall
                MazeMap.y_edges[i][j] = false;
            }
        }
    }
    return;
}

void Maze::generateCollisionTest(){
    maze_xsize = 5; // setting maze x and y size to 4x4
    maze_ysize = 1;

    MazeMap = GridGraph(maze_xsize,maze_ysize); // allocating GridGraph of specified size

    char n[1][5] = { // marking nodes for maze
                        {1, 1, 1, 1, 1},
                        };

    for (int i = 0; i < sizeof(n)/sizeof(n[0]); i++) // passing nodes into graph struct
        for (int j = 0; j < sizeof(n[0])/sizeof(n[0][0]); j++)
            MazeMap.nodes[i][j] = n[i][j];
    

    char x[1][6] = { // marking x edges for maze
                        {1, 0, 0, 0, 0, 1},
                    };

    for (int i = 0; i < sizeof(x)/sizeof(x[0]); i++) // passing x edges into graph struct
        for (int j = 0; j < sizeof(x[0])/sizeof(x[0][0]); j++)
            MazeMap.x_edges[i][j] = x[i][j];

    char y[2][5] = { // marking y edges for maze
                        {1, 1, 1, 1, 1}, 
                        {1, 1, 1, 1, 1},
                    };

    for (int i = 0; i < sizeof(y)/sizeof(y[0]); i++) // passing y edges into graph struct
        for (int j = 0; j < sizeof(y[0])/sizeof(y[0][0]); j++)
            MazeMap.y_edges[i][j] = y[i][j];
    
    return;
}

void Maze::generateRandomNxNMaze(unsigned int x_size, unsigned int y_size){
    maze_xsize = x_size;
    maze_ysize = y_size;
    
    MazeMap = GridGraph(x_size,y_size); // allocating GridGraph of specified size

    // need to fill x_edges and y_edges with "true" as a graph with no edges is required
    for(int i = 0; i < MazeMap.x_edges.size(); i++){
        for(int j = 0; j < MazeMap.x_edges[i].size(); j++){
            MazeMap.x_edges[i][j] = true;
        }
    }

    for(int i = 0; i < MazeMap.y_edges.size(); i++){
        for(int j = 0; j < MazeMap.y_edges[i].size(); j++){
            MazeMap.y_edges[i][j] = true;
        }
    }

    const unsigned int num_cells_2_draw = x_size*y_size; // total number of cells which must be visited by cursor before maze is fully mapped

    unsigned int cells_drawn = 0; // count number of cells drawn into maze

    Coordinates cursor(0,0); // selecting node 0,0 as cursor starting location

    MazeMap.nodes[cursor.y][cursor.x] = 1; // marking starting node as visted
    cells_drawn++; // incremented starting cell as it has been drawn

    std::random_device rd; // non-deterministic number generator
    std::mt19937 rand_direction(rd()); // seeding mersenne twister
    std::uniform_int_distribution<> dist(1,4); // distribute random value between 1 and 4 inclusive

    while(cells_drawn < num_cells_2_draw){
        
        Coordinates previous_cell = cursor; // gather previous cell from movement

        // first get cell in valid random direction (e.g. choose a cell that is within the allocated grid)
        int direction = dist(rand_direction); // getting direction to head in x axis (east (0) or west (1))
        int x_movement;
        int y_movement;
        
        switch(direction){
            case 1: // north
            {
                if((int)previous_cell.y - 1 < 0){ // if north is outside the grid
                    direction = 2; // set direction to south
                    y_movement = 1;
                    cursor.y = previous_cell.y + 1;
                }
                else{
                    y_movement = 0;
                    cursor.y = previous_cell.y - 1;
                }

                break;
            }
            case 2: // south
            {
                if((int)previous_cell.y + 1 >= maze_ysize){ // if south is outside the grid
                    direction = 1; // set direction to north
                    y_movement = 0;
                    cursor.y = previous_cell.y - 1;
                }
                else{
                    y_movement = 1;
                    cursor.y = previous_cell.y + 1;
                }

                break;
            }
            case 3: // east
            {
                if((int)previous_cell.x - 1 < 0){ // if east is outside the grid
                    direction = 4; // set direction to west
                    x_movement = 1;
                    cursor.x = previous_cell.x + 1;
                }
                else{
                    x_movement = 0;
                    cursor.x = previous_cell.x - 1;
                }

                break;
            }
            case 4: // west
            {
                if((int)previous_cell.x + 1 >= maze_xsize){ // if west is outside the grid
                    direction = 4; // set direction to west
                    x_movement = 0;
                    cursor.x = previous_cell.x - 1;
                }
                else{
                    x_movement = 1;
                    cursor.x = previous_cell.x + 1;
                }

                break;
            }
        }

        if(MazeMap.nodes[cursor.y][cursor.x] == 0){ // if new neighbour node has not been visited
                                                     // "draw" cell

            MazeMap.nodes[cursor.y][cursor.x] = 1; // mark new node as visited and valid

            if(direction == 1 || direction == 2){ // if moving north or south
                MazeMap.y_edges[previous_cell.y + y_movement][previous_cell.x] = false; // removing wall
            }
            else{ // if moving east or west
                MazeMap.x_edges[previous_cell.y][previous_cell.x + x_movement] = false; // removing wall
            }

            cells_drawn++; // increment number of cells drawn
        }
    }

    return;
}

GridGraph Maze::getMazeMap(){
    return MazeMap;
}

GridGraph* Maze::getMazeMapPointer(){
    return &MazeMap;
}

unsigned int Maze::getMazeXSize(){
    return maze_xsize;
}

unsigned int Maze::getMazeYSize(){
    return maze_ysize;
}

bool Maze::printMaze(){ // function to print layout of maze in an intuitive manner
                        // maze design based off what can be seen here: https://www.chegg.com/homework-help/questions-and-answers/using-c-1-write-maze-solving-program-following-functionality-note-implementation-details-a-q31826669

    std::string logos[5] = { "   ", "---", "|", " ", " R "}; // array with logos to use when printing maze
    
    if(maze_xsize == 0 || maze_ysize == 0) // if maze has not been allocated
        throw "Critical Error: Cannot print maze as it has not been allocated"; // throw except as critical failure

    printf("**Generated Maze**\n"); // printing title and maze information
    printf("Size = %d x %d\n", maze_xsize, maze_ysize);

    int string_pointer = 0; // integer used to determine which logo needs to be printed from logo vector

    int count = 0; // counter to determine if both the column and row edges have been printed
    int i = 0; // counter to track if the whole maze has been printed

    while(!(i >= maze_ysize && count == 1)){ // while loop to determing which row to print (i = node row number)
    
        if(count == 0){ // printing the horizontal walls of maze

            for(int j = 0; j < maze_xsize; j++){
                if(MazeMap.y_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 1; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 0; // print empty space
                }

                printf("+%s", logos[string_pointer].c_str());
            }
            printf("+");
        }
        else{ // printing vertical walls of the maze

            for(int j = 0; j < maze_xsize + 1; j++){
                
                if(MazeMap.x_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 2; // print vetical line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 3; // print empty space
                }

                printf("%s%s", logos[string_pointer].c_str(),logos[0].c_str());
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