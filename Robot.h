#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>

#include "GridGraph.h"

struct Coordinates{
    // Coordinates data members
    unsigned int x;
    unsigned int y;

    // Coordinates Constructors
    Coordinates(){
        x = 0;
        y = 0;
    }

    Coordinates(unsigned int a, unsigned int b){
        x = a;
        y = b;
    }

    // Coordinates overloaded operators
    bool operator<(const Coordinates& a) const{ 
        return (x < a.x || (!(x < a.x) && y < a.y));
    }
    bool operator==(const Coordinates& a) const{
        return (x == a.x && y == a.y);
    }
};

class Robot{
    public:
        Robot(int x, int y);

        void scanCell(GridGraph* maze);

        bool move2Cell(int direction);
        bool move2Cell(Coordinates* destination);

        bool pf_BFS(int x, int y);

        bool BFS_pf2NearestUnknownCell(std::vector<Coordinates>* ret_vector);
        void soloExplore(GridGraph* maze);

        std::vector<Coordinates> getValidNeighbours(unsigned int x, unsigned  int y);

        void printRobotMaze();
        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

    private:
        // cartesian cordinates
        unsigned int x_position; // x position within cells 
        unsigned int y_position; // y position within cells

        int number_of_unexplored; // number of unexplored cells encountered

        std::vector<Coordinates> planned_path; 
        
        GridGraph LocalMap; // local_map maintained by robot of areas explored
};



#endif