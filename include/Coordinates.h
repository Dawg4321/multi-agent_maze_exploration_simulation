#ifndef COORDINATES_H
#define COORDINATES_H

struct Coordinates{ // structure to track x and y coordinates of a position
    // Coordinates data members
    unsigned int x; // x location
    unsigned int y; // y location

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
    bool operator<(const Coordinates& a) const{ // < operator overloaded for usage by map data structure
        return (x < a.x || (!(x < a.x) && y < a.y)); // if left hand side x is bigger than right hand side
                                                     // or if left hand size y is bigger than right hand side y
    }

    bool operator==(const Coordinates& a) const{ // == operator overloaded
        return (x == a.x && y == a.y);
    }
};

#endif