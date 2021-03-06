#ifndef COORDINATES_H
#define COORDINATES_H

#define NULL_COORDINATE Coordinates(-1,-1)

struct Coordinates{ // structure to track x and y coordinates of a position
    // Coordinates data members
    int x; // x location
    int y; // y location

    // Coordinates Constructors
    Coordinates(){
        x = 0;
        y = 0;
    }

    Coordinates(int x_pos, int y_pos){
        x = x_pos;
        y = y_pos;
    }

    ~Coordinates(){ // Coordinates destructor
    }

    // Coordinates overloaded operators
    bool operator<(const Coordinates& a) const{ // < operator overloaded for usage by map data structure
        return (x < a.x || (!(x < a.x) && y < a.y)); // if left hand side x is bigger than right hand side
                                                     // or if left hand size y is bigger than right hand side y
    }

    bool operator==(const Coordinates& a) const{ // == operator overloaded
        return (x
         == a.x && y == a.y);
    }
    bool operator!=(const Coordinates& a) const{ // != operator overloaded
        return (!(x == a.x && y == a.y));
    }
    Coordinates& operator=(const Coordinates& a){ // copy assignment operator overloaded
        x = a.x;
        y = a.y;
        return *this;
    }
 
};

#endif