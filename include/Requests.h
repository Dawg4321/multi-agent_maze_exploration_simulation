#ifndef Messages_H
#define Messages_H

struct RequestHandler; // forward declaration of RequestHandler Class
                       // see RequestHandler.h for definition

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// Generic Message contents
// ~~~~~~~~~~~~~~~~~~~~~~~~~

// important as it is used to gather the request type before processing the request
// when sending a message, set request type to a value specified by the various macros above

struct m_genericRequest{ // template structure used by all requests
    const int request_type; // defines type of request sent
                            // uses previously defined IDs
                            
    m_genericRequest(int req_type): request_type(req_type){ // assigning request type to const int 

    }

    virtual ~m_genericRequest(){} // creating virtual destructor to ensure that child class destructor is called during "delete" operation
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// Inherited Message Contents
// ~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// Defining IDs to various request types
// ~~~~~~~~~~~~~~~~~~~~~~~~~

// IDs used for Robot -> Master Requests
#define shutDownRequest_ID -1
#define addRobotRequest_ID 0
#define updateGlobalMapRequest_ID 1
#define move2CellRequest_ID 2
#define reserveCellRequest_ID 3
#define updateRobotLocationRequest_ID 4
#define getMapRequest_ID 6

// IDs used for Robot -> Master Requests
#define updateRobotStateRequest_ID 5
#define setTargetCell_ID 7

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// Robot -> Master Messages 
// ~~~~~~~~~~~~~~~~~~~~~~~~~

// Standard Message Layout

// ** shutDownRequest Messages **
struct m_shutDownRequest:m_genericRequest{ // request to notify robot master of robot shutting down
    unsigned int robot_id; // id of robot shutting down

    // Constructor
    m_shutDownRequest():m_genericRequest(shutDownRequest_ID){ // assigning request id to request message    
    
    }    
};

struct m_shutDownResponse:m_genericRequest{ // no response needed to shutdown request hence empty
    
    // Constructor
    m_shutDownResponse():m_genericRequest(shutDownRequest_ID){ // assigning request id to response message    
    
    }    
};

// ** addRobotRequest Messages **
struct m_addRobotRequest:m_genericRequest{ // request to notify controller of robot's existance and location

    unsigned int x; // cordinates of robot
    unsigned int y; // this will be updated in robot information of robot master

    RequestHandler* robot_request_handler; // pointer to request handler for master -> robot communications

    // Constructor
    m_addRobotRequest():m_genericRequest(addRobotRequest_ID){ // assigning request id to request message
       
    }  

};
struct m_addRobotResponse:m_genericRequest{ // response containing robot's assigned id
                           // this is critical in allowing the master to differentiate between each robot's incoming requests
    unsigned int robot_id; // id assigned to the robot

    // Constructor
    m_addRobotResponse():m_genericRequest(addRobotRequest_ID){ // assigning request id to request message
       
    } 
};

// ** updateGlobalMapRequest **
struct m_updateGlobalMapRequest:m_genericRequest{
    unsigned int robot_id; // id of robot sending request
    std::vector<bool> wall_info; // content: vector containing information on walls surrounding robot
                                 // [0] = north, [1] = south, [2] = east, [3] = west
                                 // 0 = connection, 1 = wall

    Coordinates cords; // current coordinates of where the read occured

    // Constructor
    m_updateGlobalMapRequest():m_genericRequest(updateGlobalMapRequest_ID){ // assigning request id to request message

    }  
};
struct m_updateGlobalMapResponse:m_genericRequest{ // no response needed hence empty

    // Constructor
    m_updateGlobalMapResponse():m_genericRequest(updateGlobalMapRequest_ID){ // assigning request id to request message

    } 
};

// ** reserveCellRequest **
struct m_reserveCellRequest:m_genericRequest{
    unsigned int robot_id; // id of robot sending request
    
    //Coordinates target_cell; // coordinates correspoonding to full map down a node path if path has been explored already
    Coordinates neighbouring_cell; // neighbouring cell used to enter target cell 
                                   // used to determine which part of maze must be sent back in event node has already been explored
    
   std::deque<Coordinates> planned_path;

    // Constructor
    m_reserveCellRequest():m_genericRequest(reserveCellRequest_ID){ // assigning request id to request message
        
    }  
};
struct m_reserveCellResponse:m_genericRequest{
    bool cell_reserved; // bool determining whether cell was successfully reserved
    Coordinates target_cell; // cell which reserve attempt occured on
    
    // returned portion of global map
    // this is only used if the cell was not reserved
    std::vector<Coordinates>* map_coordinates; // coordinates correspoonding to full map down the requested node path
    std::vector<std::vector<bool>>* map_connections; // wall information corresponding to nodes in map_coordinates
    std::vector<char>* map_status; // node status information of nodes in map_coordinates

    m_reserveCellResponse():m_genericRequest(reserveCellRequest_ID){ // allocating various vectors
        map_coordinates = new std::vector<Coordinates>;
        map_connections = new std::vector<std::vector<bool>>;
        map_status = new std::vector<char>;
    }
    
    ~m_reserveCellResponse(){ // deallocating various vectors
        delete map_connections;
        delete map_coordinates;
        delete map_status;
    }
};

// ** updateRobotLocationRequest **
struct m_updateRobotLocationRequest:m_genericRequest{
    unsigned int robot_id; // id of robot sending request
    Coordinates new_robot_location; // cell which robot now occupies

    // Constructor
    m_updateRobotLocationRequest():m_genericRequest(updateRobotLocationRequest_ID){ // assigning request id to request message
        
    }  
};
struct m_updateRobotLocationResponse:m_genericRequest{ // no response needed hence empty
    
    // Constructor
    m_updateRobotLocationResponse():m_genericRequest(updateRobotLocationRequest_ID){ // assigning request id to request message
        
    }  
};

// ** move2CellRequest **
struct m_move2CellRequest:m_genericRequest{
    unsigned int robot_id; // id of robot sending request
    Coordinates target_cell; // target cell of robot move request

    // Constructor
    m_move2CellRequest():m_genericRequest(move2CellRequest_ID){ // assigning request id to request message
        
    }  
};
struct m_move2CellResponse:m_genericRequest{
    bool can_movement_occur;

    // Constructor
    m_move2CellResponse():m_genericRequest(move2CellRequest_ID){ // assigning request id to request message
        
    }  
};

// ** getMapRequest **
struct m_getMapRequest:m_genericRequest{
    unsigned int robot_id; // id of robot sending request
    Coordinates target_cell; // target cell which robot wants map information to
    Coordinates current_cell; // current cell of robot 
    
    // Constructor
    m_getMapRequest():m_genericRequest(getMapRequest_ID){ // assigning request id to request message
        
    }  
};
struct m_getMapResponse:m_genericRequest{

    // GlobalMap information from robot's position to target cell
    std::vector<Coordinates> map_coordinates; // coordinates correspoonding to full map down the requested node path
    std::vector<std::vector<bool>> map_connections; // wall information corresponding to nodes in map_coordinates
    std::vector<char> map_status; // node status information of nodes in map_coordinates

    // Constructor
    m_getMapResponse():m_genericRequest(getMapRequest_ID){ // assigning request id to request message
        
    }  
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// Master -> Robot Messages 
// ~~~~~~~~~~~~~~~~~~~~~~~~~

// ** updateRobotStateRequest **
struct m_updateRobotStateRequest:m_genericRequest{
    int target_state; // target state to set robot to

    m_updateRobotStateRequest():m_genericRequest(updateRobotStateRequest_ID){ // assigning request id to request message
        
    }
};

// ** setTargetCellRequest **
struct m_setTargetCellRequest:m_genericRequest{
    Coordinates new_target_cell;

    m_setTargetCellRequest():m_genericRequest(setTargetCell_ID){ // assigning request id to request message
        
    }  
};

#endif