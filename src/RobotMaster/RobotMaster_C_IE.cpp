#include "RobotMaster_C_IE.h"

RobotMaster_C_IE::RobotMaster_C_IE(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize): RobotMaster(r, num_of_robots, xsize, ysize), RobotMaster_IE(xsize, ysize), RobotMaster_C(xsize, ysize){

}

RobotMaster_C_IE::~RobotMaster_C_IE(){

}

void RobotMaster_C_IE::handleIncomingRequest(Message* incoming_request){

    m_genericRequest* r = (m_genericRequest*) incoming_request->msg_data; // use generic message pointer to gather request type

    switch (r->request_type){ // determining type of request before processing

        case addRobotRequest_ID: // addRobot request
        {
            addRobotRequest(incoming_request);

            // if all robots have been added
            // send signal to all robots to begin exploration
            if(tracked_robots.size() == max_num_of_robots)
                updateAllRobotState(1); // updating all robot states to 1
                                        // this causes them to all begin exploring by first scanning their cell
            break;
        }
        case updateGlobalMapRequest_ID: // updateGlobalMap request
        {   
            updateGlobalMapRequest(incoming_request);

            if(number_of_unexplored_cells < 1){ // if no more cells to explore
                updateAllRobotState(-1); // tell all robots to shut down
                accepting_requests = false; // set robot master to ignore all incoming requests which are not a shut down request
            }

            break;
        }
        case move2CellRequest_ID: // move2cell request
        {
            move2CellRequest(incoming_request);

            break;
        }
        case reserveCellRequest_ID: // reserveCell request (robot wants to start exploring from a cell without other robots using it)
        {
            reserveCellRequest(incoming_request);

            break;
        }
        case updateRobotLocationRequest_ID: // update Robot Location  (tells master that robot has completed move operation)
        {
            updateRobotLocationRequest(incoming_request);

            break;
        }
        default:
        {
            break;
        }
    }

    return;
}