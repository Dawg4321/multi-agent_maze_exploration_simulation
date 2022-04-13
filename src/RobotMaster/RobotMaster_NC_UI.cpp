#include "RobotMaster_NC_UI.h"

RobotMaster_NC_UI::RobotMaster_NC_UI(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize): RobotMaster(r, num_of_robots, xsize, ysize){

}

RobotMaster_NC_UI::~RobotMaster_NC_UI(){

}

void RobotMaster_NC_UI::handleIncomingRequest(Message* incoming_request){

    m_genericRequest* r = (m_genericRequest*) incoming_request->msg_data; // use generic message pointer to gather request type

    switch (r->request_type){ // determining type of request before processing

        case addRobotRequest_ID: // addRobot request
        {
            addRobotRequest(incoming_request);

            // if all robots have been added
            // send signal to all robots to begin exploration
            if(tracked_robots->size() == max_num_of_robots)
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