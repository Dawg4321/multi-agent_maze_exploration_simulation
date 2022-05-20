#ifndef REQUESTHANDLER_H
#define REQUESTHANDLER_H

#include <vector>
#include <queue>
#include <pthread.h>

#include "Coordinates.h"
#include "Requests.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// Message Template
// ~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~ Macros for easier code readability when specifiying messge type ~~~
//#define t_Request false
//#define t_Response true

enum MessageType{t_Request,t_Response};

struct Message{
    // content of messages and responses, information will be encapsulated using messages defined in RequestTemplates.h
    m_genericRequest* msg_data; // message data for master to unpack
    
    MessageType message_type; // used to determine if sent message is a request or a response within robot
                       // true = message is a response to a previous request
                       // false = message is an order from the robot supervisor
                       // use above macros for simplicity

    int transaction_id; // if message = request, give an expected transaction id to allow sender check for stale response
                        // if message = response, helps original sender determine if request is stale (e.g. robot was forced to change state thus previous request is useless)

    Message(MessageType mess_type, int res_id){
        message_type = mess_type; // assigning message type
        transaction_id = res_id; // assigning response id
    }
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~
// RequestHandler Class
// ~~~~~~~~~~~~~~~~~~~~~~~~~

class RequestHandler{ // class to handle communication between robots and robot master
                      // replace this with bluetooth or equivalent in hardware implementation
    public:
        RequestHandler(); // constructor
        ~RequestHandler(); // destructor

        void sendMessage(Message* m); // add a message address to the message queue
        Message* getMessage(); // return address of message from front of message queue
        int getNumberofMessages(); // returns number of requests in msg_queue

    private:
        std::queue<Message*> msg_queue; // FIFO queue of messages sent to RobotMaster Class
        pthread_mutex_t msg_mutex; // mutex for controlling all operations on msg_queue by threads
};

#endif