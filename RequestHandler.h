#ifndef REQUESTHANDLER_H
#define REQUESTHANDLER_H

#include <vector>
#include <queue>
#include <pthread.h>

struct Message{
    char request_type; // type of request sent to Robot Controller
                       // 0 = add_Robot
                       // 1 = updateGlobalMap
                       
    std::vector<void*> msg_data; // message data for controller to unpack

    pthread_cond_t* condition_var;  // pointer to condition variable to notify sender that response is ready in return_data
    pthread_mutex_t* acknowlgement_mutex; // pointer to condition variable which is used to block controller until robot is done with response 
    std::vector<void*> return_data; //  data returned from message request
};

class RequestHandler{
    public:
        RequestHandler(); // constructor
        ~RequestHandler(); // destructor

        void sendMessage(Message* m); // add a message address to the message queue
        Message* getMessage(); // return address of message from front of message queue
    private:
        std::queue<Message*> msg_queue; // FIFO queue of messages sent to RobotMaster Class
        pthread_mutex_t msg_mutex; // mutex for controlling all operations on msg_queue by threads
};

#endif