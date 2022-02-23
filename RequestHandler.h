#ifndef REQUESTHANDLER_H
#define REQUESTHANDLER_H

#include <vector>
#include <queue>
#include <pthread.h>
#include <semaphore.h>

struct Message{
    char request_type; // type of request sent to Robot Controller
                       // 0 = add_Robot
                       // 1 = updateGlobalMap
                       
    std::vector<void*> msg_data; // message data for controller to unpack

    // semaphores used to synchronize thread communication
    sem_t* response_semaphore; // semaphore to control whether response has sent read
    sem_t* ack_semaphore;      // semaphore to control whether response has been analysed

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