#ifndef REQUESTHANDLER_H
#define REQUESTHANDLER_H

#include <vector>
#include <queue>
#include <pthread.h>
#include <semaphore.h>

// ~~~ Macros for easier code readability when specifiying messge type ~~~
#define t_Request false
#define t_Response true

struct Message{
    // content of messages and responses, information will be encapsulated using messages defined in RequestTemplates.h
    void* msg_data; // message data for controller to unpack
    void* return_data; //  data returned from message request

    bool message_type; // used to determine if sent message is a request or a response
                       // true = response message
                       // false = request message
                       // use above macros for simplicity

    int response_id; // if message = request, give an expected response id to allow sender check for stale response
                     // if message = response, helps original sender determine if request is stale (e.g. robot was forced to change state thus previous request is useless)

    // semaphores used to synchronize thread communication
    sem_t* res_sem; // semaphore to control whether response has sent read
    sem_t* ack_sem; // semaphore to control whether response has been analysed

    Message(bool mess_type, int res_id){
        message_type = mess_type; // assigning message type
        response_id = res_id; // assigning response id
    }
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