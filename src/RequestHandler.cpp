#include "RequestHandler.h"

RequestHandler::RequestHandler(/* args */){
    pthread_mutex_init(&msg_mutex, NULL); // intialzing msg_mutex
                                          // TODO: add checks for if mutex is initialized properly
}

RequestHandler::~RequestHandler(){
    pthread_mutex_destroy(&msg_mutex); // destroy message mutex as no longer needed
}

void RequestHandler::sendMessage(Message* m){ // TODO: add basic message layout validation
    pthread_mutex_lock(&msg_mutex); // locking mutex so msg_queue can be modified safely 

    msg_queue.push(m); // add m to end of FIFO msg_queue

    pthread_mutex_unlock(&msg_mutex);// unlocking mutex so msg_queue can be modified by other threads 

    return;
}

Message* RequestHandler::getMessage(){
    Message* temp;

    pthread_mutex_lock(&msg_mutex); // locking mutex so msg_queue can be accessed and modified safely 
    
    if(msg_queue.size() > 0){
        temp = msg_queue.front(); // gathering msg from front of msg_queue
        msg_queue.pop(); // removing front of msg_queue
    }
    else{
        temp = NULL;
    }

    pthread_mutex_unlock(&msg_mutex);// unlocking mutex so msg_queue can be modified by other threads 

    return temp; // returning message from front of queue
}

int RequestHandler::getNumberofMessages(){
    pthread_mutex_lock(&msg_mutex); // locking mutex so msg_queue can be modified safely 

    int queue_size = msg_queue.size(); // get queue size

    pthread_mutex_unlock(&msg_mutex);// unlocking mutex so msg_queue can be modified by other threads 

    return queue_size;
}