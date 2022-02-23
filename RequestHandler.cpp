#include "RequestHandler.h"

RequestHandler::RequestHandler(/* args */){

}

RequestHandler::~RequestHandler(){

}

void RequestHandler::sendMessage(Message m){ // TODO: add basic message layout validation
    pthread_mutex_lock(&msg_mutex); // locking mutex so msg_queue can be modified safely 

    msg_queue.push(m); // add m to end of FIFO msg_queue

    pthread_mutex_unlock(&msg_mutex);// unlocking mutex so msg_queue can be modified by other threads 

    return;
}

Message RequestHandler::getMessage(){
    pthread_mutex_lock(&msg_mutex); // locking mutex so msg_queue can be modified safely 

    Message temp = msg_queue.front(); // gathering msg from front of msg_queue
    msg_queue.pop(); // removing front of msg_queue

    pthread_mutex_unlock(&msg_mutex);// unlocking mutex so msg_queue can be modified by other threads 

    return temp;// returning message from front of queue
}