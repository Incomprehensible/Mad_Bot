#ifndef DEMO_H
#define DEMO_H

typedef enum Command {
    FORWARD,
    BACKWARD,
    ROTATE,
    BREAK
} Command; 

esp_err_t start_http_demo_server();
void task_Demo(Command);

#endif