#include "sdkconfig.h"

#include <math.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "pca9685.h"

#include "pinout.h"
#include "demo.h"

static char TAG[] = "DEMO";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

volatile Command mode = BREAK;

// can set the pin to be fully on with
// pwm.setPWM(pin, 4096, 0);
// You can set the pin to be fully off with
// pwm.setPWM(pin, 0, 4096);
void task_forward(void *ignore)
{
    int speed_factor = pow(2,4);
    esp_err_t ret = 0;

    setPWM(M_SLEEP, 4096, 0); // wake up motor controllers

    printf("Test M2 forward mode...\n");
    printf("Test M3 forward mode...\n");

    setPWM(M2_IN1, 0, (int)(4096/speed_factor)); // reverse
    setPWM(M2_IN2, 4096, 0); // reverse

    setPWM(M3_IN1, 4096, 0); // forward mode
    setPWM(M3_IN2, 0, (int)(4096/speed_factor)); // forward mode

    if(ret == ESP_ERR_TIMEOUT)
    {
        printf("I2C timeout\n");
    }
    else if(ret == ESP_OK)
    {
        // all good
        printf("test Ok\n");
    }
    else
    {
        printf("No ack, sensor not connected...skip...\n");
    }

    vTaskDelay(500/portTICK_PERIOD_MS);

    setPWM(M2_IN1, 4096, 0); // break
    setPWM(M3_IN2, 4096, 0); // break

    printf("Test M2 breaking...\n");
    printf("Test M3 breaking...\n");

    mode = FORWARD;

    vTaskDelete(NULL);
}

void task_backward(void *ignore)
{
    int speed_factor = pow(2,4);
    esp_err_t ret = 0;

    setPWM(M_SLEEP, 4096, 0); // wake up motor controllers

    printf("Test M2 forward mode...\n");
    printf("Test M3 forward mode...\n");

    setPWM(M2_IN1, 4096, 0); // forward mode
    setPWM(M2_IN2, 0, (int)(4096/speed_factor)); // forward mode

    setPWM(M3_IN1, 0, (int)(4096/speed_factor)); // reverse
    setPWM(M3_IN2, 4096, 0); // reverse


    if(ret == ESP_ERR_TIMEOUT)
    {
        printf("I2C timeout\n");
    }
    else if(ret == ESP_OK)
    {
        // all good
        printf("test Ok\n");
    }
    else
    {
        printf("No ack, sensor not connected...skip...\n");
    }

    vTaskDelay(500/portTICK_PERIOD_MS);

    setPWM(M2_IN2, 4096, 0); // break
    setPWM(M3_IN1, 4096, 0); // break

    printf("Test M2 breaking...\n");
    printf("Test M3 breaking...\n");

    mode = BACKWARD;

    vTaskDelete(NULL);
}

void task_rotate(void *ignore)
{
    esp_err_t ret = 0;
    int speed_factor = pow(2,4);

    setPWM(M_SLEEP, 4096, 0); // wake up motor controllers

    printf("Test M1 forward mode...\n");
    printf("Test M2 forward mode...\n");
    printf("Test M3 forward mode...\n");

    setPWM(M1_IN1, 4096, 0); // forward mode
    setPWM(M1_IN2, 0, (int)(4096/speed_factor)); // forward mode

    setPWM(M2_IN1, 4096, 0); // forward mode
    setPWM(M2_IN2, 0, (int)(4096/speed_factor)); // forward mode

    setPWM(M3_IN1, 4096, 0); // forward mode
    setPWM(M3_IN2, 0, (int)(4096/speed_factor)); // forward mode

    if(ret == ESP_ERR_TIMEOUT)
    {
        printf("I2C timeout\n");
    }
    else if(ret == ESP_OK)
    {
        // all good
        printf("test Ok\n");
    }
    else
    {
        printf("No ack, sensor not connected...skip...\n");
    }

    // vTaskDelay(500/portTICK_PERIOD_MS);

    // setPWM(M1_IN2, 4096, 0); // break
    // setPWM(M2_IN2, 4096, 0); // break
    // setPWM(M3_IN2, 4096, 0); // break

    // printf("Test M1 breaking...\n");
    // printf("Test M2 breaking...\n");
    // printf("Test M3 breaking...\n");

    mode = ROTATE;

    vTaskDelete(NULL);
}

void task_break(void *ignore)
{
    if (mode != BREAK)
    {
        esp_err_t ret = 0;

        setPWM(M_SLEEP, 4096, 0); // wake up motor controllers

        if (mode == FORWARD)
        {
            setPWM(M2_IN1, 4096, 0); // break
            setPWM(M3_IN2, 4096, 0); // break
        }
        else if (mode == BACKWARD)
        {
            setPWM(M2_IN2, 4096, 0); // break
            setPWM(M3_IN1, 4096, 0); // break
        }
        else if (mode == ROTATE)
        {
            setPWM(M1_IN2, 4096, 0); // break
            setPWM(M2_IN2, 4096, 0); // break
            setPWM(M3_IN2, 4096, 0); // break
        }

        printf("Test M1 breaking...\n");
        printf("Test M2 breaking...\n");
        printf("Test M3 breaking...\n");
    }
    mode = BREAK;

    vTaskDelete(NULL);
}

void task_Demo(Command cmd)
{
    printf("Executing on core %d\n", xPortGetCoreID());
    printf("Command chosen: %d", cmd);

    esp_err_t ret = 0;

    switch (cmd) {
        case (FORWARD):
            xTaskCreate(task_forward, "task_FORWARD", 1024 * 2, (void* ) 0, 10, NULL);
            break;
        case (BACKWARD):
            xTaskCreate(task_backward, "task_BACKWARD", 1024 * 2, (void* ) 0, 10, NULL);
            break;
        case (ROTATE):
            xTaskCreate(task_rotate, "task_ROTATE", 1024 * 2, (void* ) 0, 10, NULL);
            break;
        case (BREAK):
            xTaskCreate(task_break, "task_BREAK", 1024 * 2, (void* ) 0, 10, NULL);
            break;
        default:
            printf("Invalid command\n");
            break;
    }
}
