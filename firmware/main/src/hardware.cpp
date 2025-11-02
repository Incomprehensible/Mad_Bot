#include "sdkconfig.h"

#include <math.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/i2c.h>
#include <driver/pulse_cnt.h>
#include <driver/gpio.h>

#include "pca9685.h"
#include "MPU.hpp"        // main file, provides the class itself
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "SPIbus.hpp"
// #include "vl53l5cx_api.h"
#include "hardware.h"
#include "pinout.h"
#include "motors.h"

#include "vl53l7cx_api.h" 

static char TAG[] = "HARDWARE";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

// Globals
i2c_port_t i2c_master_port = I2C_MASTER_NUM;
pcnt_unit_handle_t pcnt_unit1 = NULL;
pcnt_unit_handle_t pcnt_unit2 = NULL;
pcnt_unit_handle_t pcnt_unit3 = NULL;
MPU_t MPU;  // create a default MPU object
spi_device_handle_t mpu_spi_handle;

MotorController::MotorConfig mA = {
    .in1 = M1_IN1,
    .in2 = M1_IN2,
};
MotorController::MotorConfig mB = {
    .in1 = M2_IN1,
    .in2 = M2_IN2,
};
MotorController::MotorConfig mC = {
    .in1 = M3_IN1,
    .in2 = M3_IN2,
};
MotorController motors = MotorController(mA, mB, mC);

static void i2c_example_master_init(void);

int example1(void)
{

	/*********************************/
	/*   VL53L7CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53L7CX_Configuration 	Dev;			/* Sensor configuration */
	VL53L7CX_ResultsData 	Results;		/* Results data from VL53L7CX */


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = VL53L7CX_DEFAULT_I2C_ADDRESS;
    Dev.platform.port = I2C_MASTER_NUM;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	//Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	* from the default one (filled with 0x20 for this example).
	*/
	//status = vl53l7cx_set_i2c_address(&Dev, 0x20);


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L7CX sensor connected */
	status = vl53l7cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L7CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L7CX sensor */
	status = vl53l7cx_init(&Dev);
	if(status)
	{
		printf("VL53L7CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L7CX ULD ready ! (Version : %s)\n",
			VL53L7CX_API_REVISION);


    status = vl53l7cx_set_resolution(&Dev, VL53L7CX_RESOLUTION_8X8);        

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l7cx_start_ranging(&Dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */
 
		status = vl53l7cx_check_data_ready(&Dev, &isReady);

		if(isReady)
		{
			vl53l7cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total 
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&(Dev.platform), 5);
	}

	status = vl53l7cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}

// TODO: one callback for all units?
static bool pcnt_on_reach1(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

static bool pcnt_on_reach2(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

static bool pcnt_on_reach3(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void pcnt_init_M1()
{
    // Motor 1 encoder
    ESP_LOGI(TAG, "install PCNT1 unit");
    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
    };
    // pcnt_unit_config_t unit_config { .high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT };  
    // pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit1));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit1, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC_A1,
        .level_gpio_num = ENC_B1,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit1, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC_B1,
        .level_gpio_num = ENC_A1,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit1, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for PCNT1 channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {PCNT_LOW_LIMIT, PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit1, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach1,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit1, &cbs, queue));

    ESP_LOGI(TAG, "enable PCNT1 unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit1));
    ESP_LOGI(TAG, "clear PCNT1 unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit1));
    ESP_LOGI(TAG, "start PCNT1 unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit1));

#ifdef CONFIG_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(ENC_A1, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // // Report counter value
    // int pulse_count = 0;
    // int event_count = 0;
    // while (1) {
    //     if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(100)))
    //         ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
    //     else {
    //         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit1, &pulse_count));
    //         ESP_LOGI(TAG, "Pulse count for M1: %d", pulse_count);
    //         vTaskDelay(100 / portTICK_PERIOD_MS);
    //     }
    // }
}

void pcnt_init_M2()
{
    // Motor 2 encoder
    ESP_LOGI(TAG, "install PCNT2 unit");
    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
    };
    // pcnt_unit_config_t unit_config { .high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT };  
    // pcnt_unit_handle_t pcnt_unit2 = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit2));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit2, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC_A2,
        .level_gpio_num = ENC_B2,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit2, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC_B2,
        .level_gpio_num = ENC_A2,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit2, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for PCNT2 channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    // int watch_points[] = {PCNT_LOW_LIMIT, PCNT_LOW_LIMIT/2, 0, PCNT_HIGH_LIMIT/2, PCNT_HIGH_LIMIT};
    int watch_points[] = {PCNT_LOW_LIMIT, PCNT_HIGH_LIMIT}; // detect full rotations
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit2, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach2,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit2, &cbs, queue));

    ESP_LOGI(TAG, "enable PCNT2 unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit2));
    ESP_LOGI(TAG, "clear PCNT2 unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit2));
    ESP_LOGI(TAG, "start PCNT2 unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit2));

#ifdef CONFIG_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(ENC_A2, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // // Report counter value
    // int pulse_count = 0;
    // int event_count = 0;
    // while (1) {
    //     if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(100)))
    //         ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
    //     else {
    //         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit2, &pulse_count));
    //         ESP_LOGI(TAG, "Pulse count of M2: %d", pulse_count);
    //         vTaskDelay(100 / portTICK_PERIOD_MS);
    //     }
    // }
}

void pcnt_init_M3()
{
    // Motor 3 encoder
    ESP_LOGI(TAG, "install PCNT3 unit");
    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
    };
    // pcnt_unit_config_t unit_config { .high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT };  
    // pcnt_unit_handle_t pcnt_unit3 = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit3));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit3, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC_A3,
        .level_gpio_num = ENC_B3,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit3, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC_B3,
        .level_gpio_num = ENC_A3,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit3, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for PCNT3 channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {PCNT_LOW_LIMIT, PCNT_LOW_LIMIT/2, 0, PCNT_HIGH_LIMIT/2, PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit3, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach3,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit3, &cbs, queue));
    ESP_LOGI(TAG, "enable PCNT3 unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit3));

    ESP_LOGI(TAG, "clear PCNT3 unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit3));
    ESP_LOGI(TAG, "start PCNT3 unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit3));
#ifdef CONFIG_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(ENC_A3, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif
    // // Report counter value
    // int pulse_count = 0;
    // int event_count = 0;
    // while (1) {
    //     if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(100)))
    //         ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
    //     else {
    //         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit3, &pulse_count));
    //         ESP_LOGI(TAG, "Pulse count of M3: %d", pulse_count);
    //         vTaskDelay(100 / portTICK_PERIOD_MS);
    //     }
    // }
}

void pcnt_init()
{
    pcnt_init_M1();
    pcnt_init_M2();
    pcnt_init_M3();
}

void spi_init()
{
    esp_err_t ret = 0;

    ret = hspi.begin(SPI_MOSI, SPI_MISO, SPI_CLK);  // initialize the SPI bus
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return;
    }

    hspi.addDevice(SPI_MODE, SPI_CLOCK, SPI_CS, &mpu_spi_handle);
    ESP_LOGI(TAG, "SPI bus initialized");

    // MPU_t MPU;  // create a default MPU object
    MPU.setBus(hspi);  // set bus port, not really needed here since default is HSPI
    MPU.setAddr(mpu_spi_handle);  // set spi_device_handle, always needed!

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());  // initialize the chip and set initial configurations

    // Setup with your configurations
    // ESP_ERROR_CHECK(MPU.setSampleRate(50));  // set sample rate to 50 Hz
    // ESP_ERROR_CHECK(MPU.setGyroFullScale(mpud::GYRO_FS_500DPS));
    // ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));

    // Reading sensor data
    printf("Reading sensor data:\n");
    mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
    mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
    mpud::float_axes_t accelG;   // accel axes in (g) gravity format
    mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    // Read
    MPU.acceleration(&accelRaw);  // fetch raw data from the registers
    MPU.rotation(&gyroRaw);       // fetch raw data from the registers
    // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
    // Convert
    accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
    gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
    // Debug
    printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
    printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(TAG, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    // i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_master_port = I2C_MASTER_NUM;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    // i2c_set_timeout(I2C_NUM_1, I2C_TIMEOUT);
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE, 0));
}

// void vl53l7cx_init()
// {
//     // Temporarily using VL53L5CX API
//     uint8_t 				status, loop, isAlive, isReady, i;
//     VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
//     VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

//     Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
//     Dev.platform.port = i2c_master_port;
    
//     status = vl53l5cx_is_alive(&Dev, &isAlive);
//     if(!isAlive || status)
//     {
//         printf("VL53L5CX not detected at requested address\n");
//         return;
//     }
//     else
//     {
//         printf("VL53L5CX detected at requested address\n");
//     }

//     /* (Mandatory) Init VL53L5CX sensor */
//     status = vl53l5cx_init(&Dev);
//     if(status)
//     {
//         printf("VL53L5CX ULD Loading failed\n");
//         return;
//     }
//     else
//     {
//         printf("VL53L5CX ULD Loading succeeded\n");
//     }

//     vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
//     printf("VL53L5CX set to 8x8 resolution\n");
//       // Using 8x8, min frequency is 1Hz and max is 15Hz
//     vl53l5cx_set_ranging_frequency_hz(&Dev, 15);
//     printf("VL53L5CX ranging frequency set to 15Hz\n");

//     printf("VL53L5CX ULD ready ! (Version : %s)\n",
//            VL53L5CX_API_REVISION);
//     status = vl53l5cx_start_ranging(&Dev);

//     // TEST
//     loop = 0;
//     while (loop < 3)
//     {
//         /* Use polling function to know when a new measurement is ready.
//          * Another way can be to wait for HW interrupt raised on PIN A1
//          * (INT) when a new measurement is ready */

//         status = vl53l5cx_check_data_ready(&Dev, &isReady);

//         if (isReady)
//         {
//             vl53l5cx_get_ranging_data(&Dev, &Results);

//             /* As the sensor is set in 4x4 mode by default, we have a total
//              * of 16 zones to print. For this example, only the data of first zone are
//              * print */
//             printf("Print data no : %3u\n", Dev.streamcount);
//             for(i = 0; i < 64; i++)
//             {
//                 printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
//                        i,
//                        Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
//                        Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
//             }
//             printf("\n");
//             loop++;
//         }

//         /* Wait a few ms to avoid too high polling (function in platform
//          * file, not in API) */
//         WaitMs(&(Dev.platform), 5);
//     }

//     // VISUALIZE TEST
//     // Create a buffer to hold the CSV string
//     // char output_line[512] = {};  // Large enough for 16 distances + commas
//     // int pos = 0;
//     // int IMAGE_WIDTH = 8; // default 4x4 grid
//     // while (true) {
//     //     status = vl53l5cx_check_data_ready(&Dev, &isReady);
        
//     //     if (isReady)
//     //     {
//     //         vl53l5cx_get_ranging_data(&Dev, &Results);

//     //         pos = 0; // Reset position for each line
//     //         // Mimic Arduino's output format (row-by-row, bottom to top, left to right)
//     //         for (int y = 0; y <= IMAGE_WIDTH * (IMAGE_WIDTH - 1); y += IMAGE_WIDTH)
//     //         {
//     //             for (int x = IMAGE_WIDTH - 1; x >= 0; x--)
//     //             {
//     //                 int index = x + y;
//     //                 uint16_t dist = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * index];
//     //                 pos += sprintf(output_line + pos, "%d,", dist);
//     //             }
//     //         }
            
//     //         output_line[pos - 1] = '\n'; // Replace last comma with newline
//     //         output_line[pos] = '\0';

//     //         // Send over serial
//     //         printf("%s", output_line);
//     //     }

//     //     WaitMs(&(Dev.platform), 5);
//     // }

//     status = vl53l5cx_stop_ranging(&Dev);
// }

// DEBUG
void i2c_scanner() {
    printf("Scanning I2C bus...\n");
    // Iterate over all possible I2C addresses
    for (int addr = 1; addr < 127; addr++) {
        // Create I2C command link
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Send I2C address with write bit
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        // Execute I2C command and check for response
        esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        // If device responds, print the address
        if (ret == ESP_OK) {
            printf("Found device at address 0x%02x\n", addr);
        }
    }
    printf("I2C scan complete.\n");
}

void hardware_init()
{
    esp_err_t ret = 0;

//     gpio_config_t usb_phy_conf = {
//     .pin_bit_mask = (1ULL << SPI_CS),
//     .mode = GPIO_MODE_OUTPUT,
//     .pull_up_en = (gpio_pullup_t)0,
//     .pull_down_en = (gpio_pulldown_t)0,
//     .intr_type = GPIO_INTR_DISABLE,
// };
//     gpio_config(&usb_phy_conf);
    gpio_reset_pin(GPIO_NUM_44);
    gpio_set_direction(GPIO_NUM_44,GPIO_MODE_OUTPUT);

    i2c_example_master_init();
    example1();

    set_pca9685_adress(I2C_ADDRESS);
    resetPCA9685();
    setFrequencyPCA9685(1000);  // 1000 Hz
    turnAllOff();

    spi_init();
    i2c_scanner();
    // vl53l7cx_init();
    pcnt_init();

    ESP_ERROR_CHECK(motors.start());

    printf("Finished hardware setup.\n");
}