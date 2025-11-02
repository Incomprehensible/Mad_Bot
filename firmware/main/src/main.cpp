#include <stdio.h>
// #include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <freertos/event_groups.h>

// #include <protocol_examples_common.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "MPU.hpp"        // main file, provides the class itself
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions

#include "hardware.h"
#include "demo.h"
#include "motors.h"

static const char *TAG = "MadBot";

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t wheels_rpm_subscriber;
rcl_subscription_t motion_cmd_subscriber;
std_msgs__msg__Float64MultiArray wheel_w_msg;
std_msgs__msg__Bool cmd_msg;
sensor_msgs__msg__Imu msg;

extern MPU_t MPU;
mpud::float_axes_t accelG;   // accel axes in (g) gravity format
mpud::float_axes_t gyroDPS;    // gyro axes in (dps) degrees per second format
extern MotorController motors;
volatile bool motion_enabled = false;

double wheel_w_buffer[NUM_WHEELS] = {0};

void IMU_task(void* arg)
{
    mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
	mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
	int16_t temp = 0;
	float tempC = 0.0f;

    while (1) {
        // Read
        MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
		temp = MPU.temperature(&temp);
        // Convert
        accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
		gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
		tempC = mpud::tempCelsius(temp);
        // Debug
		// printf("WHO_AM_I = 0x%02X\n", MPU.whoAmI());
        // printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t gyro: [%+7.2f %+7.2f %+7.2f ] (DPS) \t temp: %+6.2f (C)\n", accelG.x, accelG.y, accelG.z, gyroDPS[0], gyroDPS[1], gyroDPS[2], tempC);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void motion_command_callback(const void * msgin)
{
	const std_msgs__msg__Bool* cmd = (const std_msgs__msg__Bool *)msgin;

	if (motion_enabled == cmd->data) // No change
        return;

    if (!cmd->data) {
        ESP_ERROR_CHECK(motors.brake());
    }
    motion_enabled = cmd->data;
}

void wheels_rpm_callback(const void * msgin)
{
	const std_msgs__msg__Float64MultiArray* cmd = (const std_msgs__msg__Float64MultiArray *)msgin;

    if (!motion_enabled) {// No change 
    	return;
	}

	for (size_t i = 0; i < NUM_WHEELS; i++) {
		printf("Wheel %d cmd: %f rad/s\n", (int)i, wheel_w_buffer[i]);
		// wheel_w_buffer[i] = cmd->data.data[i];
	}
	ESP_ERROR_CHECK(motors.setWheelVelocities(wheel_w_buffer));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
        msg.linear_acceleration.x = accelG.x;
        msg.linear_acceleration.y = accelG.y;
        msg.linear_acceleration.z = accelG.z;
		msg.angular_velocity.x = gyroDPS.x;
		msg.angular_velocity.y = gyroDPS.y;
		msg.angular_velocity.z = gyroDPS.z;

		// printf("Publishing: %d\n", (int) msg_test.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_publisher", "", &support));
	
	// create publisher
	// RCCHECK(rclc_publisher_init_best_effort(
	// 	&publisher_test,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	// 	"/microROS/int32_publisher"));

    // create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
    	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/microROS/IMU_publisher"));

	 // Create subscribers.
	RCCHECK(rclc_subscription_init_default(
		&wheels_rpm_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
		"/wheels_controller/commands"));
	RCCHECK(rclc_subscription_init_default(
		&motion_cmd_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"/motion_command"));


	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 2000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheels_rpm_subscriber, &wheel_w_msg, &wheels_rpm_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &motion_cmd_subscriber, &cmd_msg, &motion_command_callback, ON_NEW_DATA));

    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 50;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;

	// OPTIONALLY this struct can store rules for specific members
	// !! Using the API with rules will use dynamic memory allocations for handling strings !!

	micro_ros_utilities_memory_rule_t rules[] = {
		{"header.frame_id", 30},
	};
	conf.rules = rules;
	conf.n_rules = sizeof(rules) / sizeof(rules[0]);
    size_t dynamic_size = micro_ros_utilities_get_dynamic_size(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		conf
	);
	// The total (stack, static & dynamic) memory usage of a packet will be:
	size_t message_total_size = dynamic_size + sizeof(sensor_msgs__msg__Imu);
    // The message dynamic memory can be allocated using the following call.
	// This will use rcutils default allocators for getting memory.
	bool success = micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		&msg,
		conf
	);
    msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "madbot");

	msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;
    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 0.0;

	wheel_w_msg.data.size = NUM_WHEELS;
	wheel_w_msg.data.capacity = NUM_WHEELS;
	wheel_w_msg.data.data = wheel_w_buffer;

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_subscription_fini(&wheels_rpm_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&motion_cmd_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

// void synch_time_with_agent()
// {
//     // Sync timeout
//     const int timeout_ms = 5000;

//     // Synchronize time with the agent
//     RCCHECK(rmw_uros_sync_session(timeout_ms));

//     if (rmw_uros_epoch_synchronized())
//     {
//         // Get time in milliseconds or nanoseconds
//         //base_timestamp_ms = rmw_uros_epoch_millis();
//         base_timestamp_ns = rmw_uros_epoch_nanos();
//         base_timestamp_s = base_timestamp_ns / 1000000000;
//         base_timestamp_ns = base_timestamp_ns % 1000000000; // convert into fractional part
//     }
//     baseline_ticks = xTaskGetTickCount();
//     printf("Baseline ticks: %lld\n", baseline_ticks);
//     printf("Base timestamp (s): %lld\n", base_timestamp_s);
//     printf("Base timestamp (fraction) (ns): %lld\n", base_timestamp_ns);
// }


extern "C" void app_main() {
    // hardware initialization

	// ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

	// ESP_ERROR_CHECK(example_connect()); //old Wi-Fi connection

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    hardware_init();

	// task_Demo(Command::FORWARD);
	// vTaskDelay(1000 / portTICK_PERIOD_MS);
	// task_Demo(Command::BACKWARD);
	// vTaskDelay(1000 / portTICK_PERIOD_MS);
	// task_Demo(Command::ROTATE);
	// vTaskDelay(1000 / portTICK_PERIOD_MS);

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(IMU_task,
            "IMU_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
    
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    esp_log_level_set("wifi", ESP_LOG_NONE);
	// ESP_ERROR_CHECK(start_http_demo_server());

	// synch_time_with_agent();
}
