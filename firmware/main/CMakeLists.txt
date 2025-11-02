idf_component_register(SRCS "src/demo_commands.cpp" "src/demo_http_server.cpp" "src/main.cpp" "src/hardware.cpp" "src/motors.cpp"
                    INCLUDE_DIRS "include" "../components/pca9685" "../components/MPUdriver" "../components/micro_ros_espidf_component" 
                    PRIV_REQUIRES esp_driver_gpio MPUdriver pca9685 esp_event esp_netif nvs_flash esp_http_server micro_ros_espidf_component)

