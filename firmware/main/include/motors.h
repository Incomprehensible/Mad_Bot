#ifndef MOTORS_H_H
#define MOTORS_H_H

#include "hardware.h"
#include <esp_err.h>

class MotorController {
public:
    struct MotorConfig {
        int in1;
        int in2;
    };

    MotorController(const MotorConfig& mA,
                         const MotorConfig& mB,
                         const MotorConfig& mC,
                         int num_wheels = NUM_WHEELS,
                         float wheel_radius_m = WHEEL_RADIUS,
                         float gear_ratio = MOTOR_GEAR_RATIO,
                         float max_motor_rpm = MOTOR_MAX_RPM,
                         float min_motor_rpm = MOTOR_MIN_RPM);
   esp_err_t start();
   esp_err_t brake();
   esp_err_t setRPMs(const double* rpms);
   esp_err_t setWheelVelocities(const double* w);

private:
    void wheel_radps_to_motor_rpm(const double* omega_vec_rad_s, double D, double* motor_rpm);
    double rpm_to_pwm(double rpm);

    MotorConfig motors_[3];
    int num_wheels_;
    float r_;                 // wheel radius [m]
    float D_;                 // gear ratio: motor revs per wheel rev
    float max_motor_rpm_;     // max RPM for full PWM
    float min_motor_rpm_;     // min RPM for full PWM
    const float rpm_to_pwm_scale_ = 4096.0; // assuming 12-bit resolution
    float deadband_rpm_ = 5.0;         // rpm (or whatever unit) to ignore tiny noise
};

#endif