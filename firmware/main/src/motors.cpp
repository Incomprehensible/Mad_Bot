#include "motors.h"
#include "sdkconfig.h"

#include <math.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "pca9685.h"

#include "hardware.h"
#include "pinout.h"

// TODO: check float to int16 conversion
// check how on/off values are set (I suspect that going lower increases speed?)

static char TAG[] = "MOTORS";

MotorController::MotorController(
                                const MotorConfig& mA,
                                const MotorConfig& mB,
                                const MotorConfig& mC,
                                int num_wheels,
                                float wheel_radius_m,
                                float gear_ratio,
                                float max_motor_rpm,
                                float min_motor_rpm)
    : num_wheels_(num_wheels),
      r_(wheel_radius_m),
      D_(gear_ratio),
      max_motor_rpm_(max_motor_rpm),
      min_motor_rpm_(min_motor_rpm)
{
    motors_[0] = mA;
    motors_[1] = mB;
    motors_[2] = mC;
}

esp_err_t MotorController::start()
{
    esp_err_t ret = 0;

    setPWM(M_SLEEP, 4096, 0); // wake up motor controllers
    if (ret == ESP_ERR_TIMEOUT)
    {
        printf("MotorController::start: I2C timeout\n");
    }
    return ret;
}

esp_err_t MotorController::brake()
   {
        esp_err_t ret = 0;

        for (int i = 0; i < num_wheels_; i++) {
            setPWM(motors_[i].in1, 4096, 0);
            setPWM(motors_[i].in2, 4096, 0);
        }
        if (ret == ESP_ERR_TIMEOUT)
        {
            printf("MotorController::brake: I2C timeout\n");
        }
        return ret;
   }

   esp_err_t MotorController::setRPMs(const double* rpms)
   {
        esp_err_t ret = 0;

        for (int i = 0; i < num_wheels_; i++) {
            float rpm = rpms[i];
            // Clamp RPM to min/max
            if (rpm > max_motor_rpm_) rpm = max_motor_rpm_;
            if (rpm < min_motor_rpm_) rpm = min_motor_rpm_;

            if (fabs(rpm) <= deadband_rpm_) {
                ESP_LOGI(TAG, "Motor %d: RPM: %.2f is within deadband", i, rpm);
                // Within deadband, stop motor
                setPWM(motors_[i].in1, 4096, 0);
                setPWM(motors_[i].in2, 4096, 0);
                // setPWM(M2_IN1, 4096, 0); example
                // setPWM(M2_IN2, 4096, 0);
                continue;
            }

            float pwm = rpm_to_pwm(rpm);
            // Test
            ESP_LOGI(TAG, "Motor %d: RPM: %.2f, PWM: %.2f", i, rpm, pwm);

            if (rpm >= 0) {
                // Forward
                setPWM(motors_[i].in1, 4096, 0);
                setPWM(motors_[i].in2, 0, (uint16_t)pwm);
                // setPWM(M3_IN1, 4096, 0); // forward mode example
                // setPWM(M3_IN2, 0, (int)(4096/speed_factor));
            } else {
                // Backward
                setPWM(motors_[i].in1, 0, (uint16_t)pwm);
                setPWM(motors_[i].in2, 4096, 0);
                // setPWM(M2_IN1, 0, (int)(4096/speed_factor)); // reverse example
                // setPWM(M2_IN2, 4096, 0); // reverse
            }
        }
        
        if(ret == ESP_ERR_TIMEOUT)
        {
            printf("MotorController::setRPMs: I2C timeout\n");
            brake();
        }
        return ret;
   }

esp_err_t MotorController::setWheelVelocities(const double* w)
{
    double rpms[NUM_WHEELS] = {0};
    wheel_radps_to_motor_rpm(w, D_, rpms);
    return setRPMs(rpms);
}

void MotorController::wheel_radps_to_motor_rpm(const double* omega_vec_rad_s, double D, double* motor_rpm)
{
    // omega_vec_rad_s: iterable of 3 wheel angular speeds in rad/s
    double factor = (60.0 / (2.0 * M_PI)) * D;
    for (int i = 0; i < 3; i++) {
        motor_rpm[i] = omega_vec_rad_s[i] * factor;
    }
}

double MotorController::rpm_to_pwm(double rpm)
{
    // Convert RPM to PWM value
    double pwm = (fabs(rpm) / max_motor_rpm_) * rpm_to_pwm_scale_;
    return pwm;
}

