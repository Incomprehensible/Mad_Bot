#ifndef HARDWARE_H
#define HARDWARE_H

#define I2C_MASTER_SCL_IO   16    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   15    /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 400000 //100000     /*!< I2C master clock frequency */
#define I2C_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

#define SPI_CLOCK 1000000  // up to 1MHz for all registers, and 20MHz for sensor data registers only
#define SPI_MODE 3 // CPOL = 1, CPHA = 1

// Encoder provides a resolution of 12 counts per revolution of the motor shaft
// To compute the counts per revolution of the gearbox output shaft, multiply the gear ratio by 12
#define PCNT_HIGH_LIMIT (12 * 10)
#define PCNT_LOW_LIMIT  (-12 * 10)

void hardware_init();


#endif