#ifndef __SHT31_H__
#define __SHT31_H__

#include <esp_err.h>
#include "driver/i2c.h"

// Device addresses
#define SHT31_ADDR_44 0x44  // Address when ADDR pin is low
#define SHT31_ADDR_45 0x45  // Address when ADDR pin is high

// Commands
#define SHT31_CMD_SOFT_RESET     0x30A2
#define SHT31_CMD_CLEAR_STATUS   0x3041
#define SHT31_CMD_READ_STATUS    0xF32D
#define SHT31_CMD_HEATER_ENABLE  0x306D
#define SHT31_CMD_HEATER_DISABLE 0x3066

// Measurement commands
#define SHT31_CMD_MEASURE_HIGH_REP  0x2400
#define SHT31_CMD_MEASURE_MED_REP   0x240B
#define SHT31_CMD_MEASURE_LOW_REP   0x2416

// Repeatability modes
typedef enum {
    SHT31_REPEATABILITY_LOW    = 0,
    SHT31_REPEATABILITY_MEDIUM = 1,
    SHT31_REPEATABILITY_HIGH   = 2
} sht31_repeatability_t;

// Device descriptor
typedef struct {
    i2c_port_t i2c_port;        // I2C port number
    uint8_t i2c_addr;           // I2C address
    sht31_repeatability_t repeatability;  // Measurement repeatability
    bool initialized;           // Initialization status
} sht31_t;

// Function prototypes
esp_err_t sht31_init(sht31_t *dev);
esp_err_t sht31_init_desc(sht31_t *dev, i2c_port_t i2c_port, uint8_t addr);
esp_err_t sht31_read_temperature_humidity(sht31_t *dev, float *temperature, float *humidity);
esp_err_t sht31_read_status(sht31_t *dev, uint16_t *status);
esp_err_t sht31_clear_status(sht31_t *dev);
esp_err_t sht31_heater_enable(sht31_t *dev, bool enable);
esp_err_t sht31_soft_reset(sht31_t *dev);
esp_err_t sht31_deinit(sht31_t *dev);

#endif // __SHT31_H__