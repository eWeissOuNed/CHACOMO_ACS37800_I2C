#pragma once
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ACS37800_I2C_ADDR_DEFAULT 0x3C

// Key runtime registers
#define ACS37800_REG_RMS      0x20  // VRMS/IRMS (u16, s16)
#define ACS37800_REG_VI_CODES 0x2A  // VCODES/ICODES (s16, s16)
#define ACS37800_REG_PINSTANT 0x2C  // PINSTANT (s16)

typedef struct {
    i2c_port_t port;
    uint8_t address;
    // Voltage divider
    float r_top_ohm;
    float r_bot_ohm;
    // Current range (30 or 90 A)
    float i_range_amps;
} acs37800_t;

esp_err_t acs37800_init_desc(acs37800_t *dev, i2c_port_t port, uint8_t address);
esp_err_t acs37800_set_voltage_divider(acs37800_t *dev, float r_top, float r_bottom);
static inline void acs37800_set_current_range(acs37800_t *dev, float range_amps) { if (dev) dev->i_range_amps = range_amps; }

// SparkFun-equivalent readers
esp_err_t acs37800_read_instantaneous(acs37800_t *dev, float *v, float *i, float *p);
esp_err_t acs37800_read_rms(acs37800_t *dev, float *v_rms, float *i_rms);

#ifdef __cplusplus
}
#endif
