#include "acs37800.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>


// Constants
#define ADC_FS_VOLTS   0.25f
#define CODES_FS_VI    27500.0f
#define CODES_FS_RMS   55000.0f
#define LSB_PER_mW_30A 3.08f


static esp_err_t readN(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(port, addr, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}
static esp_err_t read16(i2c_port_t port, uint8_t addr, uint8_t reg, uint16_t *val) {
    uint8_t b[2]; esp_err_t e = readN(port, addr, reg, b, 2); if (e!=ESP_OK) return e;
    *val = ((uint16_t)b[0] << 8) | b[1]; return ESP_OK;
}

esp_err_t acs37800_init_desc(acs37800_t *dev, i2c_port_t port, uint8_t address) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->port = port;
    dev->address = address;
    dev->r_top_ohm = 178000.0f;
    dev->r_bot_ohm = 2000.0f;
    dev->i_range_amps = 30.0f;
    return ESP_OK;
}
esp_err_t acs37800_set_voltage_divider(acs37800_t *dev, float r_top, float r_bottom) {
    if (!dev || r_top<=0 || r_bottom<=0) return ESP_ERR_INVALID_ARG;
    dev->r_top_ohm = r_top; dev->r_bot_ohm = r_bottom; return ESP_OK;
}


esp_err_t acs37800_read_instantaneous(acs37800_t *dev, float *v, float *i, float *p) {
    if (!dev || !v || !i || !p) return ESP_ERR_INVALID_ARG;

    // --- Read VCODES / ICODES from 0x2A (4 bytes) ---
    uint8_t b[4];
    esp_err_t e = readN(dev->port, dev->address, ACS37800_REG_VI_CODES, b, 4);
    if (e != ESP_OK) return e;

    // Little-endian decode (LSB first in the register window)
    int16_t vcodes = (int16_t)((b[1] << 8) | b[0]);
    int16_t icodes = (int16_t)((b[3] << 8) | b[2]);

    // Divider correction back to line voltage: (Rtop + Rbot) / Rbot
    float mult  = (dev->r_bot_ohm > 0.0f) ? ((dev->r_top_ohm + dev->r_bot_ohm) / dev->r_bot_ohm) : 1.0f;
    float range = (dev->i_range_amps > 0.0f) ? dev->i_range_amps : 30.0f;

    // Convert to engineering units (SparkFun method)
    float v_adc = ((float)vcodes) / CODES_FS_VI * ADC_FS_VOLTS; // volts at ADC pin
    *v = v_adc * mult;                                          // line volts
    *i = ((float)icodes) / CODES_FS_VI * range;                 // amps

    // --- Read instantaneous power from 0x2C (2 bytes) ---
    uint8_t p2[2];
    e = readN(dev->port, dev->address, ACS37800_REG_PINSTANT, p2, 2);
    if (e != ESP_OK) return e;

    // Little-endian power code
    int16_t pcodes = (int16_t)((p2[1] << 8) | p2[0]);

    // Convert codes -> mW -> correct divider -> W
    float LSBpermW = LSB_PER_mW_30A * (30.0f / range);
    *p = ((float)pcodes) / LSBpermW * mult / 1000.0f;

    return ESP_OK;
}



esp_err_t acs37800_read_rms(acs37800_t *dev, float *v_rms, float *i_rms) {
    if (!dev || !v_rms || !i_rms) return ESP_ERR_INVALID_ARG;

    uint8_t b[4];
    esp_err_t e = readN(dev->port, dev->address, ACS37800_REG_RMS, b, 4);
    if (e != ESP_OK) return e;

    // Little-endian: VRMS (u16), IRMS (s16)
    uint16_t vrms_u16 = (uint16_t)((b[1] << 8) | b[0]);
    int16_t  irms_s16 = (int16_t) ((b[3] << 8) | b[2]);

    float mult  = (dev->r_bot_ohm > 0.0f) ? ((dev->r_top_ohm + dev->r_bot_ohm) / dev->r_bot_ohm) : 1.0f;
    float range = (dev->i_range_amps > 0.0f) ? dev->i_range_amps : 30.0f;

    *v_rms = ((float)vrms_u16) / CODES_FS_RMS * ADC_FS_VOLTS * mult;
    *i_rms = ((float)irms_s16) / CODES_FS_RMS * range;
    return ESP_OK;
}
