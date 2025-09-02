#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "acs37800.h"

#define I2C_MASTER_SCL_IO 15
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

static const char *TAG = "ACS_EXAMPLE";

#ifdef __cplusplus
extern "C" {
#endif
void app_main(void);
#ifdef __cplusplus
}
#endif

void app_main(void) {
    ESP_LOGI(TAG, "Starting ACS37800 example...");

    // Old I2C driver (stable): configure + install
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    acs37800_t sensor;
    ESP_ERROR_CHECK(acs37800_init_desc(&sensor, I2C_MASTER_NUM, 0x3C)); // 0x3C = 60
    acs37800_set_current_range(&sensor, 30.0f);                         // 30A variant
    ESP_ERROR_CHECK(acs37800_set_voltage_divider(&sensor, 180000.0f, 1500.0f)); // Rtop, Rbottom

    while (1) {
        float v,i,p, vr, ir;
        esp_err_t e1 = acs37800_read_instantaneous(&sensor, &v, &i, &p);
        esp_err_t e2 = acs37800_read_rms(&sensor, &vr, &ir);
        if (e1 == ESP_OK) printf("Inst: V=%.3f V  I=%.3f A  P=%.3f W\n", v, i, p);
        else ESP_LOGE(TAG, "Instantaneous read error: %s", esp_err_to_name(e1));
        if (e2 == ESP_OK) printf("RMS:  V=%.3f V  I=%.3f A\n", vr, ir);
        else ESP_LOGE(TAG, "RMS read error: %s", esp_err_to_name(e2));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
