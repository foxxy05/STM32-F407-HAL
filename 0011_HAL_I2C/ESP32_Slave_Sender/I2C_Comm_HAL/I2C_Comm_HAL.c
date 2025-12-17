================= WORKING CODE =================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"

/* ============== I2C CONFIGURATION ============== */
#define I2C_SLAVE_NUM           I2C_NUM_0
#define I2C_SLAVE_SDA_IO        21
#define I2C_SLAVE_SCL_IO        22
#define I2C_SLAVE_ADDR          0x55
#define I2C_SLAVE_TX_BUF_LEN    256
#define I2C_SLAVE_RX_BUF_LEN    256

static const char *TAG = "I2C_SLAVE";

/* ============== I2C SLAVE INITIALIZATION ============== */
static void i2c_slave_init(void) {
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };
    
    i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    
    i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode,
                       I2C_SLAVE_RX_BUF_LEN,
                       I2C_SLAVE_TX_BUF_LEN, 0);
    
    ESP_LOGI(TAG, "I2C Slave Initialized at address: 0x%02X", I2C_SLAVE_ADDR);
}

/* ============== SLAVE RX/TX TASK ============== */
static void i2c_slave_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(I2C_SLAVE_RX_BUF_LEN);
    uint8_t tx_response[] = "Hello_from_ESP32";
    
    while (1) {
        // Read data from master
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, I2C_SLAVE_RX_BUF_LEN, 1000 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes from master", len);
            for (int i = 0; i < len; i++) {
                printf("0x%02X ", data[i]);
            }
            printf("\n");
        }
        
        // Write response to master
        i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_response, strlen((char *)tx_response), 500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Response sent to master");
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    free(data);
    vTaskDelete(NULL);
}

/* ============== MAIN APPLICATION ============== */
void app_main(void) {
    ESP_LOGI(TAG, "ESP32 I2C Slave Application Started");
    
    // Initialize I2C Slave
    i2c_slave_init();
    
    // Create slave task
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Waiting for master requests...");
}
