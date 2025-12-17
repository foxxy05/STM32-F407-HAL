// ================= WORKING CODE =================
#include <Arduino.h>
#include <Wire.h>
#include <string.h>

/* ============== I2C CONFIGURATION ============== */
#define I2C_SLAVE_ADDR    0x55
#define I2C_SDA_PIN       21
#define I2C_SCL_PIN       22
#define I2C_FREQUENCY     100000

/* ============== BUFFERS ============== */
uint8_t rx_buffer[256];
uint8_t tx_buffer[256] = "Hello_from_ESP32";
volatile int rx_len = 0;

void onRequest();



/* ============== SETUP ============== */
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n=== ESP32 I2C Slave (Arduino) ===");
    Serial.printf("Address: 0x%02X\n", I2C_SLAVE_ADDR);
    Serial.printf("SDA: GPIO%d, SCL: GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.printf("Frequency: %d Hz\n\n", I2C_FREQUENCY);
    
    // Initialize I2C Slave
    Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
    
    // Register callbacks
    Wire.onRequest(onRequest);
    
    Serial.println("I2C Slave Ready\n");
}

/* ============== MAIN LOOP ============== */
void loop() {
    // Display received data
    // if (rx_len > 0) {
    //     Serial.print("Received: ");
    //     for (int i = 0; i < rx_len; i++) {
    //         Serial.printf("0x%02X ", rx_buffer[i]);
    //     }
    //     Serial.println();
    //     rx_len = 0;
    // }
    
    delay(10);
}


/* ============== REQUEST CALLBACK (Master requests) ============== */
void onRequest() {
    Serial.println("<< Sending response");
    Wire.write((uint8_t*)tx_buffer, strlen((char*)tx_buffer));
}