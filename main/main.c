#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bme680.h>
#include <string.h>

#include <bh1750_i2c_hal.h>
#include <bh1750_i2c.h>

#define BME680_I2C_ADDR 0x77
#define PORT 0
#define CONFIG_EXAMPLE_I2C_MASTER_SDA 21
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 22
#define CONFIG_I2C_MASTER_SCL 19
#define CONFIG_I2C_MASTER_SDA 18
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "esp_system.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"

#include "nvs_flash.h"
#include "ping/ping_sock.h"
#include "driver/gpio.h"

#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"


#define SSID "TP-Link_918C"
#define PASS "16522060"
#define PORT_UDP 48569
#define HOST_IP_ADDR "192.168.1.106"
static const char *TAG = "UDP SOCKET CLIENT";
static const char *payload = "BME680 Sensor";
// Global variables that holds the sensor data values

volatile float temperature;
volatile float humidity;
volatile float pressure;
volatile float lux;

void bme680_test(void * pvParameters)
{
    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_2X, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();

    bme680_values_float_t values;
    while (1){
    
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);
            temperature = values.temperature;
            humidity = values.humidity;
            pressure = values.pressure;
            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &values) == ESP_OK)
                printf("temperature: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                        values.temperature, values.humidity, values.pressure, values.gas_resistance);

            temperature = values.temperature;
            humidity = values.humidity;
            pressure = values.pressure;
            vTaskDelay(1000);
        }
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
        vTaskDelete(NULL);
    
}


static void udp_client_task(void * pvParameters)
{   
    char buffer[128]; /*buffer to store the data to be sent*/
    char rx_buffer[128]; /*buffer to store the received data*/
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);/* destination addr*/
        dest_addr.sin_family = AF_INET; /*IPv4 address(AF_INET)*/
        dest_addr.sin_port = htons(PORT_UDP);/* Port used for the UDP transmission*/
        addr_family = AF_INET; 
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            return;
        

        
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", host_ip, PORT_UDP);

                  
            
            while(1) {
            const char *payload_format = "temperature= %.2f°C humidity=%.2f %%, lux=%.2f";
            snprintf(buffer, sizeof(buffer), payload_format, temperature, humidity, lux);
            int err = sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                
            } else {
            ESP_LOGI(TAG, "Message sent");
            vTaskDelay(1000);
            }
            
            }
            
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    
                }
            }
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    
    vTaskDelete(NULL);

}
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}
void bht1750()
{
    bh1750_dev_t dev_1;
    esp_err_t err;

    bh1750_i2c_hal_init();

    /* Device init */
    dev_1.i2c_addr = I2C_ADDRESS_BH1750;
    dev_1.mtreg_val = DEFAULT_MEAS_TIME_REG_VAL;

    /* Perform device reset */
    err = bh1750_i2c_dev_reset(dev_1); 
    ESP_LOGI(TAG, "Device reset: %s", err == BH1750_OK ? "Successful" : "Failed");

    err += bh1750_i2c_set_power_mode(dev_1, BH1750_POWER_ON);
    ESP_LOGI(TAG, "Changing power mode to ON: %s", err == BH1750_OK ? "Successful" : "Failed");

    /* Change measurement time with  50% optical window transmission rate */
    err += bh1750_i2c_set_mtreg_val(&dev_1, 50);
    ESP_LOGI(TAG, "Changing measurement time: %s", err == BH1750_OK ? "Successful" : "Failed");

    /* Configure device */
    err += bh1750_i2c_set_resolution_mode(&dev_1, BH1750_CONT_H_RES_MODE);
    if (err == BH1750_OK)
    {
        ESP_LOGI(TAG, "BH1750 config successful");
    }
    else{
        ESP_LOGE(TAG, "BH1750 config failed!");
    }
    /* End of device config */

    if (err == BH1750_OK)
    {
        ESP_LOGI(TAG, "BH1750 initialization successful");
        //Start reading data
        uint16_t data_light;
        while(1)
        {
            bh1750_i2c_read_data(dev_1, &data_light);
            ESP_LOGI(TAG, "Light Intensity: %d Lux", data_light);
            lux = data_light;
            vTaskDelay(1000);
        }
    }
    else{
        ESP_LOGE(TAG, "BH1750 initialization failed!");
    }
}

void wifi_connection()
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_connect();
}

void app_main(void)
{   
    
    wifi_connection();
    vTaskDelay(5000 / portTICK_PERIOD_MS);    
    ESP_ERROR_CHECK(i2cdev_init());
    
    //xTaskCreate(bme680_test, "udp_send", 2048,NULL, 2, NULL);
    //vTaskDelay(1000);
    xTaskCreate(bht1750, "bht1750", 4096,NULL, 1, NULL);
    vTaskDelay(1000);
    xTaskCreate(udp_client_task, "udp_send", 4096,NULL, 1, NULL);
    
   
    
}
