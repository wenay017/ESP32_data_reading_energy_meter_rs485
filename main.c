#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include "driver/gpio.h"
#include "config.h"
uint16_t calculate_crc(uint8_t *data, int length);
void modbusSendRequest(uint8_t sensorAddress, uint16_t registerAddress);
bool modbusReadResponse(uint8_t *data, TickType_t timeout);
void processModbusDataFloat(uint8_t *data);
void send_at_command(const char *command);

uint16_t calculate_crc(uint8_t *data, int length) {
    uint16_t crc = 0xFFFF; // Initial value

    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001; // Polynomial used in Modbus RTU
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}


void modbusSendRequest(uint8_t sensorAddress, uint16_t registerAddress) {
    uint8_t request_data[8] = {
        sensorAddress,          // Sensor address
        0x03,                   // Function code (0x03 for reading holding registers)
        (registerAddress >> 8) & 0xFF, // Starting register address high byte
        registerAddress & 0xFF,         // Starting register address low byte
        0x00,                   // Number of registers to read high byte
        0x50,                   // Number of registers to read low byte (2 registers for a float)
        0x00,                   // CRC placeholder
        0x00                    // CRC placeholder
    };

    uint16_t crc = calculate_crc(request_data, 6);
    request_data[6] = crc & 0xFF;
    request_data[7] = (crc >> 8) & 0xFF;

    ESP_LOGI(TAG, "Sending %d bytes:", sizeof(request_data));
    ESP_LOG_BUFFER_HEX(TAG, request_data, sizeof(request_data));

    int bytesSent = uart_write_bytes(UART_NUM_2, (const char *)request_data, sizeof(request_data));
    ESP_LOGI(TAG, "Sent %d bytes.", bytesSent);
    vTaskDelay(pdMS_TO_TICKS(500));
}




bool modbusReadResponse(uint8_t *data, TickType_t timeout) {
    int bytesRead = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, timeout);
    if (bytesRead >= 7) {
        ESP_LOGI(TAG, "Bytes read: %d", bytesRead);
        ESP_LOG_BUFFER_HEX(TAG, data, bytesRead);

        uint16_t crc = calculate_crc(data, bytesRead - 2);
        uint16_t received_crc = (data[bytesRead - 1] << 8) | data[bytesRead - 2];
        if (crc == received_crc) {
            return true;
        } else {
            ESP_LOGE(TAG, "CRC mismatch. Calculated: 0x%04X, Received: 0x%04X", crc, received_crc);
            return false;
        }
    } else {
        ESP_LOGI(TAG, "No data received from Modbus.");
        return false;
    }
}

float bytesToFloat(uint8_t *data) {
    // Correct byte order: depending on endianness of your system and data format from device
    uint32_t intValue = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6];
    float floatValue;
    memcpy(&floatValue, &intValue, sizeof(float));
    return floatValue;
}
void processModbusDataFloat(uint8_t *data) {
    float currentAverage = bytesToFloat(data);
    printf("A Current Average: %f\n", currentAverage);
}

void RESTART_task(void *arg)
{
      vTaskDelay(pdMS_TO_TICKS(Restart_delay));
      esp_restart();
}
static void echo_task(void *arg) {
    while (1) {
        modbusSendRequest(MODBUS_SENSOR_ADDRESS, MODBUS_REGISTER_ADDRESS);
        if (modbusReadResponse(response, pdMS_TO_TICKS(500))) {
            processModbusDataFloat(response);
        } else {
            ESP_LOGI(TAG, "Failed to read Modbus response.");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
         modbusSendRequest(MODBUS_SENSOR_ADDRESS, MODBUS_REGISTER_ADDRESS);
        if (modbusReadResponse(response, pdMS_TO_TICKS(500))) {
            processModbusDataFloat(response);
        } else {
            ESP_LOGI(TAG, "Failed to read Modbus response.");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
       
    
           vTaskDelay(pdMS_TO_TICKS(sample_read_delay));
    }
}

void HTTP_POST_task(void *arg)
{
     vTaskDelay(pdMS_TO_TICKS(5000));
    while(1)
    {
      
        vTaskDelay(pdMS_TO_TICKS(post_time));
    const char *url_template = "http://3.6.204.10/public/api/updateDeviceDatanisensu?key=chinnu&token=257bbec888a81696529ee979804cca59&tds=%u&conductivity=%u&water_consumption=0&device_id=%d&water_level=0&temp=0&power_on=0&voltage_4=%u&voltage_5=%u&voltage_6=%u&voltage_7=%u&voltage_8=%u&current_1=%u&current_2=%u&current_3=%u&current_4=0&low_pressure=1&filtering_on=0";


    size_t url_len = snprintf(NULL, 0, url_template, value1, value2,deviceid, value3,value4,value5,value6,value7,value8,value9,value10) + 1;
    char *url = (char *)malloc(url_len);
    snprintf(url, url_len, url_template,value1, value2,deviceid, value3,value4,value5,value6,value7,value8,value9,value10);
    size_t command_len = strlen(url) + 20;  
    char *command = malloc(command_len);
    snprintf(command, command_len, "AT+HTTPURL=\"%s\"\r\n", url);
    send_at_command(command);
    free(command);
  

    // Send HTTP POST request
    send_at_command("AT+HTTPREQUEST=POST\r\n");


    // Read status response
    do {
        len = uart_read_bytes(uart_num, (uint8_t*)buffer, BUF_SIZE1, 1000 / portTICK_RATE_MS);
        buffer[len] = '\0';
        #if DEBUG
        ESP_LOGI(TAG1, "Status Response: %s", buffer);
        #endif
    } while (len > 0);
       // Optional: Read and print content response
    send_at_command("AT+HTTPGETSTAT?\r\n");

    // Optional: Read and print content response
     do {
        len = uart_read_bytes(uart_num, (uint8_t*)buffer, BUF_SIZE1, 1000 / portTICK_RATE_MS);
        buffer[len] = '\0';
        #if DEBUG
        ESP_LOGI(TAG1, "Response: %s", buffer);
        #endif
    } while (len>0);
    send_at_command("AT+HTTPGETCONT=0\r\n");
    // vTaskDelay(1000 / portTICK_RATE_MS);
    do {
        len = uart_read_bytes(uart_num, (uint8_t*)buffer, BUF_SIZE1, 1000 / portTICK_RATE_MS);
        buffer[len] = '\0';
        #if DEBUG
        ESP_LOGI(TAG1, "Content: %s", buffer);
        #endif
    } while (len > 0);

    }

}
void configure_uart() {
    #if DEBUG
    ESP_LOGI(TAG1, "Configuring UART...");
    #endif
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t uart_config_status = uart_param_config(uart_num, &uart_config);
    if (uart_config_status != ESP_OK) {
        ESP_LOGE(TAG1, "UART configuration failed: %s", esp_err_to_name(uart_config_status));
        // Handle the error accordingly
        return;
    }
    #if DEBUG
      ESP_LOGI(TAG1, "UART configured successfully");
      #endif
    esp_err_t uart_install_status = uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    if (uart_install_status != ESP_OK) {
        ESP_LOGE(TAG1, "UART driver installation failed: %s", esp_err_to_name(uart_install_status));
        // Handle the error accordingly
        return;
    }
    #if DEBUG
    ESP_LOGI(TAG1, "UART driver installation successful");
    #endif

    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void send_at_command(const char *command) {
    #if DEBUG
    ESP_LOGI(TAG1, "Sending command: %s", command); // Print the command before sending
    #endif

    uart_write_bytes(uart_num, command, strlen(command));
    uart_write_bytes(uart_num, "\r\n", 2);
}
char *wait_for_response(const char *expected_response, TickType_t timeout) {
    char response_buffer[256];  
    size_t response_length = 0;  
    TickType_t start_time = xTaskGetTickCount();

    while (1) {
        uint8_t byte;
        if (uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(50)) == 1) {
            if (response_length < sizeof(response_buffer) - 1) {
                response_buffer[response_length++] = (char)byte;
            }

            response_buffer[response_length] = '\0';  // Null-terminate the string

            if (strstr(response_buffer, expected_response) != NULL) {
                #if DEBUG
                ESP_LOGI(TAG1, "Waiting for response: %s", expected_response);
                ESP_LOGI(TAG1, "Received response: %s", response_buffer);
                #endif
                return strdup(response_buffer);  // Return a copy of the buffer
            }
        }

        if (xTaskGetTickCount() - start_time >= timeout) {
            break;
        }
    }

    return NULL;  // Return NULL if timeout occurs
}


void set_apn(const char *apn) {
    // Send AT command to set the APN
    char command[64];
    snprintf(command, sizeof(command), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    send_at_command(command);

    // Wait for "OK" response with a timeout
    if (wait_for_response("OK", pdMS_TO_TICKS(5000))) {
        #if DEBUG
        ESP_LOGI(TAG1, "APN configured: %s", apn);
        #endif
    } else {
        ESP_LOGE(TAG1, "Failed to configure APN");
        // Handle the error accordingly
    }
}
bool configure_gsm() {
    send_at_command("AT^SIMSWAP?\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    // send_at_command("AT^SIMSWAP=1\r\n");
    // vTaskDelay(pdMS_TO_TICKS(5000)); 
    // send_at_command("AT^SIMSWAP=1\r\n");
    // vTaskDelay(pdMS_TO_TICKS(5000)); 
     send_at_command("AT\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); 
     send_at_command("ATI\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    send_at_command("AT$QCNETDEVCTL=3,1\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); 
    // Set the APN
    set_apn("airtelgprs.com");
     do {
        len = uart_read_bytes(uart_num, (uint8_t*)buffer, BUF_SIZE1, 1000 / portTICK_RATE_MS);
        buffer[len] = '\0';
        #if DEBUG
        ESP_LOGI(TAG1, "Response: %s", buffer);
        #endif
    } while (len>0);
    // Additional GSM module configuration
    send_at_command("AT+CFUN=1\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    send_at_command("AT+CGATT=1\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    send_at_command("AT+CGACT=1,1\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
     do {
        len = uart_read_bytes(uart_num, (uint8_t*)buffer, BUF_SIZE1, 1000 / portTICK_RATE_MS);
        buffer[len] = '\0';
        #if DEBUG
        ESP_LOGI(TAG1, "Response: %s", buffer);
        #endif
    } while (len>0);
    // Check current network registration status
    send_at_command("AT+CREG?\r\n");
    if (!wait_for_response("+CREG: 0,6", pdMS_TO_TICKS(1000))) {
        ESP_LOGE(TAG1, "Failed to register on the network");
        return false;
    } else {
        #if DEBUG
        ESP_LOGI(TAG1, "Registered on the network");
        #endif
    }
     return true;
}
void getMacAddress() {
    uint8_t macAddress[6];
    esp_read_mac(macAddress, ESP_MAC_WIFI_STA);

    // Combine the bytes into a 64-bit unsigned integer
    globalMacAddress = ((uint64_t)macAddress[0] << 40) |
                       ((uint64_t)macAddress[1] << 32) |
                       ((uint64_t)macAddress[2] << 24) |
                       ((uint64_t)macAddress[3] << 16) |
                       ((uint64_t)macAddress[4] << 8) |
                       macAddress[5];
     deviceid=globalMacAddress%100000000;

    // Print the combined MAC address
    ESP_LOGI(TAG, "Global MAC Address: %" PRIu64, globalMacAddress%100000000);//used last 8 digits only as device id you can change it as your requirement in posting part
}
void app_main() {
     getMacAddress();
    // Configure UART
    configure_uart();
    ESP_LOGI(TAG, "RS485 Echo Application started.");

    // Setup UART
    uart_config_t uart_config = {
        .baud_rate = MODBUS_SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE, // 8E1 configuration
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, MODBUS_TX_PIN, MODBUS_RX_PIN, 5, UART_PIN_NO_CHANGE);

    if (uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0) == ESP_OK) {
        ESP_LOGI(TAG, "Successfully installed UART driver.");
    } else {
        ESP_LOGE(TAG, "Failed to install UART driver.");
    }

    uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_rx_timeout(UART_NUM_2, PACKET_READ_TICS);
    //---------gpio to turn on gsm modem ------------------------------
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // Set GPIO 21 high
    gpio_set_level(GPIO_OUTPUT, 1);  //this one to turn on modem
    
    bool gsm_config_success = configure_gsm();
    if (!gsm_config_success) {
            ESP_LOGE(TAG1, "GSM configuration failed.");
            // You might want to handle this error, reset the GSM module, or take appropriate action
        } else {
            #if DEBUG
            ESP_LOGI(TAG1, "GSM configuration successful.");
            // printf("GSM CONFIGURATION SUCCESSFUL\n");
            #endif
        }   // Configure GPIO pins for relays
   
    // Create relay control task
    // xTaskCreate(&relay_task, "relay_task", 2048, NULL, 10, NULL);
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 9, NULL);
     xTaskCreate(HTTP_POST_task, "HTTP_POST_task", 2048, NULL, 7, NULL);
     xTaskCreate(RESTART_task, "RESTART_task", 2048, NULL, 7, NULL);

}
