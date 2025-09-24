#include "http_client.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "sensor_manager.h"
#include "cJSON.h"
#include <string.h>
#include "esp_timer.h"

static const char *TAG = "HTTP_CLIENT";

static struct {
    bool initialized;
    http_client_config_t config;
    TaskHandle_t send_task_handle;
    char full_url[256];
} client = {0};

// Build JSON payload with sensor data
static char* build_json_payload(void) {
    sensor_status_t status;
    sensor_data_t sht31_data = {0}, bmp280_data = {0};
    
    sensor_manager_get_status(&status);
    
    cJSON *root = cJSON_CreateObject();
    cJSON *sensors = cJSON_CreateObject();
    
    // Add timestamp
    cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000000);
    
    // Add SHT31 data if available
    if (status.sht31_found) {
        sensor_manager_get_data(SENSOR_TYPE_SHT31, &sht31_data);
        cJSON *sht31 = cJSON_CreateObject();
        cJSON_AddNumberToObject(sht31, "temperature", sht31_data.temperature);
        cJSON_AddNumberToObject(sht31, "humidity", sht31_data.humidity);
        cJSON_AddNumberToObject(sht31, "dew_point", sht31_data.dew_point);
        cJSON_AddItemToObject(sensors, "sht31", sht31);
    }
    
    // Add BMP280 data if available
    if (status.bmp280_found) {
        sensor_manager_get_data(SENSOR_TYPE_BMP280, &bmp280_data);
        cJSON *bmp280 = cJSON_CreateObject();
        cJSON_AddNumberToObject(bmp280, "temperature", bmp280_data.temperature);
        cJSON_AddNumberToObject(bmp280, "pressure", bmp280_data.pressure);
        cJSON_AddNumberToObject(bmp280, "altitude", bmp280_data.altitude);
        cJSON_AddItemToObject(sensors, "bmp280", bmp280);
    }
    
    // Add sensor status
    cJSON *status_obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(status_obj, "total_sensors", status.total_sensors);
    cJSON_AddNumberToObject(status_obj, "read_count", status.read_count);
    cJSON_AddNumberToObject(status_obj, "error_count", status.error_count);
    cJSON_AddItemToObject(root, "status", status_obj);
    
    // Add sensors data
    cJSON_AddItemToObject(root, "sensors", sensors);
    
    // Add device info (optional - helps identify which ESP32 is sending)
    cJSON_AddStringToObject(root, "device_id", "ESP32_ENV_01");
    
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    return json_string;
}

// HTTP event handler
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            // You can handle server response here if needed
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
    return ESP_OK;
}

// Send sensor data to server
esp_err_t http_client_send_sensor_data(void) {
    if (!client.initialized) {
        ESP_LOGE(TAG, "HTTP client not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    char *json_payload = build_json_payload();
    if (json_payload == NULL) {
        ESP_LOGE(TAG, "Failed to build JSON payload");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Sending data to %s", client.full_url);
    ESP_LOGD(TAG, "Payload: %s", json_payload);
    
    esp_http_client_config_t config = {
        .url = client.full_url,
        .event_handler = http_event_handler,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .buffer_size = 512,
    };
    
    esp_http_client_handle_t http_client = esp_http_client_init(&config);
    
    // Set headers
    esp_http_client_set_header(http_client, "Content-Type", "application/json");
    
    // Set post data
    esp_http_client_set_post_field(http_client, json_payload, strlen(json_payload));
    
    // Perform HTTP request
    esp_err_t err = esp_http_client_perform(http_client);
    
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(http_client);
        int content_length = esp_http_client_get_content_length(http_client);
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", status_code, content_length);
        
        if (status_code == 200 || status_code == 201) {
            ESP_LOGI(TAG, "Data sent successfully");
        } else {
            ESP_LOGW(TAG, "Server returned status code: %d", status_code);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(http_client);
    free(json_payload);
    
    return err;
}

// Auto-send task
static void http_send_task(void *pvParameters) {
    ESP_LOGI(TAG, "Auto-send task started");
    
    while (client.config.auto_send_enabled) {
        http_client_send_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(client.config.send_interval_ms));
    }
    
    ESP_LOGI(TAG, "Auto-send task stopped");
    client.send_task_handle = NULL;
    vTaskDelete(NULL);
}

// Initialize HTTP client
esp_err_t http_client_init(const http_client_config_t *config) {
    if (client.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&client.config, config, sizeof(http_client_config_t));
    
    // Build full URL
    snprintf(client.full_url, sizeof(client.full_url), "http://%s:%d%s", 
             client.config.server_url, client.config.port, client.config.api_endpoint);
    
    client.initialized = true;
    ESP_LOGI(TAG, "HTTP client initialized. Server: %s", client.full_url);
    
    return ESP_OK;
}

// Start auto-send
esp_err_t http_client_start_auto_send(void) {
    if (!client.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (client.config.auto_send_enabled) {
        ESP_LOGW(TAG, "Auto-send already enabled");
        return ESP_OK;
    }
    
    client.config.auto_send_enabled = true;
    
    if (xTaskCreate(http_send_task, "http_send", 8192, NULL, 5, &client.send_task_handle) != pdPASS) {
        client.config.auto_send_enabled = false;
        ESP_LOGE(TAG, "Failed to create send task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Auto-send started (interval: %lu ms)", client.config.send_interval_ms);
    return ESP_OK;
}

// Test connection
esp_err_t http_client_test_connection(void) {
    ESP_LOGI(TAG, "Testing connection to %s", client.full_url);
    return http_client_send_sensor_data();
}