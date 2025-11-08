#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h" 
#include "driver/gpio.h"
#include "driver/adc.h"
#include "dht.h"

// --- Cấu hình Cứng ---
#define EXAMPLE_ESP_MAXIMUM_RETRY   5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Cấu hình AP Mode 
#define CONFIG_AP_SSID     "ESp32"
#define CONFIG_AP_PASSWORD "123456789"

// Cấu hình chân (Pins)
#define DHT_PIN 4
#define SOIL_PIN 32
#define RELAY_PIN 18
#define BUTTON_PIN 19

// Cấu hình NVS 
#define NVS_NAMESPACE "storage"
#define NVS_WIFI_SSID "ssid"
#define NVS_WIFI_PASS "pass"
#define NVS_DEVICE_ID "dev_id"
#define NVS_DATA_CYCLE "data_cycle"
#define NVS_SOIL_MIN "soil_min"
#define NVS_SOIL_MAX "soil_max"
#define NVS_AUTO_ENABLE "auto_en"

// Giá trị mặc định
#define DEFAULT_DEVICE_ID "esp32-01"
#define DEFAULT_DATA_CYCLE_MS 60000 

// --- Biến Global ---
static const char *TAG = "ESP32_APP";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// Biến lưu trữ cấu hình
static char wifi_ssid[32] = {0};
static char wifi_password[64] = {0};
static char device_id[32] = DEFAULT_DEVICE_ID;
static int data_cycle_ms = DEFAULT_DATA_CYCLE_MS;
static int soil_min = 40, soil_max = 60;
static bool auto_enable = false;

// Biến trạng thái
dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
float temperature, humidity;
int soil_moisture;
int relay_state = 0;

// Handles
httpd_handle_t server = NULL;
static esp_netif_t *s_ap_netif = NULL; 

// --- Khai báo hàm (Prototypes) ---
static void start_webserver();
static void wifi_transition_task(void *pvParameters); 
static void sensor_task(void *pvParameters);
static void button_task(void *pvParameters);

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Xử lý sự kiện WiFi ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        start_webserver(); 
    }
}

// --- Đọc/Ghi NVS ---
static esp_err_t load_config() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return err;

    size_t len;
    
    len = sizeof(wifi_ssid);
    if (nvs_get_str(nvs_handle, NVS_WIFI_SSID, wifi_ssid, &len) == ESP_OK) {
        len = sizeof(wifi_password);
        nvs_get_str(nvs_handle, NVS_WIFI_PASS, wifi_password, &len);
    }

    len = sizeof(device_id);
    if (nvs_get_str(nvs_handle, NVS_DEVICE_ID, device_id, &len) != ESP_OK) {
        strncpy(device_id, DEFAULT_DEVICE_ID, sizeof(device_id));
    }

    if (nvs_get_u32(nvs_handle, NVS_DATA_CYCLE, (uint32_t*)&data_cycle_ms) != ESP_OK) {
        data_cycle_ms = DEFAULT_DATA_CYCLE_MS;
    }

    nvs_get_u32(nvs_handle, NVS_SOIL_MIN, (uint32_t*)&soil_min);
    nvs_get_u32(nvs_handle, NVS_SOIL_MAX, (uint32_t*)&soil_max);
    nvs_get_u8(nvs_handle, NVS_AUTO_ENABLE, (uint8_t*)&auto_enable);

    nvs_close(nvs_handle);
    return ESP_OK;
}

static esp_err_t save_config() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    nvs_set_str(nvs_handle, NVS_WIFI_SSID, wifi_ssid);
    nvs_set_str(nvs_handle, NVS_WIFI_PASS, wifi_password);
    nvs_set_str(nvs_handle, NVS_DEVICE_ID, device_id);
    nvs_set_u32(nvs_handle, NVS_DATA_CYCLE, (uint32_t)data_cycle_ms);
    nvs_set_u32(nvs_handle, NVS_SOIL_MIN, (uint32_t)soil_min);
    nvs_set_u32(nvs_handle, NVS_SOIL_MAX, (uint32_t)soil_max);
    nvs_set_u8(nvs_handle, NVS_AUTO_ENABLE, (uint8_t)auto_enable);

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

// --- Khởi tạo WiFi ---
static void wifi_init_sta() {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = { .ssid = "", .password = "" },
    };
    strcpy((char*)wifi_config.sta.ssid, wifi_ssid);
    strcpy((char*)wifi_config.sta.password, wifi_password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Đang chờ kết nối...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", wifi_ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", wifi_ssid);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void wifi_init_ap() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    s_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(CONFIG_AP_SSID),
            .channel = 1,
            .password = CONFIG_AP_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strcpy((char*)wifi_config.ap.ssid, CONFIG_AP_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s", CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
}

// --- Logic Cảm biến & Điều khiển ---
static void read_sensors() {
    if (dht_read_float_data(sensor_type, (gpio_num_t)DHT_PIN, &humidity, &temperature) == ESP_OK) {
        ESP_LOGI(TAG, "Temp: %.1f°C, Hum: %.1f%%", temperature, humidity);
    } else {
        ESP_LOGE(TAG, "Failed to read DHT sensor");
    }
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);
    int raw = adc1_get_raw(ADC1_CHANNEL_4);
    soil_moisture = map(raw, 0, 4095, 0, 100); 
    soil_moisture = (soil_moisture - 100) * -1; 

    ESP_LOGI(TAG, "Soil: %d%% (Raw: %d)", soil_moisture, raw);
}

static void auto_control() {
    if (auto_enable) {
        if (soil_moisture < soil_min && relay_state == 0) {
            relay_state = 1;
            gpio_set_level((gpio_num_t)RELAY_PIN, relay_state);
            ESP_LOGI(TAG, "Relay ON (auto)");
        } else if (soil_moisture >= soil_max && relay_state == 1) {
            relay_state = 0;
            gpio_set_level((gpio_num_t)RELAY_PIN, relay_state);
            ESP_LOGI(TAG, "Relay OFF (auto)");
        }
    }
}

static void button_task(void *pvParameters) {
    gpio_set_direction((gpio_num_t)BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)BUTTON_PIN, GPIO_PULLUP_ONLY);
    
    int last_state = 1;
    while (1) {
        int state = gpio_get_level((gpio_num_t)BUTTON_PIN);
        if (state == 0 && last_state == 1) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (gpio_get_level((gpio_num_t)BUTTON_PIN) == 0) {
                relay_state = !relay_state;
                gpio_set_level((gpio_num_t)RELAY_PIN, relay_state);
                ESP_LOGI(TAG, "Relay %s (button)", relay_state ? "ON" : "OFF");
            }
        }
        last_state = state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void sensor_task(void *pvParameters) {
    gpio_set_direction((gpio_num_t)RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)RELAY_PIN, relay_state);

    while (1) {
        read_sensors();
        auto_control();
        vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
}

// --- Web Server Handlers ---
static esp_err_t config_get_handler(httpd_req_t *req) {
    char response[1024]; 
    snprintf(response, sizeof(response),
        "<html><head><title>ESP32 Config</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "</head><body><h1>ESP32 Config</h1>"
        "<form method='POST' action='/config'>"
        "Device ID: <input name='dev_id' value='%s'><br>"
        "Data Cycle (ms): <input name='cycle' value='%d'><br><hr>"
        "<b>Auto Control</b><br>"
        "Soil Min (Turn ON): <input name='min' value='%d'><br>"
        "Soil Max (Turn OFF): <input name='max' value='%d'><br>"
        "Auto Enable: <input type='checkbox' name='auto' %s><br><hr>"
        "<b>WiFi Config</b><br>"
        "WiFi SSID: <input name='ssid' value='%s'><br>"
        "WiFi Pass: <input name='pass' type='password' placeholder='********' value='%s'><br>"
        "<br><input type='submit' value='Save & Connect'>"
        "</form></body></html>",
        device_id, data_cycle_ms, soil_min, soil_max, 
        auto_enable ? "checked" : "", 
        wifi_ssid, wifi_password); 

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    char* buf = NULL;
    size_t buf_len = req->content_len;

    if (buf_len <= 0) {
        ESP_LOGE(TAG, "No content in POST request");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    buf = malloc(buf_len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory for POST buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int ret = httpd_req_recv(req, buf, buf_len);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
        free(buf);
        return ESP_FAIL;
    }
    buf[ret] = '\0'; 

    char param[64]; 

    if (httpd_query_key_value(buf, "dev_id", param, sizeof(param)) == ESP_OK) {
        strncpy(device_id, param, sizeof(device_id) - 1);
    }
    if (httpd_query_key_value(buf, "cycle", param, sizeof(param)) == ESP_OK) {
        data_cycle_ms = atoi(param);
    }
    if (httpd_query_key_value(buf, "min", param, sizeof(param)) == ESP_OK) {
        soil_min = atoi(param);
    }
    if (httpd_query_key_value(buf, "max", param, sizeof(param)) == ESP_OK) {
        soil_max = atoi(param);
    }
    if (httpd_query_key_value(buf, "ssid", param, sizeof(param)) == ESP_OK) {
        strncpy(wifi_ssid, param, sizeof(wifi_ssid) - 1);
    }
    if (httpd_query_key_value(buf, "pass", param, sizeof(param)) == ESP_OK) {
        strncpy(wifi_password, param, sizeof(wifi_password) - 1);
    }

    auto_enable = (httpd_query_key_value(buf, "auto", param, sizeof(param)) == ESP_OK);

    free(buf);

    save_config();
    ESP_LOGI(TAG, "Config saved. Starting WiFi transition...");
    
    const char* resp_msg = "<html><body><h1>Configuration received.</h1>"
                            "<h2>Turning off AP and attempting to connect to <b>%s</b>...</h2>"
                            "<p>You can close this page. If the connection is successful, "
                            "the device will get a new IP address.</p></body></html>";
    char resp_buffer[256];
    snprintf(resp_buffer, sizeof(resp_buffer), resp_msg, wifi_ssid);
    httpd_resp_send(req, resp_buffer, strlen(resp_buffer));

    xTaskCreate(wifi_transition_task, "wifi_transition_task", 4096, NULL, 10, NULL);

    return ESP_OK;
}

// --- Tác vụ Chuyển đổi WiFi ---
static void wifi_transition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Transition task started.");
    vTaskDelay(pdMS_TO_TICKS(100));

    if (server) {
        ESP_LOGI(TAG, "Stopping web server...");
        httpd_stop(server);
        server = NULL;
    }

    ESP_LOGI(TAG, "Stopping AP mode and cleaning up...");
    ESP_ERROR_CHECK(esp_wifi_stop());
    if (s_ap_netif) {
        esp_netif_destroy(s_ap_netif);
        s_ap_netif = NULL;
    }
    
    ESP_LOGI(TAG, "Initializing STA mode...");
    wifi_init_sta();

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA Connected. Starting main tasks.");
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
        xTaskCreate(button_task, "button_task", 2048, NULL, 4, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to connect to STA. Restarting in 10s...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart(); 
    }

    ESP_LOGI(TAG, "Transition task finished. Deleting self.");
    vTaskDelete(NULL);
}

// --- Khởi động Web Server ---
static void start_webserver() {
    if (server != NULL) {
        ESP_LOGW(TAG, "Web server already running.");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t config_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = config_get_handler,
    };
    httpd_uri_t config_post = {
        .uri       = "/config",
        .method    = HTTP_POST,
        .handler   = config_post_handler,
    };

    ESP_LOGI(TAG, "Starting web server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &config_uri);
        httpd_register_uri_handler(server, &config_post);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_config(); 

    ESP_LOGI(TAG, "Khởi động ở chế độ AP để chờ cấu hình.");
    wifi_init_ap();
    start_webserver();

    ESP_LOGI(TAG, "Hệ thống đã sẵn sàng. Truy cập http://192.168.4.1 để cấu hình.");
}