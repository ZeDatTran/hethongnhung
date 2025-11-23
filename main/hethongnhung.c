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
#include "esp_adc/adc_oneshot.h"
#include "dht.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

// --- Cấu hình Firebase ---
#define FIREBASE_HOST "https://esp32-a12b7-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "sOV3expxoWa9LSNLl6YrZJZXFyZkM9mRpQkuYOL2"

// --- Cấu hình Cứng ---
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Cấu hình AP Mode
#define CONFIG_AP_SSID "ESp32"
#define CONFIG_AP_PASSWORD "123456789"

// Cấu hình chân (Pins)
#define DHT_PIN 4
#define SOIL_PIN ADC_CHANNEL_4
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
#define DEFAULT_DATA_CYCLE_MS 2000

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
float temperature = 0.0, humidity = 0.0;
int soil_moisture = 0;
int relay_state = 0;

// Biến quét WiFi
static char wifi_list[10][32];
static int wifi_rssi[10];
static int wifi_count = 0;

// Handles
httpd_handle_t server = NULL;
static esp_netif_t *s_ap_netif = NULL;
adc_oneshot_unit_handle_t adc1_handle;

// --- Khai báo hàm (Prototypes) ---
static void start_webserver();
static void stop_webserver();
static void wifi_transition_task(void *pvParameters);
static void sensor_task(void *pvParameters);
static void button_task(void *pvParameters);
static esp_err_t scan_wifi();
static esp_err_t save_config();
static void firebase_push_config(); // [MỚI] Thêm hàm push config lên Firebase

// --- Hàm tiện ích ---
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer;
    static int output_len;
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (evt->user_data) {
            memcpy(evt->user_data + output_len, evt->data, evt->data_len);
            output_len += evt->data_len;
        }
    } else if (evt->event_id == HTTP_EVENT_ON_FINISH) {
        output_len = 0;
    }
    return ESP_OK;
}

// 1. Hàm gửi dữ liệu cảm biến (Push Status)
static void firebase_push_data() {
    char url[512];
    snprintf(url, sizeof(url), "%s/devices/%s/status.json?auth=%s",
             FIREBASE_HOST, device_id, FIREBASE_AUTH);
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temperature", temperature);
    cJSON_AddNumberToObject(root, "hum", humidity);
    cJSON_AddNumberToObject(root, "soilmoisture", soil_moisture);
    cJSON_AddNumberToObject(root, "relay_state", relay_state);
   
    char *post_data = cJSON_PrintUnformatted(root);
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_PATCH,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
    };
   
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
   
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firebase Push Status OK. Status: %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "Firebase Push Status Failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    cJSON_Delete(root);
    free(post_data);
}

// [MỚI] 2. Hàm gửi cấu hình lên Firebase (Push Config)
static void firebase_push_config() {
    char url[512];
    snprintf(url, sizeof(url), "%s/devices/%s/config.json?auth=%s",
             FIREBASE_HOST, device_id, FIREBASE_AUTH);
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "auto_en", auto_enable);
    cJSON_AddNumberToObject(root, "soil_min", soil_min);
    cJSON_AddNumberToObject(root, "soil_max", soil_max);
    cJSON_AddNumberToObject(root, "dataCycle", data_cycle_ms);
    cJSON_AddNumberToObject(root, "pump_state", relay_state);
   
    char *post_data = cJSON_PrintUnformatted(root);
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_PATCH,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
    };
   
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
   
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firebase Push Config OK. Status: %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "Firebase Push Config Failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    cJSON_Delete(root);
    free(post_data);
}

// 3. Hàm lấy cấu hình từ Firebase (Pull Config)
static void firebase_get_config() {
    char url[512];
    char response_buffer[1024] = {0};
   
    snprintf(url, sizeof(url), "%s/devices/%s/config.json?auth=%s",
             FIREBASE_HOST, device_id, FIREBASE_AUTH);
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = _http_event_handler,
        .user_data = response_buffer,
        .timeout_ms = 5000,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK && esp_http_client_get_status_code(client) == 200) {
        cJSON *root = cJSON_Parse(response_buffer);
        if (root) {
            cJSON *j_auto = cJSON_GetObjectItem(root, "auto_en");
            if (cJSON_IsBool(j_auto)) auto_enable = cJSON_IsTrue(j_auto);
           
            cJSON *j_min = cJSON_GetObjectItem(root, "soil_min");
            if (cJSON_IsNumber(j_min)) soil_min = j_min->valueint;
            
            cJSON *j_max = cJSON_GetObjectItem(root, "soil_max");
            if (cJSON_IsNumber(j_max)) soil_max = j_max->valueint;
            
            cJSON *j_cycle = cJSON_GetObjectItem(root, "dataCycle");
            if (cJSON_IsNumber(j_cycle) && j_cycle->valueint >= 1000) data_cycle_ms = j_cycle->valueint;
            
            cJSON *j_cmd = cJSON_GetObjectItem(root, "pump_state");
            if (cJSON_IsNumber(j_cmd) && !auto_enable) {
                if (j_cmd->valueint == 1) {
                    relay_state = 1;
                    gpio_set_level((gpio_num_t)RELAY_PIN, 1);
                } else if (j_cmd->valueint == 0) {
                    relay_state = 0;
                    gpio_set_level((gpio_num_t)RELAY_PIN, 0);
                }
            }
            
            cJSON_Delete(root);
        }
    }
    esp_http_client_cleanup(client);
}

// --- Xử lý sự kiện WiFi ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
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
    
    if (nvs_get_u32(nvs_handle, NVS_SOIL_MIN, (uint32_t*)&soil_min) != ESP_OK) {
        soil_min = 40;
    }
    
    if (nvs_get_u32(nvs_handle, NVS_SOIL_MAX, (uint32_t*)&soil_max) != ESP_OK) {
        soil_max = 60;
    }
    
    nvs_get_u8(nvs_handle, NVS_AUTO_ENABLE, (uint8_t*)&auto_enable);
    
    if (soil_min >= soil_max) {
        soil_min = 40;
        soil_max = 60;
    }
    
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
    
    ESP_LOGI(TAG, "wifi_init_sta finished. Waiting for connection...");
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", wifi_ssid);
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
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_country_code("01", true));
    
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s", CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
}

// --- Logic Cảm biến & Điều khiển ---
static void read_sensors() {
    int retries = 3;
    while (retries--) {
        if (dht_read_float_data(sensor_type, (gpio_num_t)DHT_PIN, &humidity, &temperature) == ESP_OK) {
            ESP_LOGI(TAG, "Temp: %.1f°C, Hum: %.1f%%", temperature, humidity);
            break;
        } else {
            ESP_LOGE(TAG, "Failed to read DHT sensor (retry %d)", retries);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    if (retries < 0) {
        temperature = 0.0;
        humidity = 0.0;
    }
    
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1_handle, SOIL_PIN, &raw);
    if (err == ESP_OK) {
        soil_moisture = map(raw, 0, 4095, 0, 100);
        soil_moisture = (soil_moisture - 100) * -1;
        if (soil_moisture < 0) soil_moisture = 0;
        if (soil_moisture > 100) soil_moisture = 100;
        ESP_LOGI(TAG, "Soil: %d%% (Raw: %d)", soil_moisture, raw);
    } else {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(err));
        soil_moisture = 0;
    }
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
    
    uint32_t last_push_time = 0;
    const uint32_t control_interval = 2000;
    
    while (1) {
        read_sensors();
        firebase_get_config();
        auto_control();
        
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if ((now - last_push_time) >= data_cycle_ms) {
            ESP_LOGI(TAG, "Uploading data to Firebase...");
            firebase_push_data();
            last_push_time = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(control_interval));
    }
}

// --- Quét WiFi ---
static esp_err_t scan_wifi() {
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) return err;
    
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 150,
    };
    
    err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) return err;
    
    uint16_t ap_count = 10;
    wifi_ap_record_t ap_records[10];
    err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (err != ESP_OK) return err;
    
    wifi_count = ap_count > 10 ? 10 : ap_count;
    for (int i = 0; i < wifi_count; i++) {
        strncpy(wifi_list[i], (char *)ap_records[i].ssid, sizeof(wifi_list[i]));
        wifi_list[i][31] = '\0';
        wifi_rssi[i] = ap_records[i].rssi;
        ESP_LOGI(TAG, "Found: %s, RSSI: %d", wifi_list[i], wifi_rssi[i]);
    }
    
    return ESP_OK;
}

// --- Web Server Handlers ---

// [SỬA] Handler lưu cấu hình + đồng bộ lên Firebase
static esp_err_t save_settings_post_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Handling POST request for /save-settings");
    
    char* buf = NULL;
    size_t buf_len = req->content_len;
    
    if (buf_len <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    buf = malloc(buf_len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    int ret = httpd_req_recv(req, buf, buf_len);
    if (ret <= 0) {
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
        if (data_cycle_ms < 1000) data_cycle_ms = 1000;
    }
    
    if (httpd_query_key_value(buf, "min", param, sizeof(param)) == ESP_OK) {
        soil_min = atoi(param);
    }
    
    if (httpd_query_key_value(buf, "max", param, sizeof(param)) == ESP_OK) {
        soil_max = atoi(param);
    }
    
    if (httpd_query_key_value(buf, "section", param, sizeof(param)) == ESP_OK && strcmp(param, "auto") == 0) {
       auto_enable = (httpd_query_key_value(buf, "auto", param, sizeof(param)) == ESP_OK);
    }
    
    if (soil_min >= soil_max) {
        soil_min = 40;
        soil_max = 60;
    }
    
    free(buf);
    
    // Lưu vào NVS
    save_config();
    
    // [MỚI] Đồng bộ lên Firebase
    firebase_push_config();
    
    httpd_resp_set_type(req, "application/json");
    const char* resp = "{\"status\":\"success\"}";
    httpd_resp_send(req, resp, strlen(resp));
    
    return ESP_OK;
}

// [SỬA] Trả về JSON có RSSI
static esp_err_t scan_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Handling GET request for /scan");
    
    scan_wifi();
    
    char *response = malloc(2048);
    if (!response) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    strcpy(response, "{\"wifi\":[");
    for (int i = 0; i < wifi_count; i++) {
        char item[128];
        snprintf(item, sizeof(item), "{\"ssid\":\"%s\",\"rssi\":%d}%s",
                 wifi_list[i], wifi_rssi[i], (i < wifi_count - 1) ? "," : "");
        strcat(response, item);
    }
    strcat(response, "]}");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    free(response);
    
    return ESP_OK;
}

// [SỬA] Giao diện HTML với icon WiFi đúng + thêm lại phần nhập manual
static esp_err_t config_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Handling GET request for /");
   
    char *response = malloc(10240);
    if (!response) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    int len = snprintf(response, 10240,
        "<html><head><title>ESP32 Config</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<meta charset='UTF-8'>"
        "<style>"
        "body{font-family:Arial;background:#f4f4f4;margin:0;padding:20px}"
        ".container{max-width:800px;margin:auto}"
        ".card{background:white;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);margin-bottom:20px;padding:20px}"
        ".card h2{margin-top:0;color:#333;border-bottom:1px solid #ddd;padding-bottom:10px}"
        "label{display:block;margin-bottom:5px;font-weight:bold}"
        "input[type='text'],input[type='password'],input[type='number'],select{width:100%%;padding:8px;margin-bottom:15px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}"
        "input[type='checkbox']{margin-right:10px}"
        "button{background:#007BFF;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;font-size:16px}"
        "button:hover{background:#0056b3}"
        ".btn-save{background:#007BFF;margin-top:10px}"
        "#password-card{display:none}"
        ".wifi-row{display:flex;justify-content:space-between;align-items:center;padding:10px;border-bottom:1px solid #eee;cursor:pointer}"
        ".wifi-row:hover{background:#f0f8ff}"
        ".wifi-signal{font-size:20px;margin-right:10px}"
        ".manual-section{margin-top:15px;padding-top:15px;border-top:2px dashed #ccc}"
        "</style>"
        "<script>"
        "function getSignalIcon(rssi){"
        " if(rssi > -60) return '📶';"
        " if(rssi > -70) return '📶';"
        " if(rssi > -80) return '📡';"
        " return '📡';"
        "}"
        "function getSignalText(rssi){"
        " if(rssi > -60) return 'Excellent';"
        " if(rssi > -70) return 'Good';"
        " if(rssi > -80) return 'Fair';"
        " return 'Weak';"
        "}"
        "function getSignalColor(rssi){"
        " if(rssi > -60) return '#28a745';"
        " if(rssi > -70) return '#ffc107';"
        " if(rssi > -80) return '#fd7e14';"
        " return '#dc3545';"
        "}"
        "function scanWiFi(){"
        " document.getElementById('wifi-container').innerHTML = '<p style=\"padding:10px\">Scanning...</p>';"
        " fetch('/scan').then(r=>r.json()).then(d=>{"
        " let c = document.getElementById('wifi-container');"
        " c.innerHTML = '';"
        " if(d.wifi.length==0){ c.innerHTML='<p style=\"padding:10px\">No WiFi found</p>'; return; }"
        " d.wifi.forEach(w=>{"
        " let div = document.createElement('div');"
        " div.className = 'wifi-row';"
        " let color = getSignalColor(w.rssi);"
        " div.innerHTML = `"
        " <div style='flex:1'>"
        " <span class='wifi-signal'>${getSignalIcon(w.rssi)}</span>"
        " <strong>${w.ssid}</strong>"
        " </div>"
        " <small style='color:${color}'>${getSignalText(w.rssi)} (${w.rssi}dBm)</small>"
        " `;"
        " div.onclick = function(){ selectWifi(w.ssid); };"
        " c.appendChild(div);"
        " });"
        " }).catch(e=>alert('Scan failed: '+e));"
        "}"
        "function selectWifi(ssid){"
        " document.getElementById('selected-ssid').value = ssid;"
        " document.getElementById('wifi-name-display').innerText = ssid;"
        " document.getElementById('password-card').style.display = 'block';"
        " document.getElementById('selected-pass').focus();"
        " document.getElementById('manual-ssid').value = '';"
        " document.getElementById('manual-pass').value = '';"
        "}"
        "function saveSettings(section){"
        " let formData = new URLSearchParams();"
        " formData.append('section', section);"
        " if(section === 'device'){"
        " formData.append('dev_id', document.getElementById('dev_id').value);"
        " formData.append('cycle', document.getElementById('cycle').value);"
        " } else if (section === 'auto'){"
        " formData.append('min', document.getElementById('min').value);"
        " formData.append('max', document.getElementById('max').value);"
        " if(document.getElementById('auto').checked){ formData.append('auto', 'on'); }"
        " }"
        " fetch('/save-settings', {method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:formData})"
        " .then(r=>r.json()).then(d=>{"
        " if(d.status==='success') alert('Settings saved and synced to Firebase!');"
        " else alert('Failed to save');"
        " }).catch(e=>alert('Error: '+e));"
        "}"
        "</script>"
        "</head><body>"
        "<div class='container'>"
        "<h1 style='text-align:center'>ESP32 Configuration</h1>"
        "<form method='POST' action='/config'>"
        "<div class='card'>"
        "<h2>📱 Device Configuration</h2>"
        "<label>Device ID:</label><input type='text' id='dev_id' name='dev_id' value='%s'>"
        "<label>Data Cycle (ms):</label><input type='number' id='cycle' name='cycle' value='%d' min='1000'>"
        "<button type='button' class='btn-save' onclick='saveSettings(\"device\")'>Save Device Settings</button>"
        "</div>"
        "<div class='card'>"
        "<h2>🌱 Automatic Control</h2>"
        "<label>Soil Moisture Min (%%)::</label><input type='number' id='min' name='min' value='%d' min='0' max='100'>"
        "<label>Soil Moisture Max (%%)::</label><input type='number' id='max' name='max' value='%d' min='0' max='100'>"
        "<label><input type='checkbox' id='auto' name='auto' %s> Enable Auto Control</label>"
        "<button type='button' class='btn-save' onclick='saveSettings(\"auto\")'>Save Auto Settings</button>"
        "</div>"
        "<div class='card'>"
        "<h2>📡 WiFi Configuration</h2>"
        "<button type='button' onclick='scanWiFi()'>Scan WiFi Networks</button>"
        "<div id='wifi-container' style='margin-top:10px;border:1px solid #ddd;border-radius:4px;max-height:250px;overflow-y:auto'>"
        "<p style='padding:10px;color:#666'>Click 'Scan WiFi Networks' to find available networks</p>"
        "</div>"
        "<div id='password-card' class='card' style='background:#f9f9f9;border:1px solid #ccc;margin-top:15px'>"
        "<h3>🔒 Connect to: <span id='wifi-name-display' style='color:#007BFF'></span></h3>"
        "<input type='hidden' id='selected-ssid' name='ssid'>"
        "<label>Password:</label>"
        "<input type='password' id='selected-pass' name='pass' placeholder='Enter WiFi Password'>"
        "<button type='submit'>Connect to Network</button>"
        "</div>"
        "<div class='manual-section'>"
        "<h3>✏️ Or Enter WiFi Manually</h3>"
        "<label>WiFi SSID:</label>"
        "<input type='text' id='manual-ssid' name='ssid' placeholder='Enter WiFi name'>"
        "<label>Password:</label>"
        "<input type='password' id='manual-pass' name='pass' placeholder='Enter WiFi password'>"
        "<button type='submit'>Connect Manually</button>"
        "</div>"
        "</div>"
        "</form></div></body></html>",
        device_id, data_cycle_ms, soil_min, soil_max,
        auto_enable ? "checked" : "");
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, len);
    free(response);
    
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Handling POST request for /config");
    
    char* buf = NULL;
    size_t buf_len = req->content_len;
    
    if (buf_len <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    buf = malloc(buf_len + 1);
    if (!buf) {
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
        if (data_cycle_ms < 1000) data_cycle_ms = 1000;
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
    
    if (soil_min >= soil_max) {
        soil_min = 40;
        soil_max = 60;
    }
    
    free(buf);
    
    save_config();
    
    ESP_LOGI(TAG, "Config saved. Starting WiFi transition...");
    
    const char* resp_msg = "<html><head><meta charset='UTF-8'></head><body>"
                           "<h1>✅ Configuration Saved</h1>"
                           "<h2>Connecting to: <b>%s</b></h2>"
                           "<p>The device is now switching to Station mode...</p>"
                           "<p>You can close this page.</p></body></html>";
    char resp_buffer[512];
    snprintf(resp_buffer, sizeof(resp_buffer), resp_msg, wifi_ssid);
    httpd_resp_send(req, resp_buffer, strlen(resp_buffer));
    
    xTaskCreate(wifi_transition_task, "wifi_transition_task", 4096, NULL, 10, NULL);
    
    return ESP_OK;
}

static esp_err_t data_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Handling GET request for /data");
    
    char response[256];
    snprintf(response, sizeof(response),
             "{\"temp\":%.1f,\"hum\":%.1f,\"soil\":%d,\"relay\":%d}",
             temperature, humidity, soil_moisture, relay_state);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    
    return ESP_OK;
}

static void wifi_transition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Transition task started.");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    stop_webserver();
    
    ESP_LOGI(TAG, "Stopping AP mode and cleaning up...");
    ESP_ERROR_CHECK(esp_wifi_stop());
    
    if (s_ap_netif) {
        esp_netif_destroy_default_wifi(s_ap_netif);
        s_ap_netif = NULL;
    }
    
    esp_event_loop_delete_default();
    esp_netif_deinit();
    
    ESP_LOGI(TAG, "Initializing STA mode...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_sta();
    
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA Connected. Starting main tasks.");
        xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 6, NULL);
        xTaskCreate(button_task, "button_task", 2048, NULL, 4, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to connect to STA. Restarting in 10s...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "Transition task finished. Deleting self.");
    vTaskDelete(NULL);
}

static void stop_webserver() {
    if (server) {
        ESP_LOGI(TAG, "Stopping web server...");
        httpd_stop(server);
        server = NULL;
    }
}

static void start_webserver() {
    if (server != NULL) {
        ESP_LOGW(TAG, "Web server already running.");
        return;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.max_open_sockets = 4;
    
    httpd_uri_t config_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = config_get_handler,
    };
    
    httpd_uri_t config_post = {
        .uri = "/config",
        .method = HTTP_POST,
        .handler = config_post_handler,
    };
    
    httpd_uri_t data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler,
    };
    
    httpd_uri_t scan_uri = {
        .uri = "/scan",
        .method = HTTP_GET,
        .handler = scan_get_handler,
    };
   
    httpd_uri_t save_settings_post = {
        .uri = "/save-settings",
        .method = HTTP_POST,
        .handler = save_settings_post_handler,
    };
    
    ESP_LOGI(TAG, "Starting web server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &config_uri);
        httpd_register_uri_handler(server, &config_post);
        httpd_register_uri_handler(server, &data_uri);
        httpd_register_uri_handler(server, &scan_uri);
        httpd_register_uri_handler(server, &save_settings_post);
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
    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_PIN, &config));
    
    load_config();
    
    ESP_LOGI(TAG, "Starting in AP mode for configuration.");
    wifi_init_ap();
    start_webserver();
    
    ESP_LOGI(TAG, "System ready. Access http://192.168.4.1 to configure.");
}