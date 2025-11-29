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

// --- Cấu hình chung ---
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Cấu hình AP Mode (WiFi do ESP phát ra)
#define CONFIG_AP_SSID "ESP32_Config"
#define CONFIG_AP_PASSWORD "12345678" // Mật khẩu để vào cấu hình

// Cấu hình NVS (Nơi lưu SSID/Pass)
#define NVS_NAMESPACE "storage"
#define NVS_WIFI_SSID "ssid"
#define NVS_WIFI_PASS "pass"

// --- Biến Global ---
static const char *TAG = "WIFI_MGR";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// Biến lưu trữ cấu hình
static char wifi_ssid[32] = {0};
static char wifi_password[64] = {0};

// Biến quét WiFi
static char wifi_list[10][32];
static int wifi_rssi[10];
static int wifi_count = 0;

// Handles
httpd_handle_t server = NULL;
static esp_netif_t *s_ap_netif = NULL;

// --- Khai báo hàm ---
static void start_webserver();
static void stop_webserver();
static void wifi_transition_task(void *pvParameters);
static esp_err_t scan_wifi();
static esp_err_t save_wifi_credentials();

// --- Xử lý sự kiện WiFi (Event Handler) ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, EXAMPLE_ESP_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --- Đọc/Ghi NVS (Lưu mật khẩu WiFi) ---
static esp_err_t load_wifi_credentials() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return err;
    
    size_t len = sizeof(wifi_ssid);
    if (nvs_get_str(nvs_handle, NVS_WIFI_SSID, wifi_ssid, &len) == ESP_OK) {
        len = sizeof(wifi_password);
        nvs_get_str(nvs_handle, NVS_WIFI_PASS, wifi_password, &len);
        ESP_LOGI(TAG, "Loaded SSID from NVS: %s", wifi_ssid);
    } else {
        ESP_LOGI(TAG, "No SSID saved in NVS");
    }
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

static esp_err_t save_wifi_credentials() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;
    
    nvs_set_str(nvs_handle, NVS_WIFI_SSID, wifi_ssid);
    nvs_set_str(nvs_handle, NVS_WIFI_PASS, wifi_password);
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi Credentials Saved to NVS");
    return err;
}

// --- Khởi tạo WiFi Station (Kết nối mạng) ---
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
    
    ESP_LOGI(TAG, "Connecting to SSID:%s...", wifi_ssid);
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", wifi_ssid);
        // --- CHỖ NÀY ĐỂ BẠN THÊM CODE LOGIC CỦA BẠN SAU NÀY ---
        // Ví dụ: xTaskCreate(my_application_task, ...);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", wifi_ssid);
        // Có thể thêm logic: Nếu fail thì quay lại AP Mode?
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// --- Khởi tạo WiFi AP (Phát mạng config) ---
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
    
    ESP_LOGI(TAG, "AP Started. SSID: %s, Pass: %s, IP: 192.168.4.1", CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
}

// --- Quét WiFi ---
static esp_err_t scan_wifi() {
    wifi_scan_config_t scan_config = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 150,
    };
    
    esp_wifi_scan_start(&scan_config, true);
    
    uint16_t ap_count = 10;
    wifi_ap_record_t ap_records[10];
    esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    
    wifi_count = ap_count > 10 ? 10 : ap_count;
    for (int i = 0; i < wifi_count; i++) {
        strncpy(wifi_list[i], (char *)ap_records[i].ssid, sizeof(wifi_list[i]));
        wifi_list[i][31] = '\0';
        wifi_rssi[i] = ap_records[i].rssi;
    }
    return ESP_OK;
}

// --- Web Server Handlers ---

// 1. API trả về danh sách WiFi (JSON)
static esp_err_t scan_get_handler(httpd_req_t *req) {
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

// 2. Giao diện cấu hình (HTML thuần)
static esp_err_t config_get_handler(httpd_req_t *req) {
    char *response = malloc(8192);
    if (!response) return ESP_FAIL;
    
    // Giao diện HTML đã được tinh gọn chỉ còn phần WiFi
    int len = snprintf(response, 8192,
        "<html><head><title>WiFi Manager</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;background:#f4f4f4;padding:20px;max-width:600px;margin:auto}"
        ".card{background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);margin-bottom:20px}"
        "input{width:100%%;padding:10px;margin:10px 0;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}"
        "button{background:#007BFF;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;width:100%%}"
        ".wifi-row{padding:10px;border-bottom:1px solid #eee;cursor:pointer;display:flex;justify-content:space-between}"
        ".wifi-row:hover{background:#eef}"
        "</style>"
        "<script>"
        "function scanWiFi(){"
        " document.getElementById('list').innerHTML = 'Scanning...';"
        " fetch('/scan').then(r=>r.json()).then(d=>{"
        " let c = document.getElementById('list'); c.innerHTML = '';"
        " d.wifi.forEach(w=>{"
        " let div = document.createElement('div');"
        " div.className = 'wifi-row';"
        " div.innerHTML = `<b>${w.ssid}</b> <small>(${w.rssi}dBm)</small>`;"
        " div.onclick = function(){ document.getElementById('ssid').value = w.ssid; document.getElementById('pass').focus(); };"
        " c.appendChild(div);"
        " });"
        " });"
        "}"
        "</script>"
        "</head><body>"
        "<div class='card'>"
        "<h2>WiFi Configuration</h2>"
        "<button onclick='scanWiFi()'>Scan Networks</button>"
        "<div id='list' style='max-height:200px;overflow-y:auto;margin-top:10px'></div>"
        "<form method='POST' action='/config' style='margin-top:20px'>"
        "<label>SSID:</label><input type='text' id='ssid' name='ssid' placeholder='WiFi Name'>"
        "<label>Password:</label><input type='password' id='pass' name='pass' placeholder='Password'>"
        "<button type='submit'>Save & Connect</button>"
        "</form>"
        "</div></body></html>");
    
    httpd_resp_send(req, response, len);
    free(response);
    return ESP_OK;
}

// 3. Xử lý lưu cấu hình (POST)
static esp_err_t config_post_handler(httpd_req_t *req) {
    char* buf = malloc(req->content_len + 1);
    int ret = httpd_req_recv(req, buf, req->content_len);
    if (ret <= 0) { free(buf); return ESP_FAIL; }
    buf[ret] = '\0';
    
    char param[64];
    if (httpd_query_key_value(buf, "ssid", param, sizeof(param)) == ESP_OK) {
        strncpy(wifi_ssid, param, sizeof(wifi_ssid) - 1);
    }
    if (httpd_query_key_value(buf, "pass", param, sizeof(param)) == ESP_OK) {
        strncpy(wifi_password, param, sizeof(wifi_password) - 1);
    }
    free(buf);
    
    save_wifi_credentials(); // Lưu vào NVS
    
    httpd_resp_send(req, "Saved. Connecting...", HTTPD_RESP_USE_STRLEN);
    
    // Tạo task để chuyển chế độ sang Station
    xTaskCreate(wifi_transition_task, "wifi_transition", 4096, NULL, 5, NULL);
    
    return ESP_OK;
}

// Task chuyển đổi từ AP -> STA
static void wifi_transition_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    stop_webserver();
    
    ESP_LOGI(TAG, "Stopping AP...");
    esp_wifi_stop();
    if (s_ap_netif) {
        esp_netif_destroy_default_wifi(s_ap_netif);
        s_ap_netif = NULL;
    }
    esp_event_loop_delete_default();
    esp_netif_deinit();
    
    ESP_LOGI(TAG, "Starting Station Mode...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_sta(); // Thử kết nối với WiFi mới
    
    vTaskDelete(NULL);
}

static void stop_webserver() {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
}

static void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    httpd_uri_t uri_get = { .uri = "/", .method = HTTP_GET, .handler = config_get_handler };
    httpd_uri_t uri_scan = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler };
    httpd_uri_t uri_post = { .uri = "/config", .method = HTTP_POST, .handler = config_post_handler };
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_scan);
        httpd_register_uri_handler(server, &uri_post);
    }
}

void app_main(void) {
    // Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    load_wifi_credentials();
    
    // Luôn khởi động AP trước để người dùng có thể cấu hình lại nếu cần
    // Nếu muốn tự động kết nối ngay nếu có pass, có thể thêm logic kiểm tra wifi_ssid ở đây
    ESP_LOGI(TAG, "Starting WiFi Manager...");
    wifi_init_ap();
    start_webserver();
    
    ESP_LOGI(TAG, "Ready. Connect to WiFi 'ESP32_Config' to configure.");
}