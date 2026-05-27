#include "wifi.h"


static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    printf("Event nr: %ld!\n", event_id);
}

void wifi_init_softap() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}

httpd_handle_t start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    printf("Starting server on port %d\n", config.server_port);
    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    printf("httpd_start returned: %d\n", ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server");
        return NULL;
    }
    return server;
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, 
        "<!DOCTYPE html>"
        "<html>"
        "<body style='background:gray'>"
        "   <canvas id='map' width='250' height='250' style='border:2px solid red; width: 1000px; height: 1000px; image-rendering: pixelated; margin: auto;'></canvas>"
        "   <script>"
        "   async function updateMap() {"
        "       const response = await fetch('/map');"
        "       const buffer = await response.arrayBuffer();"
        "       const data = new Uint8Array(buffer);"
        "       const canvas = document.getElementById('map');"
        "       const ctx = canvas.getContext('2d');"
        "       const imageData = ctx.createImageData(250, 250);"
        "       function mapColor(i) {"
        "           let values = [255, 255, 255, 255];"
        "           if (data[i] == 1) values = [255, 0, 0, 255];"
        "           else if (data[i] == 2) values = [0, 0, 255, 255];"
        "           else if (data[i] == 3) values = [255, 0, 255, 255];"
        "           else if (data[i] == 4) values = [0, 255, 0, 255];"
        "           return values;"
        "       }"
        "       for (let i = 0; i < 250 * 250; i++) {" // MAP_SIZE 250
        "           const v = mapColor(i);"
        "           imageData.data[i*4]   = v[0];"
        "           imageData.data[i*4+1] = v[1];"
        "           imageData.data[i*4+2] = v[2];"
        "           imageData.data[i*4+3] = v[3];"
        "       }"
        "       ctx.putImageData(imageData, 0, 0);"
        "   }"
        "   setInterval(updateMap, 1000);"
        "   updateMap();"
        "   </script>"
        "</body>"
        "</html>",
        HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t map_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_send(req, (char*)map, MAP_SIZE * MAP_SIZE);
    return ESP_OK;
}

static const httpd_uri_t map_uri = {
    .uri        = "/map",
    .method     = HTTP_GET,
    .handler    = map_handler,
    .user_ctx   = NULL
};

static const httpd_uri_t index_uri = {
    .uri        = "/",
    .method     = HTTP_GET,
    .handler    = index_handler,
    .user_ctx   = NULL
};

void handle_server_init() {
    wifi_init_softap();
    httpd_handle_t server = start_webserver();
    printf("server handle: %p\n", server);  // if NULL, server failed to start

    if (server) {
        esp_err_t r1 = httpd_register_uri_handler(server, &index_uri);
        esp_err_t r2 = httpd_register_uri_handler(server, &map_uri);
        printf("index handler registered: %d\n", r1);
        printf("map handler registered: %d\n", r2);
    }
}