#ifndef WIFI_H
#define WIFI_H

#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "config.h"


extern uint8_t map[MAP_SIZE][MAP_SIZE];
extern uint8_t map_tree[MAP_SIZE][MAP_SIZE];

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t map_handler(httpd_req_t *req);
static esp_err_t restart_slam_handler(httpd_req_t *req);
void wifi_init_softap();
httpd_handle_t start_webserver();
void handle_server_init();

#endif