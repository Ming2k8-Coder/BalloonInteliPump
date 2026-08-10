#ifndef BIP_WEBSERVER_H
#define BIP_WEBSERVER_H

#include "bip_config.h"
#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t start_web_server();
void stop_web_server();

#endif // BIP_WEBSERVER_H
