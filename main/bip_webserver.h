#ifndef BIP_WEBSERVER_H
#define BIP_WEBSERVER_H

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t start_bip_webserver(void);
void stop_bip_webserver(void);
bool is_webserver_running(void);

#endif // BIP_WEBSERVER_H
