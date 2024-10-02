// #include <stdio.h>
// #include "pico/cyw43_arch.h"
// #include "pico/stdlib.h"
// #include "lwip/err.h"
// #include "lwip/sockets.h"
// #include "lwip/dns.h"
// #include "lwip/netdb.h"
// #include "lwip/api.h"



// const char *ssid = "MayTheWifiBeWithYou";
// const char *password = "brighthippo726";

// const char *NOAA_API_URL = "api.weather.gov";
// const char *NOAA_API_PATH = "/stations/KAKR/observations/latest";

// #define request = "GET /" NOAA_API_PATH " HTTP/1.1\r\n"\
//                       "HOST: " NOAA_API_URL "\r\n"\
//                       "Connection: close\r\n"

// void connect_to_wifi() {
//     if(cyw43_arch_init()){
//         printf("Wifi init failed");
//         return;
//     }


//     cyw43_arch_enable_sta_mode();
//     printf("Connectin to wifi...\n");
//     if(cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
//         pringf("Connection failed");
//         return;
//     }
//     printf("Conection successful");
// }

// void fetch_weather_data() {
//     //struct sockaddr_in server_addr;
//     struct hostent *server;
// }