#pragma once
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "esp_mac.h"


#define QUEUE_SIZE 10

typedef struct {
    char mac_address[18];
    char protocol[6];
    char payload[1024];
    uint16_t payload_size;
} message_t;

uint16_t calculate_crc16(const uint8_t *data, size_t length);
void mac_str_to_bytes(const char *mac_str, uint8_t *mac_bytes);
void mac_bytes_to_string(uint8_t mac[6], char *mac_str);
int identify_protocol(const char *protocol) ;
int hexstr_to_bytes(const char *hexstr, uint8_t *bytes, size_t max_len);
void bytes_to_hexstr(const uint8_t *bytes, char *hexstr, size_t length);