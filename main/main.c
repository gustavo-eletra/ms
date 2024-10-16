
/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "driver/uart.h"
#include "esp_system.h"

#include "utils.h"
#include "mesh_network.h"
#include "uart.h"

#include "msgpack.h"

#define RX_BUF_SIZE 1024
#define ROUTER_SSID "ELETRA350" // SSID da rede Wi-Fi do roteador
#define ROUTER_PWD "01234567"   // senha da rede Wi-Fi do roteador
#define MESH_AP_PASS "mesh_password"
#define MESH_CHANNEL 6
#define MESH_AP_PASS "mesh_password"

static uint8_t MESH_ID[6] = { 0x12, 0x34, 0x56, 0x78, 0x90, 0x91 }; // endereço MAC da rede mesh (mesh ID)
static const char *DEBUG_TAG_Main = "Debug Main";

static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 128

//WiFi router credentials size
#define MAX_SSID_LEN 32  // Comprimento máximo do SSID
#define MAX_PASS_LEN 64  // Comprimento máximo da senha

//extern QueueHandle_t send_message_mesh_q;

void app_main()
{
    func();
    start_uart();
    start_mesh(ROUTER_SSID, ROUTER_PWD, MESH_ID, MESH_AP_PASS);
    
}