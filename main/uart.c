#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "utils.h"
#include "uart.h"

#define UART2_TX_PIN GPIO_NUM_18
#define UART2_RX_PIN GPIO_NUM_5

static const char *TAG = "UART: ";
static const char *DEBUG_TAG = "DEBUG UART TASK: ";

QueueHandle_t send_message_mesh_q;
extern QueueHandle_t rcv_message_mesh_q;

/**
 * @brief Inicializa as UARTs usadas no projeto.
 */
static void uart_init(void)
{
    // Configuração da UART0 (recepção)
    uart_config_t uart_config_0 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_UART0, &uart_config_0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_UART0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_UART0, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configuração da UART2 (envio)
    uart_config_t uart_config_2 = {
        .baud_rate = 9600,  // Ajuste conforme necessário
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_UART2, &uart_config_2));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_UART2, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_UART2, BUF_SIZE_UART2, 0, 0, NULL, 0));
     // Inverts the TX and RX sign
    ESP_ERROR_CHECK(uart_set_line_inverse(UART_NUM_UART2, UART_SIGNAL_TXD_INV  | UART_SIGNAL_RXD_INV));

    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(UART_NUM_UART0);
    uart_flush(UART_NUM_UART2);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "UARTs inicializadas com sucesso");
}

/**
 * @brief Task principal que lida com a recepção, processamento e envio dos dados.
 */
static void uart_task(void *arg)
{
    uint8_t data_buffer[DATA_LENGTH_ABNT] = {0};
    uint8_t send_buffer[TOTAL_LENGTH_ABNT] = {0};
    uint8_t temp_buffer[BUF_SIZE];
    uint8_t recv_buffer[BUF_SIZE_UART2];
    int len = 0;

    while (1)
    {
        // Lê os dados da UART0
        len = uart_read_bytes(UART_NUM_UART0, temp_buffer, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            ESP_LOGI(TAG, "Dados recebidos: %d bytes", len);

            // Limita o tamanho máximo a 64 bytes
            int copy_length = len > DATA_LENGTH_ABNT ? DATA_LENGTH_ABNT : len;
            memcpy(data_buffer, temp_buffer, copy_length);

            // Se o tamanho for menor que 64 bytes, os zeros já estão preenchidos
            ESP_LOGI(TAG, "Buffer de dados preenchido com %d bytes", copy_length);
             printf("Bytes depois de Preenchidos com 0: ");
            for (int i = 0; i < sizeof(data_buffer); i++) {
               printf("%02X ", data_buffer[i]);
            }
            printf("\n\n");

            // Calcula o CRC16 dos 64 bytes
            uint16_t crc = calculate_crc16(data_buffer, DATA_LENGTH_ABNT);
            ESP_LOGI(TAG, "CRC16 calculado: 0x%04X", crc);

            // Prepara o buffer de envio
            memcpy(send_buffer, data_buffer, DATA_LENGTH_ABNT);
            send_buffer[DATA_LENGTH_ABNT] = (uint8_t)(crc & 0xFF);            // Byte menos significativo
            send_buffer[DATA_LENGTH_ABNT + 1] = (uint8_t)((crc >> 8) & 0xFF); // Byte mais significativo

            printf("Bytes preparados para serem enviados: ");
            for (int i = 0; i < sizeof(send_buffer); i++) {
               printf("%02X ", send_buffer[i]);
            }
            printf("\n\n");

            // Envia os 68 bytes pela UART2
            int bytes_sent = uart_write_bytes(UART_NUM_UART2, (const char *)send_buffer, TOTAL_LENGTH_ABNT);
            if (bytes_sent == TOTAL_LENGTH_ABNT)
            {
                ESP_LOGI(TAG, "Dados enviados com sucesso pela UART2");
            }
            else
            {
                ESP_LOGE(TAG, "Erro ao enviar dados pela UART2");
            }
            // Wait for response from UART2
            int len = uart_read_bytes(UART_NUM_2, recv_buffer, BUF_SIZE_UART2, 1000 / portTICK_PERIOD_MS);
            
            if (len > 0) {
                printf("UART2 response: ");
                for (int i = 0; i < 258; i++) {
                    printf("%02X ", recv_buffer[i]);
                }
                printf("Tamanho do Vetor: %d \n", len);
                printf("\n");
            } else {
                printf("No response from UART2\n");
            }

            // Limpa os buffers para a próxima operação
            memset(data_buffer, 0, DATA_LENGTH_ABNT);
            memset(send_buffer, 0, TOTAL_LENGTH_ABNT);
            memset(temp_buffer, 0, BUF_SIZE);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Pequeno atraso para evitar uso excessivo da CPU
    }
}

/**
 * @brief task to receive commands from the PC's UART and add them to the Queue that will be sent to the Mesh network
 **/
void uart_rcv_task()
{
    uint8_t recv_buffer[BUF_SIZE];
    for(;;)
    {
        // Read data from UART0
        int len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_UART0, (size_t*)&len));
        len = uart_read_bytes(UART_NUM_UART0, recv_buffer, len, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            recv_buffer[len] = '\0'; // Adiciona o terminador nulo ao final da string

            char *mac_address = strtok((char *)recv_buffer, "_");
            char *protocol = strtok(NULL, "_");
            char *payload = strtok(NULL, "_");
        
            if (mac_address && protocol && payload) 
            {
                message_t cmd;
                strncpy(cmd.mac_address, mac_address, sizeof(cmd.mac_address) - 1);
                strncpy(cmd.protocol, protocol, sizeof(cmd.protocol) - 1);
                strncpy(cmd.payload, payload, sizeof(cmd.payload) - 1);
                cmd.mac_address[sizeof(cmd.mac_address) - 1] = '\0';
                cmd.protocol[sizeof(cmd.protocol) - 1] = '\0';
                cmd.payload[sizeof(cmd.payload) - 1] = '\0';
                cmd.payload_size = sizeof(cmd.payload);

                if (xQueueSend(send_message_mesh_q, &cmd, pdMS_TO_TICKS(1000)) != pdTRUE) {
                    ESP_LOGW(DEBUG_TAG, "Queue full, command dropped");
                }
            } else {
                ESP_LOGE(DEBUG_TAG, "Formato de comando invalido");
            }
        }//if (len > 0)
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }//for(;;)
}//void uart_rcv_task()

void nock_loose_task()
{
    message_t cmd;
    message_t answ_cmd;

    uint8_t data_buffer[DATA_LENGTH_ABNT] = {0};
    uint8_t send_buffer[TOTAL_LENGTH_ABNT] = {0};
    //uint8_t temp_buffer[BUF_SIZE];
    uint8_t recv_buffer[BUF_SIZE_UART2];

    int len = 0;
    for(;;)
    {
        if (xQueueReceive(rcv_message_mesh_q, &cmd, (TickType_t)portMAX_DELAY)) 
        {
          // Save data size
          len = cmd.payload_size;//sizeof(cmd.payload);
          int protocol_number = identify_protocol(cmd.protocol);
          memcpy(answ_cmd.mac_address, cmd.mac_address, sizeof(answ_cmd.mac_address));//copia endereco de envio
          
          switch(protocol_number)
          {
            case 1: //ABNT
                ESP_LOGI(TAG, "Dados recebidos: %d bytes", len);

                // Limita o tamanho máximo a 64 bytes
                int copy_length = len > DATA_LENGTH_ABNT ? DATA_LENGTH_ABNT : len;
                //memcpy(data_buffer, cmd.payload, copy_length);
                
                // Converte o payload de string hexadecimal para bytes reais
                if (hexstr_to_bytes(cmd.payload, data_buffer, DATA_LENGTH_ABNT) == -1) {
                    ESP_LOGE(TAG, "Erro ao converter payload hexadecimal em bytes");
                    break;
                }

                // Se o tamanho for menor que 64 bytes, os zeros já estão preenchidos
                ESP_LOGI(TAG, "Buffer de dados preenchido com %d bytes", copy_length);
                printf("Bytes depois de Preenchidos com 0: ");
                for (int i = 0; i < sizeof(data_buffer); i++) {
                    printf("%02X ", data_buffer[i]);
                }
                printf("\n\n");

                // Calcula o CRC16 dos 64 bytes
                uint16_t crc = calculate_crc16(data_buffer, DATA_LENGTH_ABNT);
                ESP_LOGI(TAG, "CRC16 calculado: 0x%04X", crc);

                // Prepara o buffer de envio
                memcpy(send_buffer, data_buffer, DATA_LENGTH_ABNT);
                send_buffer[DATA_LENGTH_ABNT] = (uint8_t)(crc & 0xFF);            // Byte menos significativo
                send_buffer[DATA_LENGTH_ABNT + 1] = (uint8_t)((crc >> 8) & 0xFF); // Byte mais significativo

                printf("Bytes preparados para serem enviados: ");
                for (int i = 0; i < sizeof(send_buffer); i++) {
                printf("%02X ", send_buffer[i]);
                }
                printf("\n\n");

                // Envia os 68 bytes pela UART2
                int bytes_sent = uart_write_bytes(UART_NUM_UART2, (const char *)send_buffer, TOTAL_LENGTH_ABNT);
                if (bytes_sent == TOTAL_LENGTH_ABNT)
                {
                    ESP_LOGI(TAG, "Dados enviados com sucesso pela UART2");
                }
                else
                {
                    ESP_LOGE(TAG, "Erro ao enviar dados pela UART2");
                }
                // Wait for response from UART2
                int len = uart_read_bytes(UART_NUM_2, recv_buffer, BUF_SIZE_UART2, 1000 / portTICK_PERIOD_MS);
                
                if (len > 0) {
                    printf("UART2 response: ");
                    for (int i = 0; i < 258; i++) {
                        printf("%02X ", recv_buffer[i]);
                    }
                    printf("Tamanho do Vetor: %d \n", len);
                    printf("\n");
                    strcpy(answ_cmd.protocol, "ANSW");
                    // Converte os bytes recebidos para uma string hexadecimal
                    bytes_to_hexstr(recv_buffer, answ_cmd.payload, len);
                    answ_cmd.payload_size = len;  // Atualiza o tamanho do payload
                    // memcpy(answ_cmd.payload, recv_buffer, len);
                    // answ_cmd.payload_size = len;
                    printf("The payload size is %d, UART2 response in HEX: %s\n", answ_cmd.payload_size, answ_cmd.payload);

                    if (xQueueSend(send_message_mesh_q, &answ_cmd, pdMS_TO_TICKS(1000)) != pdTRUE) {
                    ESP_LOGW(DEBUG_TAG, "Queue full, command dropped");
                }


                } else {
                    printf("No response from UART2\n");
                }

                // Limpa os buffers para a próxima operação
                memset(data_buffer, 0, DATA_LENGTH_ABNT);
                memset(send_buffer, 0, TOTAL_LENGTH_ABNT);
                //memset(temp_buffer, 0, BUF_SIZE);
             
            break;
            case 2: //DLMS
                ESP_LOGE(TAG, "DLMS not defined yet");
            break;
            case 3: //ANSW
                uint8_t data_buffer_ABNT[DATA_LENGTH_ABNT_ANSW] = {0};
                ESP_LOGE(TAG, "The received command:");
                // Converte o payload de string hexadecimal para bytes reais
                if (hexstr_to_bytes(cmd.payload, data_buffer_ABNT, DATA_LENGTH_ABNT_ANSW) == -1) {
                    ESP_LOGE(TAG, "Erro ao converter payload hexadecimal em bytes");
                    break;
                }
                 for (int i = 0; i < sizeof(data_buffer_ABNT[i]); i++) {
                    printf("%02X ", data_buffer_ABNT[i]);
                }
            break;
            default:
                ESP_LOGE(TAG, "Protocol not defined yet");
            break;
          }
            
        }//if (xQueueReceive(rcv_message_mesh_q, &cmd, (TickType_t)portMAX_DELAY)) 

    }//for(;;)
}//void nock_loose_task()

/**
 * @brief Task to initialize UART periferical and create the principal task
 */
void start_uart()
{
    send_message_mesh_q = xQueueCreate(QUEUE_SIZE, sizeof(message_t));
    if (send_message_mesh_q == NULL)
        ESP_LOGE(DEBUG_TAG, "Failed to create queue");
    else ESP_LOGE(DEBUG_TAG, "Success in create queue");

    rcv_message_mesh_q = xQueueCreate(QUEUE_SIZE, sizeof(message_t));
    if (send_message_mesh_q == NULL)
        ESP_LOGE(TAG, "Failed to create queue");
    else ESP_LOGE(TAG, "Success in create queue");
    
    uart_init();
    //xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreatePinnedToCore(uart_rcv_task, "uart_rcv_task", 5120, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(nock_loose_task, "nock_loose_task", 5120, NULL, 10, NULL, 1);
}