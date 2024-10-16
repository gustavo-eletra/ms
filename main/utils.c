
#include "utils.h"


//QueueHandle_t send_message_mesh_q;

/**
 * @brief Calcula o CRC16 dos dados fornecidos usando o polinômio 0x8005.
 *
 * @param data Ponteiro para os dados de entrada.
 * @param length Tamanho dos dados de entrada.
 * @return Valor CRC16 calculado.
 */
 uint16_t calculate_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0x0000; // CRC inicializado com 0
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    printf("CRC Calculado dentro da funcao: ");
    printf("%04X ", crc);
    printf("\n");
    return crc;
}

/**
 * @brief Tranforma uma String MAC address em array de byte d0:ef:76:44:ec:18 -> D0 EF 76 44 EC 18 
 *
 * @param mac_str Ponteiro para os dados de entrada.
 * @param mac_bytes Vetor aonde será armazenado esses bytes
 *
 */
void mac_str_to_bytes(const char *mac_str, uint8_t *mac_bytes) {
    for (int i = 0; i < 6; ++i) {
        sscanf(mac_str + 3 * i, "%2hhx", &mac_bytes[i]);
    }
}

void mac_bytes_to_string(uint8_t mac[6], char *mac_str) {
    sprintf(mac_str, MACSTR, MAC2STR(mac));
}

// Função para identificar o protocolo e retornar o número correspondente
int identify_protocol(const char *protocol) {
    if (strcmp(protocol, "ABNT") == 0) {
        return 1;
    } else if (strcmp(protocol, "DLMS") == 0) {
        return 2;
    } else if (strcmp(protocol, "ANSW") == 0) {
        return 3;
    } else if (strcmp(protocol, "ABNTUO") == 0) {
        return 4;
    } else if (strcmp(protocol, "PIMA") == 0) {
        return 5;
    }
    return 0; // Protocolo desconhecido
}

int hexstr_to_bytes(const char *hexstr, uint8_t *bytes, size_t max_len) {
    size_t hexstr_len = strlen(hexstr);
    size_t bytes_len = hexstr_len / 2;

    if (bytes_len > max_len) {
        return -1; // Erro: tamanho do buffer insuficiente
    }

    for (size_t i = 0; i < bytes_len; i++) {
        sscanf(&hexstr[i * 2], "%2hhx", &bytes[i]);
    }

    return bytes_len;
}

// Função para converter um buffer de bytes em uma string hexadecimal
void bytes_to_hexstr(const uint8_t *bytes, char *hexstr, size_t length) {
    for (size_t i = 0; i < length; i++) {
        sprintf(hexstr + i * 2, "%02X", bytes[i]);
    }
    hexstr[length * 2] = '\0'; // Adiciona o terminador nulo ao final da string
}