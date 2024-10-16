#pragma once
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "utils.h"

#define UART_NUM_UART0      UART_NUM_0   // UART padrão para recepção de dados
#define UART_NUM_UART2      UART_NUM_2   // UART para envio de dados processados

#define BUF_SIZE            1024         // Tamanho do buffer de recepção
#define BUF_SIZE_UART2      1024         // Tamanho do buffer de envio
#define DATA_LENGTH_ABNT    64           // Tamanho esperado dos dados
#define TOTAL_LENGTH_ABNT   66           // Tamanho total após adicionar o CRC16
#define DATA_LENGTH_ABNT_ANSW    258           // Tamanho esperado dos dados

// Function prototypes
static void uart_init(void);
static void uart_task(void *arg);
void start_uart(void);
void uart_rcv_task(void);
void nock_loose_task(void);
