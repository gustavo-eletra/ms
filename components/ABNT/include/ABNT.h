#pragma once
#include <stdint.h>

typedef struct
{
    uint8_t command;
    uint8_t data_size;
    uint8_t data[65];
} ABNTCommand;

void abnt_set_data(ABNTCommand *abnt_command);
int abnt_send_data(ABNTCommand *abnt_command);
int abnt_open_session();