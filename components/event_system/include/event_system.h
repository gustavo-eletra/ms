#pragma once
#include <stdint.h>

typedef int (*event_callback)(void *data);

typedef struct 
{
    event_callback *event_callbacks;
    uint8_t *event_marker;
}EventStorage_t;

EventStorage_t *event_system_init(uint8_t event_number);
int event_system_register_event(event_callback *callback, int event_marker);
void event_system_fsm(EventStorage_t *event_storage);
void event_system_queue(EventStorage_t *event_storage);
void event_system_circuar_queue(EventStorage_t *event_storage);