#ifndef __CAN_API_H
#define __CAN_API_H

#include <stdint.h>

/**
 Please don't edit this file unless you
 know what you do
 */

enum { MAX_SENSORS_NUMBER = 10 };
enum { MAX_NAME_LENGTH    = 64 };
enum { MAX_JSON_LENGTH    = 256 };
enum { MAX_BUFFER_LENGTH  = 64 / sizeof(uint32_t) };
enum { IDENTIFICATOR      = 2  }; // Warning: hardcode!

struct cbck_s {
    char sens_name[64];
    int (*get)(uint32_t* buf);
};

struct cbcks_s {
    // Please don't attempt to access fields
    // of this structure directly
    void (*setup)(void);
    void (*loop)(void);

    int sens_num;
    uint32_t sens_buf[MAX_BUFFER_LENGTH];
    struct cbck_s sens_cbcks[MAX_SENSORS_NUMBER];
};

void can_do_setup(void (*setup_routine)(void));

void can_do_loop(void (*loop_routine)(void));

void can_add_get(const char* sensor_name,
                 int (*get_routine)(uint32_t* buf));

void can_get(char* result);

#endif
