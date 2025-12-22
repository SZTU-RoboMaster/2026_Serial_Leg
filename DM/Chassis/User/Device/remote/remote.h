#ifndef _REMOTE_H
#define _REMOTE_H

#include "stm32h7xx.h"                  // Device header

#define SBUS_RX_BUF_NUM 18u

#define RC_CH_VALUE_OFFSET 1024u

typedef struct {
    __packed struct {
        int16_t ch[5];
        char s[2];
    } rc;
    __packed struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct {
        uint16_t v;
    } key;

} RC_ctrl_t;

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

extern RC_ctrl_t remote_ctrl;

#endif
