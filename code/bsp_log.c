/*
 * @Author: skybase
 * @Date: 2025-01-13 17:12:42
 * @LastEditors: skybase
 * @LastEditTime: 2025-02-15 18:03:34
 * @Description:  ?(???)?? 
 * @FilePath: \25_feika_luntui\code\bsp_log.c
 */
#include "bsp_log.h"
#include "zf_common_headfile.h"

char _debug[1024];

DISPLAY_MODE dis_mode = DISPLAY_MODE_DEFAULT; // 显示模式(默认)
FOREGROUND_COLOR fwd_clor = FOREGROUND_COLOR_BLACK; // 字体色（黑色）
BACKGROUND_COLOR bak_clor = BACKGROUND_COLOR_WHITE; // 背景色（白色）

/**
 * @brief 用户自定义的串口发送
 * 
 * @param data 
 * @param len 
 */
void debug_transmit(uint8 *data, uint16 len)
{
    uart_write_buffer(UART_2, data, len);
}

// void Float2Str(char *str, float va)
// {
//     int flag = va < 0;
//     int head = (int)va;
//     int point = (int)((va - head) * 1000);
//     head = abs(head);
//     point = abs(point);
//     if (flag)
//         sprintf(str, "-%d.%d", head, point);
//     else
//         sprintf(str, "%d.%d", head, point);
// }
