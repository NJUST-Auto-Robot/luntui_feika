/*
 * @Author: skybase
 * @Date: 2025-01-13 17:12:42
 * @LastEditors: skybase
 * @LastEditTime: 2025-02-15 12:34:43
 * @Description:  ?(???)??
 * @FilePath: \luntui_feika\CODE\bsp_log.h
 */
#ifndef _BSP_LOG_H_
#define _BSP_LOG_H_

#include <stdio.h>
#include "string.h"
#include "zf_common_headfile.h"

#define COLR_LOG_MODE 1

extern char _debug[1024];

#if COLR_LOG_MODE
// ��ʾģʽö��
typedef enum
{
    DISPLAY_MODE_DEFAULT = 0,       // Ĭ��ֵ
    DISPLAY_MODE_BOLD = 1,          // ����
    DISPLAY_MODE_UNDERLINE = 4,     // �»���
    DISPLAY_MODE_BLINK = 5,         // ��˸
    DISPLAY_MODE_REVERSE = 7,       // ����
    DISPLAY_MODE_NORMAL = 22,       // �Ǵ���
    DISPLAY_MODE_NO_UNDERLINE = 24, // ���»���
    DISPLAY_MODE_NO_BLINK = 25,     // ����˸
    DISPLAY_MODE_NO_REVERSE = 27    // �Ƿ���
} DISPLAY_MODE;

// ǰ��ɫö��
typedef enum
{
    FOREGROUND_COLOR_BLACK = 30,   // ��ɫ
    FOREGROUND_COLOR_RED = 31,     // ��ɫ
    FOREGROUND_COLOR_GREEN = 32,   // ��ɫ
    FOREGROUND_COLOR_YELLOW = 33,  // ��ɫ
    FOREGROUND_COLOR_BLUE = 34,    // ��ɫ
    FOREGROUND_COLOR_MAGENTA = 35, // ���
    FOREGROUND_COLOR_CYAN = 36,    // ��ɫ
    FOREGROUND_COLOR_WHITE = 37    // ��ɫ
} FOREGROUND_COLOR;

// ����ɫö��
typedef enum
{
    BACKGROUND_COLOR_BLACK = 40,   // ��ɫ
    BACKGROUND_COLOR_RED = 41,     // ��ɫ
    BACKGROUND_COLOR_GREEN = 42,   // ��ɫ
    BACKGROUND_COLOR_YELLOW = 43,  // ��ɫ
    BACKGROUND_COLOR_BLUE = 44,    // ��ɫ
    BACKGROUND_COLOR_MAGENTA = 45, // ���
    BACKGROUND_COLOR_CYAN = 46,    // ��ɫ
    BACKGROUND_COLOR_WHITE = 47    // ��ɫ
} BACKGROUND_COLOR;

extern DISPLAY_MODE dis_mode;
extern FOREGROUND_COLOR fwd_clor;
extern BACKGROUND_COLOR bak_clor;

#define LOG_PRINT(...) (sprintf(_debug, "\033[%d;%d;%dm", dis_mode, fwd_clor, bak_clor), \
                        debug_transmit((uint8_t *)_debug,                                \
                                       (strlen(_debug) + sprintf(_debug + strlen(_debug), __VA_ARGS__))))

#else
#define LOG_PRINT(...) (debug_transmit((uint8_t *)_debug, sprintf(_debug, __VA_ARGS__)))

#endif

/**
 * @brief ��־���ܵ�ԭʼ����,��Ҫ�û��Լ��Զ���
 *
 */
void debug_transmit(uint8 *data, uint16 len);


/**
 * ������־���,����ʹ����Щ���������־
 * @note �����ʹ�ü�����־���ʹ��Ĭ����־�ӿ� LOG_PRINT(...) ��ʹ��Ĭ�ϵĻ�����ɫ
 */
#if COLR_LOG_MODE
// information level
#define LOGINFO(...) (dis_mode = DISPLAY_MODE_DEFAULT, fwd_clor = FOREGROUND_COLOR_BLACK, bak_clor = BACKGROUND_COLOR_WHITE, LOG_PRINT("I:" __VA_ARGS__))

// warning level
#define LOGWARNING(...) (dis_mode = DISPLAY_MODE_DEFAULT, fwd_clor = FOREGROUND_COLOR_BLACK, bak_clor = BACKGROUND_COLOR_YELLOW, LOG_PRINT("W:"__VA_ARGS__))

// error level
#define LOGERROR(...) (dis_mode = DISPLAY_MODE_DEFAULT, fwd_clor = FOREGROUND_COLOR_BLACK, bak_clor = BACKGROUND_COLOR_RED, LOG_PRINT("E:"__VA_ARGS__))

#else

#define LOGINFO(...) LOG_PRINT("I:" __VA_ARGS__)
#define LOGWARNING(...) (LOG_PRINT("W:"__VA_ARGS__))
#define LOGERROR(...) (LOG_PRINT("E:"__VA_ARGS__))


#endif

/*�û��Զ���LOG����,���������ÿ�Ļ������ٴ������������־��ӡ*/

#endif
