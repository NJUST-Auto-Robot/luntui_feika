/*
 * @Author: skybase
 * @Date: 2024-12-18 21:43:51
 * @LastEditors: skybase
 * @LastEditTime: 2025-01-17 04:37:43
 * @Description:  �?(◠ڼ◠)ᕗ�?
 * @FilePath: \BSP\Math\m_math.h
 */
#ifndef M_MATH_H
#define M_MATH_H

#include "stdio.h"
#include "math.h"
#include "string.h"

float remap(float x, float y, float x1, float y1, float value);
float remap_stage(float x, float y, float x1, float y1, float stage, float value);
float fast_sqrt(float number);
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

typedef struct mathtype
{
    float init_val;
    float target_val;
    float now_val;

} mathtype;

typedef void (*mathCalUpdate)(mathtype *, float, float);

void mathtypeSetVal(mathtype *obj, float init_val, float target_val);
void mathtypeUpdate(mathtype *obj, mathCalUpdate call, float timebase, float spd);

void linearCalculation(mathtype *obj, float timebase, float spd);

#endif
