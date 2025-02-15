/*
 * @Author: skybase
 * @Date: 2024-11-06 23:28:30
 * @LastEditors: skybase
 * @LastEditTime: 2025-01-14 09:20:23
 * @Description:  ?(???)?? 
 * @FilePath: \luntui\BSP\Math\m_math.c
 */
#include "m_math.h"

float remap(float x, float y, float x1, float y1, float value)
{
    return x1 + (value - x) * (y1 - x1) / (y - x);
}

/**
 * @brief �Դ���ǰ�����ݽ��н׶ηּ����ֽ׶�ӳ�䵽rmap�������
 * @param stage �׶θ���
 */
float remap_stage(float x, float y, float x1, float y1, float stage, float value)
{
    float inputStageSize = (y - x) / stage;
    float outputStageSize = (y1 - x1) / stage;

    int currentStage = (int)(1.0 * (value - x) / inputStageSize);
    if (currentStage < 0)
        currentStage = 0; // Clamp to stage 0 if below range
    else if (currentStage >= stage)
        currentStage = stage - 1; // Clamp to last stage if above range

    float stageInputStart = x + currentStage * inputStageSize;
    float stageOutputStart = x1 + currentStage * outputStageSize;

    float k = outputStageSize / inputStageSize;
    return stageOutputStart + k * (value - stageInputStart);
}

float fast_sqrt(float number)
{
    long i;
    float x, y;
    const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);

    y = *(float *)&i;
    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));
    return number * y;
}

// >�ú���Ϊ�û��ӿ�
void mathtypeSetVal(mathtype *obj, float init_val, float target_val)
{
    float temp = obj->now_val;
    memset(obj, 0, sizeof(mathtype));
    obj->init_val = init_val;
    obj->target_val = target_val;
    obj->now_val = temp;
}

// >�ú���Ϊ�û��ӿ�
void mathtypeUpdate(mathtype *obj, mathCalUpdate callback, float timebase, float spd)
{
    callback(obj, timebase, spd);
}

/**
 * @brief һ�����Լ���
 * @param timebase ʱ��,����ʱ��
 * @param spd ����Ĳ���,�ٶ�,ÿ����(ms)���µĵ�λ����
 */
void linearCalculation(mathtype *obj, float timebase, float spd)
{

    // ����������ÿ�θ��µĲ���Ϊ spd * timebase
    float increment = spd * timebase;

    // �ж��ǵ������ǵݼ��������� now_val
    if (obj->now_val < obj->target_val)
    {
        obj->now_val += increment;
        if (obj->now_val > obj->target_val)
        {
            obj->now_val = obj->target_val; // ��ֹ����Ŀ��ֵ
        }
    }
    else if (obj->now_val > obj->target_val)
    {
        obj->now_val -= increment;
        if (obj->now_val < obj->target_val)
        {
            obj->now_val = obj->target_val; // ��ֹ����Ŀ��ֵ
        }
    }
    // ��� now_val ���� target_val������Ҫ����
}