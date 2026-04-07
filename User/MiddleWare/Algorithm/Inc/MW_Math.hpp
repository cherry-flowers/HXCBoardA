/*===========================================================
* @file      MW_Math.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* MW_Common.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* 封装了一些中间件常用的数学函数
* ===========================================================
* @version   0.1
* @date      2025-10-23
* @copyright Copyright (c) 2025
============================================================*/
#ifndef MW_MATH_HPP
#define MW_MATH_HPP

/*=========================依赖文件===========================*/
#include "MW_Common.hpp"
#include "arm_math.h"

inline float Int16ToFloat(int16_t x, int16_t int16Min, int16_t int16Max, float floatMin, float floatMax) {
    if (int16Max == int16Min) {
        return floatMin;
    }
    return floatMin + (floatMax - floatMin) * ((float)x - (float)int16Min) / ((float)int16Max - (float)int16Min);
}

inline float Int32ToFloat(int32_t x, int32_t int32Min, int32_t int32Max, float floatMin, float floatMax) {
    if (int32Max == int32Min) {
        return floatMin;
    }
    return floatMin + (floatMax - floatMin) * ((float)x - (float)int32Min) / ((float)int32Max - (float)int32Min);
}

inline int32_t FloatToInt32(float x, float floatMin, float floatMax, int32_t int32Min, int32_t int32Max) {
    if (floatMax == floatMin) {
        return int32Min;
    }

    float result = (float)int32Min + (x - floatMin) * ((float)int32Max - (float)int32Min) / (floatMax - floatMin);

    if (result > (float)int32Max) {
        return int32Max;
    }
    if (result < (float)int32Min) {
        return int32Min;
    }

    return (int32_t)result;
}

inline int16_t FloatToInt16(float x, float floatMin, float floatMax, int16_t int16Min, int16_t int16Max) {
    if (floatMax == floatMin) {
        return int16Min;
    }

    float result = (float)int16Min + (x - floatMin) * ((float)int16Max - (float)int16Min) / (floatMax - floatMin);

    if (result > (float)int16Max) {
        return int16Max;
    }
    if (result < (float)int16Min) {
        return int16Min;
    }

    return (int16_t)result;
}

/**
 * @brief 约束值在指定范围内
 * @tparam T 任意数值类型
 * @param value 要约束的值
 * @param min 最小值
 * @param max 最大值
 */
template <typename T>
void Constrain(T& value, T min, T max) {
    if (value < min) {
        value = min;
    }
    if (value > max) {
        value = max;
    }
}

/**
 * @brief 取绝对值
 * @tparam T 任意数值类型
 * @param value 要取绝对值的值
 */
template <typename T>
T Abs(T value) {
    if (value < 0) {
        value = -value;
    }
    return value;
}

/**
 * @brief 弧度转换为角度
 * @param rad 要转换的弧度值
 * @return float 转换后的角度值
 */
inline float Rad2Deg(float rad) {
    return (float)( rad * (float)180.0 / (float)PI);
}

/**
 * @brief 角度转换为弧度
 * @param deg 要转换的角度值
 * @return float 转换后的弧度值
 */
inline float Deg2Rad(float deg) {
    return (float)( deg * (float)PI / (float)180.0);
}

/**
 * @brief RPM转换为rad/s
 * @param rpm 要转换的RPM值
 * @return float 转换后的rad/s值
 */
inline float RPM2Rad(float rpm) {
    return (float)( rpm * (float)PI / (float)30.0);
}

/**
 * @brief rad/s转换为RPM
 * @param rad 要转换的rad/s值
 * @return float 转换后的RPM值
 */
inline float Rad2RPM(float rad) {
    return (float)( rad * (float)30.0 / (float)PI);
}

/**
 * @brief 将浮点数映射到指定位宽的无符号整数
 * @param x 要映射的浮点数
 * @param x_min 浮点数的最小值
 * @param x_max 浮点数的最大值
 * @param bits 目标位宽（不要大于31位）
 * @return uint32_t 映射后的无符号整数
 */
inline uint32_t FloatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    // 限制范围
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    // 特判 32 位，防止 1UL << 32 溢出
    if (bits >= 32) {
        return (uint32_t)((x - offset) * 4294967295.0f / span);
    }
    // 映射
    return (uint32_t) ((x - offset) * ((float)((1UL << bits) - 1)) / span);
}
#endif /* MW_MATH_HPP */