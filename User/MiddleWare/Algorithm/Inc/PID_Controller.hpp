/*===========================================================
* @file      Pid_Controller.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* arm_math.h
* MW_Math.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* ===========================================================
* @version   0.3
* @date      2025-11-08
* @copyright Copyright (c) 2025
============================================================*/

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "arm_math.h"
#include "MW_Math.hpp"

/**
 * @brief PID 微分项低通滤波模式
 * 该枚举用于表示PID控制器的微分项低通滤波模式
 */
enum PID_D_LowFilter_Mode{
    PID_D_LowFilter_DISABLE = 0,
    PID_D_LowFilter_ENABLE = 1
};

/**
 * @brief PID微分先行模式枚举
 * 该枚举用于表示PID控制器的微分先行模式
 */
enum PID_D_First_Mode{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE = 1
};

/**
 * @brief PID死区模式枚举
 * 该枚举用于表示PID控制器的死区模式
 */
enum PID_DeedZone_Mode{
    PID_DeedZone_DISABLE = 0,
    PID_DeedZone_ENABLE = 1
};

/**
 * @brief PID输出限制模式枚举
 * 该枚举用于表示PID控制器的输出限制模式
 */
enum PID_Output_Limit_Mode{
    PID_Output_Limit_DISABLE = 0,
    PID_Output_Limit_ENABLE = 1
};

/**
 * @brief PID积分限制模式枚举
 * 该枚举用于表示PID控制器的积分限制模式
 */
enum PID_I_Limit_Mode{
    PID_I_Limit_DISABLE = 0,
    PID_I_Limit_ENABLE = 1
};

/**
 * @brief PID积分分离模式枚举
 * 该枚举用于表示PID控制器的积分分离模式
 */
enum PID_I_Separate_Mode{
    PID_I_Separate_DISABLE = 0,
    PID_I_Separate_ENABLE = 1
};

/**
 * @brief PID积分变速模式枚举
 * 该枚举用于表示PID控制器的积分变速模式
 */
enum PID_I_VarSpeed_Mode{
    PID_I_VarSpeed_DISABLE = 0,
    PID_I_VarSpeed_ENABLE = 1
};

/**
 * @brief PID+前馈模式
 * 该枚举用于表示PID控制器的前馈系数
 */
enum PID_FeedForward_Mode{
    PID_FeedForward_DISABLE = 0,
    PID_FeedForward_ENABLE = 1
};



/**
 * @brief PID参数结构体
 * 该结构体用于存储PID控制器的参数
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param Kf 前馈系数
 * @param Dt 采样时间
 * @param DeadZone 死区
 * @param OutputLimit 输出限制
 * @param I_Limit 积分限制
 * @param I_Separate_Threshold 积分分离阈值误差
 * @param I_VarSpeed_A 积分变速A系数
 * @param I_VarSpeed_B 积分变速B系数
 * @details 参数只要为0，就默认关闭该模式
 */
class PID_Param
{
public:
    /**
     * @brief PID参数构造函数
     * @param _Kp 比例系数
     * @param _Ki 积分系数
     * @param _Kd 微分系数
     * @param _Kf 前馈系数
     * @param _Dt 采样时间
     * @param _DeadZone 死区
     * @param _OutputLimit 输出限制
     * @param _I_Limit 积分限制
     * @param _I_Separate_Threshold 积分分离阈值误差
     * @param _I_VarSpeed_A 积分变速A系数
     * @param _I_VarSpeed_B 积分变速B系数
     */
    PID_Param(float32_t _Kp,
              float32_t _Ki, 
              float32_t _Kd, 
              float32_t _Kf, 
              float32_t _Dt, 
              float32_t _DeadZone, 
              float32_t _OutputLimit, 
              float32_t _I_Limit, 
              float32_t _I_Separate_Threshold, 
              float32_t _I_VarSpeed_A, 
              float32_t _I_VarSpeed_B)
        :Kp(_Kp),
        Ki(_Ki),
        Kd(_Kd),
        Kf(_Kf),
        Dt(_Dt),
        Integral_Err(0.0f),
        P_Out(0.0f),
        I_Out(0.0f),
        D_Out(0.0f),
        F_Out(0.0f),
        Expert{0.0f, 0.0f, 0.0f},
        FeedBack{0.0f, 0.0f, 0.0f},
        Output{0.0f, 0.0f, 0.0f},
        Error{0.0f, 0.0f, 0.0f},
        DeadZone(_DeadZone),
        OutputLimit(_OutputLimit),
        I_Limit(_I_Limit),
        I_Separate_Threshold(_I_Separate_Threshold),
        I_VarSpeed_A(_I_VarSpeed_A),
        I_VarSpeed_B(_I_VarSpeed_B),
        Speed_ratio(1.0f),
        Abs_Error(0.0f){};


    /* PID参数 */
    float32_t Kp = 0.0f;                        /* 比例系数 */
    float32_t Ki = 0.0f;                        /* 积分系数 */
    float32_t Kd = 0.0f;                        /* 微分系数 */
    float32_t Kf = 0.0f;                        /* 前馈系数 */
    float32_t Dt = 0.001f;                      /* 采样时间 */ 
    float32_t Integral_Err = 0.0f;              /* 积分项中间变量 */
    float32_t P_Out = 0.0f;                     /* 比例项输出 */
    float32_t I_Out = 0.0f;                     /* 积分项输出 */
    float32_t D_Out = 0.0f;                     /* 微分项输出 */
    float32_t F_Out = 0.0f;                     /* 前馈项输出 */
    float32_t Expert[3] = {0.0f, 0.0f, 0.0f};   /* 期望 0:当前期望 1:上一次期望 2:上上次期望 */
    float32_t FeedBack[3] = {0.0f, 0.0f, 0.0f}; /* 反馈 0:当前反馈 1:上一次反馈 2:上上次反馈 */
    float32_t Output[3] = {0.0f, 0.0f, 0.0f};   /* 输出 0:当前输出 1:上一次输出 2:上上次输出 */
    float32_t Error[3] = {0.0f, 0.0f, 0.0f};    /* 误差 0:当前误差 1:上一次误差 2:上上次误差 */
    float32_t DeadZone = 0.0f;                  /* 死区 */
    float32_t OutputLimit = 0.0f;               /* 输出限制 */
    float32_t I_Limit = 0.0f;                   /* 积分限制 */
    float32_t I_Separate_Threshold = 0.0f;      /* 积分分离误差阈值 */
    float32_t I_VarSpeed_A = 0.0f;              /* 积分定速误差区间 */
    float32_t I_VarSpeed_B = 0.0f;              /* 积分变速误差区间 */
    float32_t Speed_ratio = 1.0f;               /* 积分变速系数 */
    float32_t Abs_Error = 0.0f;                 /* 绝对误差 */

    /* get and set methods 一般不用Debug的时候用 */
    inline float32_t Get_Ki() const { return Ki; }
    inline float32_t Get_Kd() const { return Kd; }
    inline float32_t Get_Kf() const { return Kf; }
    inline float32_t Get_Dt() const { return Dt; }
    inline float32_t Get_Expert(int32_t index) const { return Expert[index]; }
    inline float32_t Get_Output(int32_t index) const { return Output[index]; }
    inline float32_t Get_Error(int32_t index) const { return Error[index]; }
    inline float32_t Get_FeedBack(int32_t index) const { return FeedBack[index]; }
    inline float32_t Get_DeadZone() const { return DeadZone; }
    inline float32_t Get_OutputLimit() const { return OutputLimit; }
    inline float32_t Get_I_Limit() const { return I_Limit; }
    inline float32_t Get_I_Separate_Threshold() const { return I_Separate_Threshold; }
    inline float32_t Get_I_VarSpeed_A() const { return I_VarSpeed_A; }
    inline float32_t Get_I_VarSpeed_B() const { return I_VarSpeed_B; }

    inline void Set_Kp(float32_t Kp) { this->Kp = Kp; }
    inline void Set_Ki(float32_t Ki) { this->Ki = Ki; }
    inline void Set_Kd(float32_t Kd) { this->Kd = Kd; }
    inline void Set_Kf(float32_t Kf) { this->Kf = Kf; }
    inline void Set_Dt(float32_t Dt) { this->Dt = Dt; }
    inline void Set_Expert(int32_t index, float32_t Expert) { this->Expert[index] = Expert; }
    inline void Set_Output(int32_t index, float32_t Output) { this->Output[index] = Output; }
    inline void Set_Error(int32_t index, float32_t Error) { this->Error[index] = Error; }
    inline void Set_FeedBack(int32_t index, float32_t FeedBack) { this->FeedBack[index] = FeedBack; }
    inline void Set_DeadZone(float32_t DeadZone) { this->DeadZone = DeadZone; }
    inline void Set_OutputLimit(float32_t OutputLimit) { this->OutputLimit = OutputLimit; }
    inline void Set_I_Limit(float32_t I_Limit) { this->I_Limit = I_Limit; }
    inline void Set_I_Separate_Threshold(float32_t I_Separate_Threshold) { this->I_Separate_Threshold = I_Separate_Threshold; }
    inline void Set_I_VarSpeed_A(float32_t I_VarSpeed_A) { this->I_VarSpeed_A = I_VarSpeed_A; }
    inline void Set_I_VarSpeed_B(float32_t I_VarSpeed_B) { this->I_VarSpeed_B = I_VarSpeed_B; }
};

/**
 * @brief 位置式PID控制器类
 * 该类用于实现位置式PID控制器
 */
class Loc_PID_Controller
{
public:    
    /* PID参数 */
    PID_Param   Param;
    
    /**
     * @brief 构造函数
     * @param Param PID参数
     * @param D_First_Mode 微分先行模式
     * @param I_Limit_Mode 积分限制模式
     * @param DeedZone_Mode 死区模式
     * @param I_Separate_Mode 积分分离模式
     * @param I_VarSpeed_Mode 积分变速模式
     * @param Output_Limit_Mode 输出限制模式
     * @param FeedForward_Mode 前馈项模式
     */
    Loc_PID_Controller(const PID_Param& Param,
                   PID_D_First_Mode D_First_Mode = PID_D_First_ENABLE, 
                   PID_I_Limit_Mode I_Limit_Mode = PID_I_Limit_ENABLE, 
                   PID_DeedZone_Mode DeedZone_Mode = PID_DeedZone_ENABLE, 
                   PID_I_Separate_Mode I_Separate_Mode = PID_I_Separate_ENABLE, 
                   PID_I_VarSpeed_Mode I_VarSpeed_Mode = PID_I_VarSpeed_ENABLE, 
                   PID_Output_Limit_Mode Output_Limit_Mode = PID_Output_Limit_ENABLE,
                   PID_FeedForward_Mode FeedForward_Mode = PID_FeedForward_ENABLE);

    /**
     * @brief 计算PID输出
     * @param Expert 期望值
     * @param FeedBack 反馈值
     * @return float32_t PID输出值
     */               
    float32_t Calculate(float32_t Expert, float32_t FeedBack);
    
    /**
     * @brief 设置PID参数
     * @param Param PID参数
     */
    void Set_Param(const PID_Param& Param);

    /**
     * @brief 设置PID控制器的模式
     * @param D_First_Mode 微分先行模式
     * @param I_Limit_Mode 积分限制模式
     * @param DeedZone_Mode 死区模式
     * @param I_Separate_Mode 积分分离模式
     * @param I_VarSpeed_Mode 积分变速模式
     * @param Output_Limit_Mode 输出限制模式
     * @param FeedForward_Mode 前馈项模式
     */
     void Set_Mode(PID_D_First_Mode D_First_Mode, 
                   PID_I_Limit_Mode I_Limit_Mode, 
                   PID_DeedZone_Mode DeedZone_Mode, 
                   PID_I_Separate_Mode I_Separate_Mode, 
                   PID_I_VarSpeed_Mode I_VarSpeed_Mode, 
                   PID_Output_Limit_Mode Output_Limit_Mode,
                   PID_FeedForward_Mode FeedForward_Mode);


    /**
     * @brief 设置微分先行模式
     * @param D_First_Mode 微分先行模式
     */
    void Set_D_Mode(PID_D_First_Mode D_First_Mode);
    
    /**
     * @brief 设置积分限制模式
     * @param I_Limit_Mode 积分限制模式
     */
    void Set_I_Limit_Mode(PID_I_Limit_Mode I_Limit_Mode);

     /**
      * @brief 设置死区模式
      * @param DeedZone_Mode 死区模式
      */
     void Set_DeedZone_Mode(PID_DeedZone_Mode DeedZone_Mode);

     /**
      * @brief 设置积分分离模式
      * @param I_Separate_Mode 积分分离模式
      */
     void Set_I_Separate_Mode(PID_I_Separate_Mode I_Separate_Mode);
     
     /**
      * @brief 设置积分变速模式
      * @param I_VarSpeed_Mode 积分变速模式
      */
     void Set_I_VarSpeed_Mode(PID_I_VarSpeed_Mode I_VarSpeed_Mode);
     
     /**
      * @brief 设置输出限制模式
      * @param Output_Limit_Mode 输出限制模式
      */
     void Set_Output_Limit_Mode(PID_Output_Limit_Mode Output_Limit_Mode);
     
     /**
      * @brief 设置前馈项模式
      * @param FeedForward_Mode 前馈项模式
      */
     void Set_FeedForward_Mode(PID_FeedForward_Mode FeedForward_Mode);


private:
    /**
     * @brief 计算PID比例项输出
     */
    void Calculate_P(void);

    /**
     * @brief 计算PID积分项输出
     */
    void Calculate_I(void);
    
    /**
     * @brief 计算PID微分项输出
     */
    void Calculate_D(void);
    
    /**
     * @brief 计算PID前馈项输出
     */
    void Calculate_F(void);

    /**
     * @brief 计算PID输出
     */
    void Calculate_Out(void);

    /**
     * @brief 刷新PID控制器状态
     * @param Expert 当前期望值
     * @param FeedBack 当前反馈值
     */
    void Refresh(float32_t Expert, float32_t FeedBack);
    

    /* PID 控制器的模式 */
    PID_D_First_Mode      D_First_Mode = PID_D_First_Mode::PID_D_First_DISABLE;
    PID_I_Limit_Mode      I_Limit_Mode = PID_I_Limit_Mode::PID_I_Limit_DISABLE;
    PID_DeedZone_Mode     DeedZone_Mode = PID_DeedZone_Mode::PID_DeedZone_DISABLE;
    PID_I_Separate_Mode   I_Separate_Mode = PID_I_Separate_Mode::PID_I_Separate_DISABLE;
    PID_I_VarSpeed_Mode   I_VarSpeed_Mode = PID_I_VarSpeed_Mode::PID_I_VarSpeed_DISABLE;
    PID_Output_Limit_Mode Output_Limit_Mode = PID_Output_Limit_Mode::PID_Output_Limit_DISABLE;
    PID_FeedForward_Mode  FeedForward_Mode = PID_FeedForward_Mode::PID_FeedForward_DISABLE;
};



#endif /* PID_CONTROLLER_HPP__ */

