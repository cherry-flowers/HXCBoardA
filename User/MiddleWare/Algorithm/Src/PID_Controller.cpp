/*===========================================================
* @file      PID_Controller.cpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* Pid_Controller.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* ===========================================================
* @version   0.3
* @date      2025-11-08
* @copyright Copyright (c) 2025
============================================================*/

#include "PID_Controller.hpp"

/**
 * @brief 构造函数
 * @param Param PID参数
 * @param D_First_Mode 微分先行模式
 * @param I_Limit_Mode 积分限制模式
 * @param DeedZone_Mode 死区模式
 * @param I_Separate_Mode 积分分离模式
 * @param I_VarSpeed_Mode 积分变速模式
 * @param Output_Limit_Mode 输出限制模式
 */
Loc_PID_Controller::Loc_PID_Controller(const PID_Param& Param,
                   PID_D_First_Mode D_First_Mode, 
                   PID_I_Limit_Mode I_Limit_Mode, 
                   PID_DeedZone_Mode DeedZone_Mode, 
                   PID_I_Separate_Mode I_Separate_Mode, 
                   PID_I_VarSpeed_Mode I_VarSpeed_Mode, 
                   PID_Output_Limit_Mode Output_Limit_Mode,
                   PID_FeedForward_Mode FeedForward_Mode)
:Param(Param),
 D_First_Mode(D_First_Mode),
 I_Limit_Mode(I_Limit_Mode),
 DeedZone_Mode(DeedZone_Mode),
 I_Separate_Mode(I_Separate_Mode),
 I_VarSpeed_Mode(I_VarSpeed_Mode),
 Output_Limit_Mode(Output_Limit_Mode),
 FeedForward_Mode(FeedForward_Mode)
{}

/**
 * @brief 计算PID输出
 * @param Expert 期望值
 * @param FeedBack 反馈值
 * @return float32_t PID输出值
 */
float32_t Loc_PID_Controller::Calculate(float32_t Expert, float32_t FeedBack)
{
    /* 刷新PID控制器状态 */
    Refresh(Expert, FeedBack);
    /* 计算前馈项输出 */
    Calculate_F();    
    /* 计算P项输出 */
    Calculate_P();
    /* 计算I项输出 */
    Calculate_I();
    /* 计算D项输出 */
    Calculate_D();
    /* 计算PID输出 */
    Calculate_Out();
    /* 返回PID输出 */
    return Param.Output[0];
}

/**
 * @brief 设置PID参数
 * @param Param PID参数
 */
void Loc_PID_Controller::Set_Param(const PID_Param& Param)
{
    this->Param = Param;
}

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
void Loc_PID_Controller::Set_Mode(PID_D_First_Mode D_First_Mode, 
                   PID_I_Limit_Mode I_Limit_Mode, 
                   PID_DeedZone_Mode DeedZone_Mode, 
                   PID_I_Separate_Mode I_Separate_Mode, 
                   PID_I_VarSpeed_Mode I_VarSpeed_Mode, 
                   PID_Output_Limit_Mode Output_Limit_Mode,
                   PID_FeedForward_Mode FeedForward_Mode)
{
    this->D_First_Mode = D_First_Mode;
    this->I_Limit_Mode = I_Limit_Mode;
    this->DeedZone_Mode = DeedZone_Mode;
    this->I_Separate_Mode = I_Separate_Mode;
    this->I_VarSpeed_Mode = I_VarSpeed_Mode;
    this->Output_Limit_Mode = Output_Limit_Mode;
    this->FeedForward_Mode = FeedForward_Mode;
}

/**
 * @brief 设置微分先行模式
 * @param D_First_Mode 微分先行模式
 */
void Loc_PID_Controller::Set_D_Mode(PID_D_First_Mode D_First_Mode)
{
    this->D_First_Mode = D_First_Mode;
}

/**
 * @brief 设置积分限制模式
 * @param I_Limit_Mode 积分限制模式
 */
void Loc_PID_Controller::Set_I_Limit_Mode(PID_I_Limit_Mode I_Limit_Mode)
{
    this->I_Limit_Mode = I_Limit_Mode;
}

/**
 * @brief 设置死区模式
 * @param DeedZone_Mode 死区模式
 */
void Loc_PID_Controller::Set_DeedZone_Mode(PID_DeedZone_Mode DeedZone_Mode)
{
    this->DeedZone_Mode = DeedZone_Mode;
}

/**
 * @brief 设置积分分离模式
 * @param I_Separate_Mode 积分分离模式
 */
void Loc_PID_Controller::Set_I_Separate_Mode(PID_I_Separate_Mode I_Separate_Mode)
{
    this->I_Separate_Mode = I_Separate_Mode;
}

/**
 * @brief 设置积分变速模式
 * @param I_VarSpeed_Mode 积分变速模式
 */
void Loc_PID_Controller::Set_I_VarSpeed_Mode(PID_I_VarSpeed_Mode I_VarSpeed_Mode)
{
    this->I_VarSpeed_Mode = I_VarSpeed_Mode;
}

/**
 * @brief 设置输出限制模式
 * @param Output_Limit_Mode 输出限制模式
 */
void Loc_PID_Controller::Set_Output_Limit_Mode(PID_Output_Limit_Mode Output_Limit_Mode)
{
    this->Output_Limit_Mode = Output_Limit_Mode;
}

/**
 * @brief 设置前馈项模式
 * @param FeedForward_Mode 前馈项模式
 */
void Loc_PID_Controller::Set_FeedForward_Mode(PID_FeedForward_Mode FeedForward_Mode)
{
    this->FeedForward_Mode = FeedForward_Mode;
}

/*================== 私有函数用户无需访问 ==================*/

/**
 * @brief 计算PID前馈项输出
 */
void Loc_PID_Controller::Calculate_F(void)
{
    if(FeedForward_Mode == PID_FeedForward_ENABLE && Param.Kf != 0){
        Param.F_Out = Param.Kf * Param.Expert[0];
    }
}

/**
 * @brief 计算PID比例项输出
 */
void Loc_PID_Controller::Calculate_P(void)
{
    Param.P_Out = Param.Kp * Param.Error[0];
}

/**
 * @brief 计算PID积分项输出
 * @details 积分项输出受积分分离、积分变速、积分限制约束
 */
void Loc_PID_Controller::Calculate_I(void)
{   
    /* 积分比例系数为0，或采样时间为0(这是异常情况)，积分输出为0 */
    if(Param.Ki == 0 || Param.Dt == 0){
        /* 积分误差为0时，积分输出为0 */
        Param.I_Out = 0.0f;
        Param.Integral_Err = 0.0f;
        return;
    }
    /* 积分系数用于变速积分 */
    Param.Speed_ratio = 1.0f;
    
    /* 计算变速积分系数 */
    if(I_VarSpeed_Mode == PID_I_VarSpeed_DISABLE)
    {
        Param.Speed_ratio = 1.0f;
    }/*开启了I项变速积分*/
    else if(Param.I_VarSpeed_A > 0 && Param.I_VarSpeed_B > 0 && Param.I_VarSpeed_A < Param.I_VarSpeed_B) 
    {
        if(Param.Abs_Error <= Param.I_VarSpeed_A){
            /* 当误差绝对值小于等于I_VarSpeed_A时，积分系数为1 */
            Param.Speed_ratio = 1.0f;
        }else if(Param.Abs_Error >= Param.I_VarSpeed_B){
            /* 当误差绝对值大于等于I_VarSpeed_B时，积分系数为0，避免大误差下积分项对系统的影响 */
            Param.Speed_ratio = 0.0f;
        }else{
            /* 在I_VarSpeed_A和I_VarSpeed_B之间，误差越大，其比例系数就越小 */
            Param.Speed_ratio = (Param.I_VarSpeed_B - Param.Abs_Error) / (Param.I_VarSpeed_B - Param.I_VarSpeed_A);
        }
    }/* 开启了I项变速积分，但是I_VarSpeed_A或I_VarSpeed_B为0 */
    else{
        Param.Speed_ratio = 1.0f;
    }

    /*防御性夹逼（避免异常参数导致越界）*/ 
    if (Param.Speed_ratio < 0.0f) Param.Speed_ratio = 0.0f;
    else if (Param.Speed_ratio > 1.0f) Param.Speed_ratio = 1.0f;

    /* 积分误差 */
    Param.Integral_Err += Param.Dt * Param.Error[0] * Param.Speed_ratio;
    /* 积分输出 */
    Param.I_Out = this->Param.Ki * Param.Integral_Err;
    
    /* 是否开启积分限制,并且积分限制系数不为0 */
    if(I_Limit_Mode == PID_I_Limit_ENABLE && Param.I_Limit != 0){
       /* 积分输出受积分限制约束 */
       Constrain<float32_t>(Param.Integral_Err, -Param.I_Limit/Param.Ki, Param.I_Limit/Param.Ki);
       Constrain<float32_t>(Param.I_Out, -Param.I_Limit, Param.I_Limit);
    }
    
    /* 是否开启积分分离,并且积分分离阈值不为0 */
    if(I_Separate_Mode == PID_I_Separate_ENABLE && Param.I_Separate_Threshold != 0){
        /* 当误差绝对值大于积分分离阈值时，积分输出为0 */
        if(Param.Abs_Error >= Param.I_Separate_Threshold){
            Param.Integral_Err = 0.0f;
            Param.I_Out = 0.0f;
        }
    }
}

/**
 * @brief 计算PID微分项输出(目前还没有使用到低通滤波的情况下)
 */
void Loc_PID_Controller::Calculate_D(void)
{   if(Param.Dt == 0||Param.Kd == 0){
        Param.D_Out = 0.0f;
        return;
    }
    /* 微分先行模式为启动 */
    if(D_First_Mode == PID_D_First_ENABLE){
        Param.D_Out = this->Param.Kd * (Param.FeedBack[0] - Param.FeedBack[1]) / Param.Dt;
    }
    else{
        Param.D_Out = this->Param.Kd * (Param.Error[0] - Param.Error[1]) / Param.Dt;
    }
}


/**
 * @brief 计算PID输出
 * @details 计算PID输出值，根据比例项、积分项、微分项和前馈项的输出值进行累加。刷新输出值
 */
void Loc_PID_Controller::Calculate_Out(void)
{   
    Param.Output[2] = Param.Output[1];
    Param.Output[1] = Param.Output[0];
    Param.Output[0] = Param.P_Out + Param.I_Out + Param.D_Out + Param.F_Out;
    /* 是否开启输出限制,并且输出限制系数不为0 */
    if(Output_Limit_Mode == PID_Output_Limit_ENABLE && Param.OutputLimit != 0){
        /* 输出受输出限制约束 */
        Constrain<float32_t>(Param.Output[0], -Param.OutputLimit, Param.OutputLimit);
    }
}

/**
 * @brief 刷新PID控制器状态
 * @param Expert 当前期望值
 * @param FeedBack 当前反馈值
 * @details 刷新PID的期望和反馈值，误差值，如果开启死去模式，需要对误差进行处理
 */
void Loc_PID_Controller::Refresh(float32_t Expert, float32_t FeedBack)
{
    /* 刷新期望和反馈值 */
    this->Param.Expert[2] = Param.Expert[1];
    this->Param.Expert[1] = Param.Expert[0];
    this->Param.Expert[0] = Expert;
    
    this->Param.FeedBack[2] = Param.FeedBack[1];
    this->Param.FeedBack[1] = Param.FeedBack[0];
    this->Param.FeedBack[0] = FeedBack;

    this->Param.Error[2] = Param.Error[1];
    this->Param.Error[1] = Param.Error[0];
    this->Param.Error[0] = Expert - FeedBack;
    /* 计算误差绝对值 */
    Param.Abs_Error = abs(Param.Error[0]);
    
    /* 是否开启死区,并且死区系数不为0,执行死区操作*/
    if(DeedZone_Mode == PID_DeedZone_ENABLE && Param.DeadZone != 0){
        /* 如果误差绝对值小于等于死区，将误差设为0，反馈值设为期望 */
        if(Param.Abs_Error <= Param.DeadZone){
            Param.Error[0] = 0.0f;
            Param.FeedBack[0] = Expert;
            Param.Abs_Error = 0.0f;
        }
    }
}


