/*===========================================================
* @file      DMJ4310.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* MW_Common.hpp
* B2MW_CANManager.hpp
* arm_math.h
* MW_Math.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* ===========================================================
* @version   0.8
* @date      2025-12-14
* @copyright Copyright (c) 2025
============================================================*/
#ifndef DMJ4310_HPP
#define DMJ4310_HPP

/*======================= 依赖C++文件 ========================*/
#include "MW_Common.hpp"
#include "B2MW_CANManager.hpp"
#include "MW_Math.hpp"

/*======================== 依赖C文件 =========================*/
#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"

#ifdef __cplusplus
}
#endif

/*==========================宏定义开关=========================*/

/*是否开启DMJ4310断言纠错功能*/
#define DMJ4310_ASSERT_ENABLE 1

/**
 * @brief DMJ4310断言宏 
 * @details
 * 该宏用于在DMJ4310模块中检查中间件状态是否成功
 * 如果状态失败,则调用断言失败处理函数
 * @param status 断言中间件状态
 * @param msg    断言失败的信息,格式为"具体错误信息"
 * @details
 * 1. 如果断言中间件状态(status)不是MW_Status::SUCCESS,则调用MW_AssertStatusFailed函数
 * 2. 该函数会打印出模块名、断言失败信息、文件名和行号
 */
#if (DMJ4310_ASSERT_ENABLE == 1)
    #define DMJ4310_ASSERT(status, msg) MODULE_ASSERT(DMJ4310, status, msg)
#else
    #define DMJ4310_ASSERT(status, msg) ((void)0)
#endif

/*======================== 枚举定义 ==========================*/

/**
 * @brief DMJ4310状态枚举
 */
enum DMJ4310_Status:uint8_t{
    DMJ4310_DISABLE      = 0x0,   /* 失能 */
    DMJ4310_ENABLE       = 0x1,   /* 使能 */
    DMJ4310_OCUnexpected = 0x3,   /* 输出轴检查异常 */
    DMJ4310_SensorError  = 0x4,   /* 传感器异常 */
    DMJ4310_EncoderError = 0x5,   /* 编码器异常 */
    DMJ4310_OverVoltage  = 0x8,   /* 过压 */
    DMJ4310_UnderValtage = 0x9,   /* 欠压 */
    DMJ4310_OverCurrent  = 0xA,   /* 过流 */
    DMJ4310_MosOverTemp  = 0xB,   /* MOS过温 */
    DMJ4310_CoilOverTemp = 0xC,   /* 线圈过温 */
    DMJ4310_Offline      = 0xD,   /* 通讯丢失 */
    DMJ4310_OverLoad     = 0xE,   /* 过载 */
};

/**
 * @brief DMJ4310控制模式枚举
 */
enum DMJ4310_ControlMode:uint8_t{
    DMJ4310_MIT_Location_Mode = 0x00,           /* MIT位置模式 */
    DMJ4310_MIT_Speed_Mode    = 0x01,           /* MIT速度模式 */
    DMJ4310_MIT_Torque_Mode   = 0x02,           /* MIT定扭矩模式 */
    DMJ4310_Cascade_Loc_Mode  = 0x03,           /* 串级位置速度PI模式 */
    DMJ4310_Speed_Mode        = 0x04,           /* 速度PI模式 */    
    DMJ4310_Force_Loc_Mode    = 0x05,           /* 力位混控模式 */
};

/**
 * @brief DMJ4310ID枚举
 * @note  (用于发送控制包)
 * @note  规范电机ID
 */
enum DMJ4310_ID :uint32_t{
    DMJ4310_Begin = 0x01,
    
    DMJ4310_1     = 0x01,
    DMJ4310_2     = 0x02,
    DMJ4310_3     = 0x03,
    DMJ4310_4     = 0x04,
    DMJ4310_5     = 0x05,
    DMJ4310_6     = 0x06,
    DMJ4310_7     = 0x07,
    DMJ4310_8     = 0x08,

    DMJ4310_End   = 0x09,
};

/**
 * @brief DMJ4310反馈ID枚举
 * @note  (用于接收反馈包)
 * @note  规范电机反馈ID
 */
enum DMJ4310_MST_ID:uint32_t{
    DMJ4310_MST_Begin = 0xF1,
    
    DMJ4310_MST_1     = 0xF1,
    DMJ4310_MST_2     = 0xF2,
    DMJ4310_MST_3     = 0xF3,
    DMJ4310_MST_4     = 0xF4,
    DMJ4310_MST_5     = 0xF5,
    DMJ4310_MST_6     = 0xF6,
    DMJ4310_MST_7     = 0xF7,
    DMJ4310_MST_8     = 0xF8,
    
    DMJ4310_MST_End   = 0xF9,
};

/**
 * @brief DMJ4310ID偏移枚举(用于发送控制包)
 */
enum class DMJ4310_IDoffset{
    MIT_Offset = 0x00,                  /* MIT偏移 */
    Cascade_LocPI_Offset = 0x100,       /* 级联位置PI偏移 */
    Speed_PI_Offset = 0x200,            /* 速度PI偏移 */
    Cascade_LocPI_Limit_Offset = 0x300, /* 级联位置PI限制偏移 */
};

/**
 * @brief DMJ4310命令枚举(控制命令不在其中)
 */
enum class DMJ4310_Command{
    Enable_Command = 0x01,              /* 使能命令 */
    Disable_Command = 0x02,             /* 失能命令 */
    SaveZero_Command = 0x03,            /* 保存位置零点命令 */
    ClearErr_Command = 0x04,            /* 清除错误命令 */
};

/**
 * @brief DMJ4310寄存器地址枚举
 * @note  枚举格式为: 寄存器名_读写权限_存储的数据类型
 * @note  请在熟悉DMJ4310数据手册后再使用
 */
enum class DMJ4310_Register{
    /*==================== 可读写寄存器 ======================*/
    UV_Value_RW_f32    = 0x00,  /* 低压保护值 */
    KT_Value_RW_f32    = 0x01,  /* 扭矩系数 */
    OT_Value_RW_f32    = 0x02,  /* 过温保护值 */
    OC_Value_RW_f32    = 0x03,  /* 过流保护值 */
    ACC_RW_f32         = 0x04,  /* 加速度 */
    DEC_RW_f32         = 0x05,  /* 减速度 */
    MAX_SPD_RW_f32     = 0x06,  /* 最大速度 */
    MST_ID_RW_u32      = 0x07,  /* 反馈 ID */
    ESC_ID_RW_u32      = 0x08,  /* 接收 ID */
    TIMEOUT_RW_u32     = 0x09,  /* 超时警报时间 */
    CTRL_MODE_RW_u32   = 0x0A,  /* 控制模式 */
    /*===================== 只读寄存器 ======================*/    
    Damp_RO_f32        = 0x0B,  /* 电机粘滞系数 */
    Inertia_RO_f32     = 0x0C,  /* 电机转动惯量 */
    hw_ver_RO_u32      = 0x0D,  /* 硬件版本号 (保留) */
    sw_ver_RO_u32      = 0x0E,  /* 软件版本号 */
    SN_RO_u32          = 0x0F,  /* 序列号 (保留) */
    NPP_RO_u32         = 0x10,  /* 电机极对数 */
    Rs_RO_f32          = 0x11,  /* 电机相电阻 */
    Ls_RO_f32          = 0x12,  /* 电机相电感 */
    Flux_RO_f32        = 0x13,  /* 电机磁链值 */
    Gr_RO_f32          = 0x14,  /* 齿轮减速比 */
    /*==================== 可读写寄存器 ======================*/
    PMAX_RW_f32        = 0x15,  /* 位置映射范围 */
    VMAX_RW_f32        = 0x16,  /* 速度映射范围 */
    TMAX_RW_f32        = 0x17,  /* 扭矩映射范围 */
    I_BW_RW_f32        = 0x18,  /* 电流环控制带宽 */
    KP_ASR_RW_f32      = 0x19,  /* 速度环 Kp */
    KI_ASR_RW_f32      = 0x1A,  /* 速度环 Ki */
    KP_APR_RW_f32      = 0x1B,  /* 位置环 Kp */
    KI_APR_RW_f32      = 0x1C,  /* 位置环 Ki */
    OV_Value_RW_f32    = 0x1D,  /* 过压保护值 */
    GREF_RW_f32        = 0x1E,  /* 齿轮力矩效率 */
    Deta_RW_f32        = 0x1F,  /* 速度环阻尼系数 */
    V_BW_RW_f32        = 0x20,  /* 速度环滤波带宽 */
    IQ_cl_RW_f32       = 0x21,  /* 电流环增强系数 */
    VL_cl_RW_f32       = 0x22,  /* 速度环增强系数 */
    can_br_RW_u32      = 0x23,  /* CAN 波特率代码 */
    /*===================== 只读寄存器 ======================*/    
    sub_ver_RO_u32     = 0x24,  /* 子版本号 */
    Boot_ver_RO_u32    = 0x25,  /* Boot 版本号 */
    dir_RO_f32         = 0x37,  /* 方向 */
    m_off_RO_f32       = 0x38,  /* 电机侧角度偏移 */
    Imax_RO_f32        = 0x3B,  /* 驱动板最大电流 */
    VBus_RO_f32        = 0x3C,  /* 电源电压 */
    Tpcb_RO_f32        = 0x3D,  /* 驱动板温度 */
    Tmtr_RO_f32        = 0x3E,  /* 电机温度 */
    Iu_off_RO_f32      = 0x3F,  /* U 相电流偏置 */
    Iv_off_RO_f32      = 0x40,  /* V 相电流偏置 */
    Iw_off_RO_f32      = 0x41,  /* W 相电流偏置 */
    p_m_RO_f32         = 0x50,  /* 电机当前位置 单位:rad */
    xout_RO_f32        = 0x51,  /* 输出轴位置 单位:rad */
};

/***
 * @brief DMJ4310反馈数据包
 * @details __packed 用于确保结构体在内存中按字节对齐,不添加填充字节
 */
struct __attribute__((__packed__)) DMJ4310_FeedBackMsg{
    uint8_t  ERR_ID;         /* ID|ERR<<4*/
    uint8_t  Pos15_8;        /* 位置高8位 */
    uint8_t  Pos7_0;         /* 位置低8位 */
    uint8_t  Speed11_4;      /* 速度高8位*/
    uint8_t  Speed3_0_T11_8; /* 速度低4位和扭矩高4位*/
    uint8_t  Torque7_0;      /* 扭矩低8位 */
    uint8_t  T_MOS;          /* 电机MOS温度,单位:℃ */
    uint8_t  T_Rotor;        /* 电机转子温度,单位:℃ */
};

/**
 * @brief DMJ4310数据结构体
 * @details 该结构体用于存储DMJ4310电机的实时位置(这里的数据统一是指输出端)
 */
struct DMJ4310_Data{
    /* 以下参数被用于用户设置期望值 */
    float32_t Exp_Angle;         /* 电机输出轴期望角度 单位:° */
    float32_t Exp_Rad;           /* 电机输出轴期望弧度 单位:rad */
    float32_t Exp_Speed;         /* 电机输出轴期望速度 单位:rad/s */
    float32_t Exp_Torque;        /* 电机输出轴期望扭矩 单位:N.m */
    float32_t Kp;                /* 电机MIT模式下的Kp */
    float32_t Kd;                /* 电机MIT模式下的Kd */
    float32_t Speed_Limit;       /* 电机力位混控模式下和电机级联位置速度模式下的速度限制 单位:rad/s */
    float32_t Torque_Max_Rate;   /* 电机力位混控模式下的最大扭矩比例 */

    /* 以下参数被用于发包和映射数值 */
    uint16_t  MIT_p_des;         /* 电机MIT模式下的期望位置映射值(用于发包) 16位数据*/
    uint16_t  MIT_v_des;         /* 电机MIT模式下的期望速度映射值(用于发包) 12位数据*/
    uint16_t  MIT_t_ff;          /* 电机MIT模式下的期望扭矩映射值(用于发包) 12位数据*/
    uint16_t  MIT_Kp_Map;        /* 电机MIT模式下的Kp映射值(用于发包) 12位数据*/
    uint16_t  MIT_Kd_Map;        /* 电机MIT模式下的Kd映射值(用于发包) 12位数据*/
    uint16_t  FL_V_des;          /* 电机力位混控模式下的限速值(用于发包) */
    uint16_t  FL_I_des;          /* 电机力位混控模式下的限制扭矩电流标幺值(实际相电流/最大相电流)*/

    /* 以下参数被用于回调函数 */    
    uint16_t  Now_Encoder;       /* 电机输出轴反馈帧中的POS值.根据PMAX去映射到具体的弧度 */
    float32_t Now_Angle;         /* 电机输出轴当前角度 单位:° */
    float32_t Now_Rad;           /* 电机输出轴当前弧度 单位:rad */
    float32_t Now_Speed;         /* 电机输出轴当前速度 单位:rad/s */
    float32_t Now_Torque;        /* 电机输出轴当前扭矩 单位:N.m */
    uint8_t   Mos_Temp;          /* 电机MOS平均温度,单位:℃ */
    uint8_t   Rotor_Temp;        /* 电机转子平均温度,单位:℃ */
    uint16_t  Pre_Encoder;       /* 电机输出轴上一次反馈帧中的POS值.根据PMAX去映射到具体的弧度 */
    int32_t   Total_Encoder;     /* 电机输出轴总编码器值.根据PMAX去映射到具体的弧度 */
    int32_t   Total_Round;       /* 电机输出轴总转数 */
    int32_t   Total_EncoderRound;/* 电机输出轴总编码器转数 */
    bool      FirstGetMSG_Flag;  /* 初始化标志位,收到第一帧数据置1 */
};

/**
 * @brief DMJ4310电机特征参数
 * @details 用于存储DMJ4310电机的特征参数
 */
namespace DMJ4310_CharacterParam{
    constexpr float32_t GearBoxRate = 10.f;                                     /* 齿轮箱减速比 */
    constexpr float32_t IDLE_GearBoxRPM = 200.f;                                /* 空载齿轮箱转速 RPM */
    constexpr float32_t IDLE_GearBoxRad = IDLE_GearBoxRPM * 2.f * PI / 60.f;    /* 空载齿轮箱转速 单位:rad/s */
    constexpr float32_t Rated_GearBoxRPM = 120.f;                               /* 额定齿轮箱转速 RPM */
    constexpr float32_t Rated_GearBoxRad = Rated_GearBoxRPM * 2.f * PI / 60.f;  /* 额定齿轮箱转速 单位:rad/s */
    constexpr int32_t   Encoder_Per_Round = 65536;                              /* 编码器最大计数(16位)  */
    constexpr uint16_t  Speed_Per_Round = 4096;                                 /* 速度最大表示范围 */
    constexpr uint16_t  Torque_Per_Round = 4096;                                /* 扭矩最大表示范围 */
    constexpr float32_t KPMax = 500.0f;                                         /* MIT模式Kp最大值 500*/
    constexpr float32_t KpMin = 0.0f;                                           /* MIT模式Kp最小值 0*/
    constexpr float32_t KdMax = 5.0f;                                           /* MIT模式Kd最大值 5*/
    constexpr float32_t KdMin = 0.0f;                                           /* MIT模式Kd最小值 0*/
};

class DMJ4310{

/** @brief 为映射创建一个类型别名,提高可读性 */
using Map_t = float;

public:

/**
 * @brief 构造函数
 * @param motorid       电机ID
 * @param feedbackid    反馈ID
 * @param canBus        使用的CAN总线
 * @param baudrate      CAN总线波特率
 * @param mode          CAN总线模式
 * @param pmax          电机位置映射范围(PMAX) 单位:rad
 * @param vmax          电机速度映射范围(VMAX) 单位:rad/s
 * @param tmax          电机扭矩映射范围(TMAX) 单位:N.m
 * @param controlMode   电机控制模式
 */
DMJ4310(DMJ4310_ID motorid,
        DMJ4310_MST_ID feedbackid,
        USE_CanBus canBus,
        Can::CanBaudRate baudrate,
        Can::CanMode mode,
        Map_t pmax,
        Map_t vmax,
        Map_t tmax,
        DMJ4310_ControlMode controlMode);   

/**
 * @brief 析构函数
 */
~DMJ4310();

/**
 * @brief  初始化函数
 * @return MW_Status 初始化状态
 * @note   该函数启动CAN总线资源并设置CAN接受回调函数
 */
MW_Status Init();

/**
 * @brief 启动DMJ4310电机控制任务
 * @return MW_Status 启动结果
 * @note   该函数会创建一个任务,用于控制电机(多个电机也依旧是这个任务去管理)
 */
MW_Status Start_ControlTask();

/**
 * @brief  发送命令函数
 * @param command 命令结构体引用
 * @return MW_Status 发送状态
 * @note   用于发送DMJ4310命令,将命令填充到ConmandMsg中
 */
MW_Status Send_Command(const DMJ4310_Command& command);

/**
 * @brief  设置MIT_Location_Mode模式下的期望值
 * @param Exp_Rad    位置期望 单位:rad [-PMAX,PMAX]
 * @param Exp_Speed  速度期望 单位:rad/s [-VMAX,VMAX]
 * @param Exp_Torque 扭矩期望 单位:N.m [-TMAX,TMAX]
 * @param Kp         Kp参数 (不能为0) (0,500]
 * @param Kd         Kd参数 (不能为0) (0,5]
 * @return MW_Status 设置状态
 * @note   用于设置MIT_Location_Mode模式下的位置期望,速度期望,扭矩期望,和Kp,Kd参数
 * @warning 该模式下,Exp_Rad的范围为[-PMAX,PMAX]
 * @warning Exp_Speed的范围为[-VMAX,VMAX]
 * @warning Exp_Torque的范围为[-TMAX,TMAX]
 * @warning KP的范围为(0,500]
 * @warning Kd的范围为(0,5]
 */
MW_Status Set_MIT_Location_Mode_Expect(float32_t Exp_Rad,float32_t Exp_Speed,float32_t Exp_Torque,float32_t Kp,float32_t Kd);

/**
 * @brief  设置MIT_Speed_Mode模式下的期望值
 * @param Exp_Speed  速度期望 单位:rad/s [-VMAX,VMAX]
 * @param Exp_Torque 扭矩期望 单位:N.m [-TMAX,TMAX]
 * @param Kd         Kd参数 (不能为0) (0,5]
 * @return MW_Status 设置状态
 * @note   用于设置MIT_Speed_Mode模式下的速度期望,和Kd参数
 * @warning 该模式下,Exp_Speed的范围为[-VMAX,VMAX]
 * @warning Exp_Torque的范围为[-TMAX,TMAX]
 * @warning Kd的范围为(0,5]
 */
MW_Status Set_MIT_Speed_Mode_Expect(float32_t Exp_Speed,float32_t Exp_Torque, float32_t Kd);

/**
 * @brief  设置MIT_Torque_Mode模式下的期望值
 * @param Exp_Torque 扭矩期望 单位:N.m [-TMAX,TMAX]
 * @return MW_Status 设置状态
 * @note   用于设置MIT_Torque_Mode模式下的扭矩期望
 * @warning 该模式下,Exp_Torque的范围为[-TMAX,TMAX]
 */
MW_Status Set_MIT_Torque_Mode_Expect(float32_t Exp_Torque);

/**
 * @brief  设置级联位置速度模式下的期望值
 * @param Exp_Rad    位置期望 单位:rad
 * @param Exp_Speed  最高速度 单位:rad/s(非负数)
 * @return MW_Status 设置状态
 * @note   用于设置级联位置模式下的位置期望,速度期望
 * @warning Exp_Speed不可超过空载转速(0,IDLE_GearBoxRad]
 */
MW_Status Set_Cascade_Loc_Mod_Expect(float32_t Exp_Rad,float32_t Exp_Speed);

/**
 * @brief  设置速度PI模式下的期望值
 * @param Exp_Speed  速度期望 单位:rad/s
 * @return MW_Status 设置状态
 * @note   用于设置速度PI模式下的速度期望
 * @warning 该模式下,Exp_Speed不可超过空载转速[-IDLE_GearBoxRad,IDLE_GearBoxRad]
 */
MW_Status Set_Speed_Mode_Expect(float32_t Exp_Speed);

/**
 * @brief  设置力位混控下的期望值
 * @param Exp_Rad    位置期望 单位:rad
 * @param Speed_Limit 速度限制 单位:rad/s
 * @param Torque_MAX_Rate 扭矩限制最大比例,单位:0-1
 * @return MW_Status 设置状态
 * @note   用于设置位置PI模式下的位置期望
 * @warning 该模式下Speed_Limit最高转速限制为(0,100]rad/s,
 * @warning Torque_MAX_Rate的范围为[0,1]
 */
MW_Status Set_Force_Loc_Mode_Expect(float32_t Exp_Rad,float32_t Speed_Limit,float32_t Torque_MAX_Rate);

/**
 * @brief 获取DMJ4310反馈消息
 * @return DMJ4310_FeedBackMsg DMJ4310反馈消息结构体
 * @note  该函数用于获取电机反馈信息
 */
DMJ4310_FeedBackMsg getFeedBackMsg() const;

/**
 * @brief 获取DMJ4310电机的数据参数
 * @return DMJ4310_Data 电机数据参数结构体
 * @note  该函数用于获取电机的实时数据参数
 */
DMJ4310_Data getMotorData() const;

private:
/*======================== 私有成员变量 =======================*/

/** @brief 电机ID */
DMJ4310_ID Motor_ID;
/** @brief 反馈ID */
DMJ4310_MST_ID Feedback_ID;
/** @brief 使用的CAN总线 */
USE_CanBus CanBus;
/** @brief CAN总线波特率 */
Can::CanBaudRate BaudRate;
/** @brief CAN总线模式 */
Can::CanMode Mode;
/** @brief 电机位置映射范围 */
Map_t PMAX;
/** @brief 电机速度映射范围 */
Map_t VMAX;
/** @brief 电机扭矩映射范围 */
Map_t TMAX;
/** @brief 电机控制模式 */
DMJ4310_ControlMode ControlMode;
/** @brief 硬件资源状态 */
volatile bool Resource_Inited;
/** @brief 电机状态*/
volatile DMJ4310_Status Status;
/** @brief 电机ID偏移量 */
DMJ4310_IDoffset Motor_IDoffset;
/** @brief DMJ4310反馈消息结构体 */
volatile DMJ4310_FeedBackMsg FeedBackMsg;
/** @brief 电机的数据参数 */
volatile DMJ4310_Data MotorData;
/** @brief 指向CanManager的指针,用于申请CAN总线资源 */
CanManager* CanManagerPtr;

/*====================== 私有成员函数 =======================*/

/**
 * @brief  DMJ4310电机CAN初始化函数,申请CAN总线资源
 * @return MW_Status 初始化状态
 * @note   被Init调用,申请CAN总线资源，Init已经保证电机ID和CAN总线的有效性
 */
MW_Status InitResource();

/**
 * @brief  向CANManager注册CAN消息回调函数
 * @return MW_Status 设置结果
 * @note   被Init调用,设置CAN消息回调函数，Init已经保证电机ID和CAN总线的有效性
 */
MW_Status SetCallBack();

/**
 * @brief  构建使能命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建使能命令,将使能命令填充到ConmandMsg中
 */
void Build_EnableCommand(CanMessage& ConmandMsg);

/**
 * @brief  构建失能命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建禁用命令,将禁用命令填充到ConmandMsg中
 */
void Build_DisableCommand(CanMessage& ConmandMsg);

/**
 * @brief  构建保存零点命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建保存零点命令,将保存零点命令填充到ConmandMsg中
 */
void Build_SaveZeroPosCommand(CanMessage& ConmandMsg);

/**
 * @brief  构建清除错误命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建清除错误命令,将清除错误命令填充到ConmandMsg中
 */
void Build_ClearErrCommand(CanMessage& ConmandMsg);

/**
 * @brief  控制函数,根据控制模式,发送对应CAN命令
 * @note   被ControlTask调用,根据控制模式,发送对应CAN命令
 * @return MW_Status 发送状态
 */
MW_Status Send_ControlCommand();

/*==================== 静态成员变量和函数 ====================*/

/** @brief 控制任务函数句柄,指向控制任务函数ControlTask */
static TaskHandle_t ControlTaskHandle;

/** @brief 静态实例注册表，存储所有DMJ4310实例的指针 */
static DMJ4310* Instance_Registry[DMJ4310_End-DMJ4310_Begin];

/**
 * @brief DMJ4310 CAN 消息回调函数
 * @param canId CAN 消息 ID
 * @param data  CAN 消息数据指针
 * @param len   CAN 消息数据长度
 */
static void DMJ4310_CanMsgCallBack(uint32_t canId,uint8_t* data,uint8_t len);

/**
 * @brief 控制任务函数,用于根据控制模式,控制所有注册的DMJ4310电机
 */
static void ControlTask(void* pvParameters); 

};
#endif /* DMJ4310_HPP */