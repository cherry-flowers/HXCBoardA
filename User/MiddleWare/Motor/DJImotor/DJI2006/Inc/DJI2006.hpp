/*===========================================================
* @file      DJI2006.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* 
* ===========================================================
* 该文件功能表述(先声明后定义):
* ===========================================================
* @version   0.4
* @date      2025-11-27
* @copyright Copyright (c) 2025
============================================================*/
#ifndef DJI2006_HPP
#define DJI2006_HPP

/*======================== 依赖文件 ==========================*/
#include "MW_Common.hpp"
#include "B2MW_CANManager.hpp"
#include "PID_Controller.hpp"
#include "MW_Math.hpp"

/*==========================宏定义开关=========================*/
/*是否开启DJI2006断言纠错功能*/
#define DJI2006_ASSERT_ENABLE 1

/**
 * @brief DJI2006断言宏 
 * @details
 * 该宏用于在DJI2006模块中检查中间件状态是否成功
 * 如果状态失败,则调用断言失败处理函数
 * @param status 断言中间件状态
 * @param msg    断言失败的信息,格式为"具体错误信息"
 * @details
 * 1. 如果断言中间件状态(status)不是MW_Status::SUCCESS,则调用MW_AssertStatusFailed函数
 * 2. 该函数会打印出模块名、断言失败信息、文件名和行号
 */
#if (DJI2006_ASSERT_ENABLE == 1)
    #define DJI2006_ASSERT(status, msg) MODULE_ASSERT(DJI2006, status, msg)
#else
    #define DJI2006_ASSERT(status, msg) ((void)0)
#endif

/*==================== DJI2006 电机类 =====================*/

/**
 * @brief DJI2006电机状态枚举
 * @details 用于表示DJI2006电机的状态
 */
enum DJI2006_Status{
    DJI2006_Online = 0,
    DJI2006_Offline
};

/**
 * @brief DJI2006控制模式枚举
 * @details 用于设置DJI2006电机的控制模式
 */
enum DJI2006_ControlMode{
    DJI2006_OpenLoopMode = 0, /*开环电流*/
    DJI2006_SpeedLoopMode,    /*闭环速度*/
    DJI2006_LocationLoopMode  /*闭环位置*/
};

/**
 * @brief DJI2006 PID闭环控制器枚举
 * @details 用于设置DJI2006电机的哪个环路的PID控制器模式(与ControlMode对应)
 */
enum DJI2006_PID_LOOP{
    DJI2006_SpeedPIDLoop = 1,
    DJI2006_LocationPIDLoop,
};

/**
 * @brief C610反馈CAN帧枚举,C610返回的Can消息对应的DJI2006的电机ID,
 * 不要传入DJI2006_Begin,DJI2006_End,DJI2006_IDLowMax,DJI2006_IDHighMin这些作为电机的ID
 */
enum DJI2006_ID :uint32_t{
    DJI2006_Begin = 0x201,
    
    
    DJI2006_1 = 0x201,
    DJI2006_2 = 0x202,
    DJI2006_3 = 0x203,
    DJI2006_4 = 0x204,
    DJI2006_IDLowMax =0x204,
    
    DJI2006_IDHighMin = 0x205,
    DJI2006_5 = 0x205,
    DJI2006_6 = 0x206,
    DJI2006_7 = 0x207,
    DJI2006_8 = 0x208,
    DJI2006_End = 0x209,
};

/**
 * @brief C610反馈CAN帧结构体
 * @details 用于表示C610返回的DJI2006电机的反馈信息,与C620电流映射范围不同
 * @note 1. 转子角度 单位° 8192 映射到0-360°
 * @note 2. 转子转速 单位RPM
 * @note 3. 输出电流 单位A 映射到-16384-16384,对应-10A到+10A
 * @note 4. 保留位
 */
struct __attribute__((__packed__)) C610_FeedBackMsg{
    uint16_t Encoder;      /* 转子角度 单位° 8192 映射到0-360° */
    int16_t  RPM;          /* 转子转速 单位RPM */
    int16_t  Current;      /* 输出电流 单位A 映射到-16384-16384,对应-10A到+10A */
    uint16_t Reserved;     /* 保留位 */
};

/**
 * @brief DJI2006电机数据结构体
 * @details 用于存储DJI2006电机的数据(统一是指的电机输出端的数据)
 */
struct DJI2006_Data{
    float32_t Exp_Angle;          /* 电机输出轴期望角度 单位° */
    float32_t Exp_Rad;            /* 电机输出轴期望弧度 单位Rad */
    float32_t Exp_Speed;          /* 电机输出轴期望速度 单位Rad/s */
    float32_t Exp_Current;        /* 电机输出轴期望电流 单位mA */
    float32_t Now_Angle;          /* 电机输出轴当前角度 单位° */
    float32_t Now_Rad;            /* 电机输出轴当前弧度 单位Rad */
    float32_t Now_Speed;          /* 电机输出轴当前速度 单位Rad/s */
    float32_t Now_Current;        /* 电机输出轴当前电流 单位mA */
    uint16_t  Pre_Encoder;        /* 电机上次编码器值 */
    int32_t   Total_Encoder;      /* 电机总编码器值 */
    int32_t   Total_RotorRound;   /* 电机转子总圈数 */
    int32_t   Online_CountFlag;   /* 电机在线次数标志位 */
};

/**
 * @brief DJI2006电机特征参数
 * @details 用于存储DJI2006电机的特征参数
 */
namespace DJI2006_CharacterParam{
    constexpr float32_t GearBoxRate = 36.f;                                   /* 齿轮箱减速比 */
    constexpr float32_t IDLE_GearBoxRPM =  500.0f ;                           /* 空载齿轮箱转速 RPM */
    constexpr float32_t IDLE_GearBoxRad = IDLE_GearBoxRPM *PI /30.0f;         /* 空载齿轮箱转速 Rad/s */
    constexpr float32_t Rated_GearBoxRPM = 416.0f;                            /* 额定齿轮箱转速 RPM */
    constexpr float32_t Rated_GearBoxRad = Rated_GearBoxRPM *PI /30.0f;       /* 额定齿轮箱转速 Rad/s */
    constexpr float32_t IDLE_RotorRPM = IDLE_GearBoxRPM * GearBoxRate;        /* 转子空载转速 */
    constexpr float32_t IDLE_RotorCurrent_A = 0.6f;                           /* 转子空载电流 单位A */
    constexpr float32_t IDLE_RotorCurrent_mA = IDLE_RotorCurrent_A * 1000.0f; /* 转子空载电流 单位mA */
    constexpr float32_t Rated_RotorRPM = Rated_GearBoxRPM*GearBoxRate;        /* 转子额定转速 */ 
    constexpr float32_t Rated_Torque_NM = 1.0f;                               /* 转子额定转矩,最大连续力矩(N.m) */
    constexpr float32_t Rated_Current_A = 3.0f;                               /* 额定电流 单位A */
    constexpr float32_t Rated_Current_mA = Rated_Current_A * 1000.0f;         /* 额定电流 单位mA */
    constexpr float32_t Rated_Voltage_V = 24.0f;                              /* 额定电压 单位V */
    constexpr float32_t Torque_Constant_NM_A = 0.18f;                         /* 转矩常数 N.m/A */
    constexpr float32_t Speed_Constant_RPM_V = 32.96f;                        /* 速度常数 RPM/V */
    constexpr float32_t Speed_Torque_Gradient = 110.0f;                       /* 转速转矩梯度 (RPM)/N.m */
    constexpr float32_t MaxTemperature = 55.0f;                               /* 最大温度 单位C° */
    constexpr uint16_t  Encoder_Per_Round = 8192;                             /* 编码器最大值 0-8191 映射到0-360° */
    constexpr float32_t Max_C610Current = 10000.0f;                           /* 电调最大控制电流 10000mA*/
};

class DJI2006{

public:
/**
 * @brief 构造函数
 * @param motorID               电机ID,范围为1-8
 * @param canBus                使用的CAN总线
 * @param baudRate              CAN总线波特率
 * @param Location_PID_Param    位置PID参数
 * @param Speed_PID_Param       速度PID参数
 * @param controlMode           控制模式,默认开环电流模式
 * @param canMode               CAN总线模式,默认正常模式
 */
DJI2006(DJI2006_ID motorID,
        USE_CanBus canBus,
        Can::CanBaudRate baudRate,
        PID_Param& Location_PID_Param,
        PID_Param& Speed_PID_Param,
        DJI2006_ControlMode controlMode = DJI2006_ControlMode::DJI2006_OpenLoopMode,
        Can::CanMode canMode = Can::CanMode::MODE_NORMAL);

/**
 * @brief 析构函数
 */
~DJI2006();

/**
 * @brief  初始化DJI2006电机
 * @return MW_Status 初始化结果
 * @note   该函数启动CAN总线资源并设置CAN接受回调函数
 */
MW_Status Init();

/**
 * @brief 启动DJI2006电机控制任务
 * @return MW_Status 启动结果
 * @note   该函数会创建一个任务,用于控制电机(多个电机也依旧是这个任务去管理)
 */
MW_Status StartControlTask(void);

/**
 * @brief 依据电机控制模式设置电机期望值
 * @param exp  
 * 1. 如果是位置模式就是角度
 * 2. 如果是速度模式就是速度
 * 3. 如果是电流模式就是电流
 * @return MW_Status 设置结果
 */
MW_Status SetExpect(float32_t exp);

/**
 * @brief 设置DJI2006电机的哪个环路的PID控制器模式
 * @param Loop 环路枚举,用于指定设置哪个环路的PID控制器模式
 * @param D_First_Mode D项系数模式
 * @param I_Limit_Mode I项积分限幅模式
 * @param DeedZone_Mode 死区模式
 * @param I_Separate_Mode I项分离模式
 * @param I_VarSpeed_Mode I项变速模式
 * @param Output_Limit_Mode 输出限幅模式
 * @param FeedForward_Mode 前馈模式
 * @return MW_Status 设置结果
 * @note  该函数用于设置DJI2006电机中指定环路的PID控制器模式
 */
MW_Status SetPIDControllerMode(DJI2006_PID_LOOP Loop,
                                PID_D_First_Mode D_First_Mode,
                                PID_I_Limit_Mode I_Limit_Mode,
                                PID_DeedZone_Mode DeedZone_Mode,
                                PID_I_Separate_Mode I_Separate_Mode,
                                PID_I_VarSpeed_Mode I_VarSpeed_Mode,
                                PID_Output_Limit_Mode Output_Limit_Mode,
                                PID_FeedForward_Mode FeedForward_Mode);

/**
 * @brief 获取DJI2006电机的C610反馈信息
 * @return C610_FeedBackMsg C610反馈信息结构体
 * @note  该函数用于获取电调C610反馈信息
 */
const C610_FeedBackMsg& getC610FeedBackMsg();

/**
 * @brief 获取DJI2006电机的数据参数
 * @return DJI2006_Data 电机数据参数结构体
 * @note  该函数用于获取电机的实时数据参数
 */
const DJI2006_Data& getMotoData();

private:
/*===================== 私有成员变量 =====================*/ 

/* 电机ID，范围为DJI2006_1-DJI2006_8 */
DJI2006_ID MotorID;
/* 使用的CAN总线 */
USE_CanBus CanBus;
/* CAN总线波特率 */
Can::CanBaudRate BaudRate;
/* CAN总线模式 */
Can::CanMode Mode;
/* 电机控制模式,默认开环电流模式 */
DJI2006_ControlMode ControlMode;
/* 硬件资源状态 */
bool Resource_Inited;
/* 电机状态,默认离线 */
DJI2006_Status Status;
/* C610反馈消息结构体 */
C610_FeedBackMsg FeedBackMsg;
/* 电机的数据参数 */
DJI2006_Data MotorData;
/* 指向CanManager的指针，用于申请CAN总线资源 */
CanManager* CanManagerPtr;

/*===================== 电机算法控制器 ======================*/

/* 位置PID控制器 */
Loc_PID_Controller Location_PID;
/* 速度PID控制器 */
Loc_PID_Controller Speed_PID;

/*====================== 私有成员函数 =======================*/

/**
 * @brief 发送当前电流指令
 * @param current 电流指令,范围为-16384-16384,映射到-20A到+20A
 * @return MW_Status 发送结果
 * @note  该函数用于底层测试,排查物理连接是否正常,正常情况不应调用API
 */
MW_Status SendCanMsg(int16_t current);

/**
 * @brief  向CANManger申请CAN资源(被Init调用)
 * @return MW_Status 申请结果
 */
MW_Status InitResource();

/**
 * @brief  向CANManger订阅CAN消息回调函数(被Init调用)
 * @return MW_Status 设置结果
 * @details
 * 1. 该函数会向 CanManager 注册 DJI2006_CanMsgCallBack 函数作为静态回调函数
 * 2. 该函数被Init调用
 */
MW_Status SetRouterCallBack();

/*==================== 静态成员变量和函数 ====================*/

/**
 * @brief 控制任务函数句柄,指向控制任务函数ControlTask
 */
static TaskHandle_t ControlTaskHandle;

/**
 * @brief 用于发送CAN消息的结构体,负责发送1-4号电机的指令
 */
static CanMessage LowID_CanMsgSend;

/**
 * @brief 用于发送CAN消息的结构体,负责发送5-8号电机的指令
 */
static CanMessage HighID_CanMsgSend;

/**
 * @brief 静态实例注册表,存储所有DJI2006实例的指针
 */
static DJI2006* Instance_Registry[DJI2006_End-DJI2006_Begin];

/**
 * @brief 电机2006的CAN消息回调函数
 * @param canId CAN消息ID
 * @param data  指向CAN消息数据的指针
 * @param len   CAN消息数据长度
 * @details
 * 1. 该函数会根据CAN消息ID将数据解析为C610_FeedBackMsg结构体
 * 2. 该函数会将解析出的C610_FeedBackMsg结构体存储在对应的DJI2006实例中
 */
static void DJI2006_CanMsgCallBack(uint32_t canId, uint8_t* data, uint8_t len);

/**
 * @brief  将计算出的控制输出加入到对应的发送CAN消息的结构体中
 * @param Instance 指向DJI2006实例的指针
 * @param controlOut 计算出的控制输出
 */
static void AddControlOutToCanMsg(DJI2006* Instance, int16_t Input_cur);

/**
 * @brief 控制任务函数,用于根据控制模式,控制所有注册的DJI2006电机
 */
static void ControlTask(void* pvParameters); 

};

#endif /* DJI2006_HPP */