/*===========================================================
* @file      DJI3508.hpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* MW_Common.hpp
* B2MW_CANManager.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* 1. 定义了DJI3508电机的状态枚举、控制模式枚举、ID枚举、反馈ID枚举、命令枚举、寄存器地址枚举
* 2. 定义了DJI3508电机的反馈数据包结构体、数据结构体、特征参数命名空间
* 3. 定义了DJI3508电机类,包括构造函数、析构函数、初始化函数、启动控制任务函数、设置期望值函数、设置PID控制器模式函数、获取反馈消息函数、获取电机数据函数
* 4. 实现了DJI3508电机的CAN通信初始化、回调函数注册、消息解析、控制指令发送等功能
* 5. 实现了基于PID控制器的电机闭环控制(电流环、速度环、位置环)
* ===========================================================
* @version   1.8
* @date      2025-12-14
* @copyright Copyright (c) 2025
============================================================*/
#ifndef DJI3508_HPP
#define DJI3508_HPP

/*======================= 依赖C++文件 ========================*/
#include "MW_Common.hpp"
#include "B2MW_CANManager.hpp"
#include "PID_Controller.hpp"
#include "MW_Math.hpp"

/*==========================宏定义开关=========================*/

/*是否开启DJI3508断言纠错功能*/
#define DJI3508_ASSERT_ENABLE 1

/**
 * @brief DJI3508断言宏 
 * @details
 * 该宏用于在DJI3508模块中检查中间件状态是否成功
 * 如果状态失败,则调用断言失败处理函数
 * @param status 断言中间件状态
 * @param msg    断言失败的信息,格式为"具体错误信息"
 * @details
 * 1. 如果断言中间件状态(status)不是MW_Status::SUCCESS,则调用MW_AssertStatusFailed函数
 * 2. 该函数会打印出模块名、断言失败信息、文件名和行号
 */
#if (DJI3508_ASSERT_ENABLE == 1)
    #define DJI3508_ASSERT(status, msg) MODULE_ASSERT(DJI3508, status, msg)
#else
    #define DJI3508_ASSERT(status, msg) ((void)0)
#endif




/**
 * @brief DJI3508电机状态枚举
 * @details 用于表示DJI3508电机的状态
 */
enum DJI3508_Status{
    DJI3508_Online = 0,      /* 电机在线 */
    DJI3508_Offline,         /* 电机离线 */
    DJI3508_OverTemperature, /* 电机过温 */
};

/**
 * @brief DJI3508控制模式枚举
 * @details 用于设置DJI3508电机的控制模式
 */
enum DJI3508_ControlMode{
    DJI3508_OpenLoopMode = 0, /*开环电流*/
    DJI3508_SpeedLoopMode,    /*闭环速度*/
    DJI3508_LocationLoopMode  /*闭环位置*/
};

/**
 * @brief DJI3508 PID闭环控制器枚举
 * @details 用于设置DJI3508电机的哪个环路的PID控制器模式(与ControlMode对应)
 */
enum DJI3508_PID_LOOP{
    DJI3508_SpeedPIDLoop = 1,
    DJI3508_LocationPIDLoop,
};

/**
 * @brief C620反馈CAN帧枚举,C620返回的Can消息对应的DJI3508的电机ID,
 * 不要传入DJI3508_Begin和DJI3508_End这些作为电机的ID
 */
enum DJI3508_ID :uint32_t {
    DJI3508_Begin = 0x201,
    
    DJI3508_1 = 0x201,
    DJI3508_2 = 0x202,
    DJI3508_3 = 0x203,
    DJI3508_4 = 0x204,
    DJI3508_IDLowMax =0x204,
    
    DJI3508_IDHighMin = 0x205,
    DJI3508_5 = 0x205,
    DJI3508_6 = 0x206,
    DJI3508_7 = 0x207,
    DJI3508_8 = 0x208,
    DJI3508_End = 0x209,
};

/**
 * @brief C620反馈CAN消息结构体
 * @details __packed 用于确保结构体在内存中按字节对齐,不添加填充字节
 */
struct __attribute__((__packed__)) C620_FeedBackMsg{
    uint16_t Encoder;      /* 转子角度 单位° 0-8192 映射到0-360° */
    int16_t  RPM;          /* 转子转速 单位RPM */
    int16_t  Current;      /* 电流 单位A 映射到-16384-16384,对应-20A到+20A */
    int8_t   Temperature;  /* 温度 单位C°*/
    uint8_t  Reserved;     /* 保留位 */
};

/**
 * @brief DJI3508电机数据结构体
 * @details 用于存储DJI3508电机的实时数据(这里的数据统一是指输出端)
 */
struct DJI3508_Data{
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
 * @brief DJI3508电机特征参数
 * @details 用于存储DJI3508电机的特征参数
 */
namespace DJI3508_CharacterParam{
    constexpr float32_t GearBoxRate = 3591.0f/187.0f;                         /* 齿轮箱减速比 */
    constexpr float32_t IDLE_GearBoxRPM =  482.0f ;                           /* 空载齿轮箱转速 RPM */
    constexpr float32_t IDLE_GearBoxRad = IDLE_GearBoxRPM *PI /30.0f;         /* 空载齿轮箱转速 Rad/s */
    constexpr float32_t Rated_GearBoxRPM = 469.0f;                            /* 额定齿轮箱转速 RPM */
    constexpr float32_t Rated_GearBoxRad = Rated_GearBoxRPM *PI /30.0f;       /* 额定齿轮箱转速 Rad/s */
    constexpr float32_t IDLE_RotorRPM = IDLE_GearBoxRPM*GearBoxRate;          /* 转子空载转速 */
    constexpr float32_t IDLE_RotorCurrent_A = 0.78f;                          /* 转子空载电流 单位A */
    constexpr float32_t IDLE_RotorCurrent_mA = IDLE_RotorCurrent_A * 1000.0f; /* 转子空载电流 单位mA */
    constexpr float32_t Rated_RotorRPM = Rated_GearBoxRPM*GearBoxRate;        /* 转子额定转速 */ 
    constexpr float32_t Rated_Torque_NM = 3.0f;                               /* 额定转矩,最大连续力矩(N.m) */
    constexpr float32_t Rated_Current_A = 10.0f;                              /* 额定电流 单位A */
    constexpr float32_t Rated_Current_mA = Rated_Current_A * 1000.0f;         /* 额定电流 单位mA */
    constexpr float32_t Rated_Voltage_V = 24.0f;                              /* 额定电压 单位V */
    constexpr float32_t Torque_Constant_NM_A = 0.3f;                          /* 转矩常数 N.m/A */
    constexpr float32_t Speed_Constant_RPM_V = 24.48f;                        /* 速度常数 RPM/V */
    constexpr float32_t Speed_Torque_Gradient =  72.0f;                       /* 速度转矩梯度 (RPM)/N.m */
    constexpr float32_t MaxTemperature = 90.0f;                               /* 最大温度 单位C° */
    constexpr uint16_t  Encoder_Per_Round = 8192;                             /* 编码器最大值 0-8191 映射到0-360° */
    constexpr float32_t Max_C620Current = 20000.0f;                           /* 电调最大控制电流 20000mA*/
};

/*==================== DJI3508 电机类 =====================*/

class DJI3508{
public:

/**
 * @brief 构造函数
 * @param motorID               电机ID,范围为1-8
 * @param canBus                使用的CAN总线
 * @param baudRate              CAN总线波特率   
 * @param Location_PID_Param    位置PID参数
 * @param Speed_PID_Param       速度PID参数
 * @param controlMode           控制模式,默认开环电流模式
 * @param mode                  CAN总线模式,默认正常模式
 */
DJI3508(DJI3508_ID motorID,
        USE_CanBus canBus,
        Can::CanBaudRate baudRate,
        PID_Param& Location_PID_Param,
        PID_Param& Speed_PID_Param,
        DJI3508_ControlMode controlMode =DJI3508_OpenLoopMode,
        Can::CanMode mode = Can::CanMode::MODE_NORMAL 
    );

/**
 * @brief 析构函数
 */
~DJI3508();

/**
 * @brief  初始化DJI3508电机
 * @return MW_Status 初始化结果
 * @note   该函数启动CAN总线资源并设置CAN接受回调函数
 */
MW_Status Init();

/**
 * @brief 启动DJI3508电机控制任务
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
 * @brief 设置DJI3508电机的哪个环路的PID控制器模式
 * @param Loop 环路枚举,用于指定设置哪个环路的PID控制器模式
 * @param D_First_Mode D项系数模式
 * @param I_Limit_Mode I项积分限幅模式
 * @param DeedZone_Mode 死区模式
 * @param I_Separate_Mode I项分离模式
 * @param I_VarSpeed_Mode I项变速模式
 * @param Output_Limit_Mode 输出限幅模式
 * @param FeedForward_Mode 前馈模式
 * @return MW_Status 设置结果
 * @note  该函数用于设置DJI3508电机中指定环路的PID控制器模式
 */
MW_Status SetPIDControllerMode(DJI3508_PID_LOOP Loop,
                            PID_D_First_Mode D_First_Mode, 
                            PID_I_Limit_Mode I_Limit_Mode, 
                            PID_DeedZone_Mode DeedZone_Mode, 
                            PID_I_Separate_Mode I_Separate_Mode, 
                            PID_I_VarSpeed_Mode I_VarSpeed_Mode, 
                            PID_Output_Limit_Mode Output_Limit_Mode,
                            PID_FeedForward_Mode FeedForward_Mode);     

/**
 * @brief 获取C620反馈消息
 * @return C620_FeedBackMsg C620反馈消息结构体
 * @note  该函数用于获取电调C620反馈信息
 */
C620_FeedBackMsg getC620FeedBackMsg() const;

/**
 * @brief 获取DJI3508电机的数据参数
 * @return DJI3508_Data 电机数据参数结构体
 * @note  该函数用于获取电机的实时数据参数
 */
DJI3508_Data getMotorData() const;
private:
/*===================== 私有成员变量 =====================*/ 
 
/* 电机ID,范围为DJI3508_1-DJI3508_8 */
DJI3508_ID MotorID;
/* 使用的CAN总线 */
USE_CanBus CanBus;
/* CAN总线波特率 */
Can::CanBaudRate BaudRate;
/* CAN总线模式 */
Can::CanMode Mode;
/* 电机控制模式,默认开环电流模式 */
DJI3508_ControlMode ControlMode;
/* 硬件资源状态 */
bool Resource_Inited;
/* 电机状态,默认离线 */
volatile DJI3508_Status Status;
/* C620反馈消息结构体 */
volatile C620_FeedBackMsg FeedBackMsg;
/* 电机的数据参数 */
volatile DJI3508_Data MotorData;
/* 指向CanManager的指针,用于申请CAN总线资源 */
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
 * 1. 该函数会向 CanManager 注册 DJI3508_CanMsgCallBack 函数作为静态回调函数
 * 2. 该函数被Init调用
 */
MW_Status SetCallBack();

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
 * @brief 静态实例注册表，存储所有DJI3508实例的指针
 */
static DJI3508* Instance_Registry[DJI3508_End-DJI3508_Begin];

/**
 * @brief 电机3508的CAN消息回调函数
 * @param canId CAN消息ID
 * @param data  指向CAN消息数据的指针
 * @param len   CAN消息数据长度
 * @details
 * 1. 该函数会根据CAN消息ID将数据解析为C620_FeedBackMsg结构体
 * 2. 该函数会将解析出的C620_FeedBackMsg结构体存储在对应的DJI3508实例中
 */
static void DJI3508_CanMsgCallBack(uint32_t canId, uint8_t* data, uint8_t len);

/**
 * @brief  将计算出的控制输出加入到对应的发送CAN消息的结构体中
 * @param Instance 指向DJI3508实例的指针
 * @param controlOut 计算出的控制输出
 */
static void AddControlOutToCanMsg(DJI3508* Instance, int16_t Input_cur);

/**
 * @brief 控制任务函数,用于根据控制模式,控制所有注册的DJI3508电机
 */
static void ControlTask(void* pvParameters); 

};

#endif /* DJI3508_HPP */