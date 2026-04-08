/*===========================================================
* @file      DJI2006.cpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* DJI2006.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* 1.定义并初始化了DJI2006类的静态成员变量
* 2.实现了DJI2006类的成员函数
* ===========================================================
* @version   0.4
* @date      2025-11-27
* @copyright Copyright (c) 2025
============================================================*/

/*========================= 文件依赖 ========================*/
#include "DJI2006.hpp"

/*================= 为DJI2006的静态成员分配内存 ===============*/

/**
 * @brief 控制任务函数句柄，用于指向DJI2006的控制任务函数
 */
TaskHandle_t DJI2006::ControlTaskHandle = nullptr;

/**
 * @brief DJI2006类的实例注册表，用于存储所有已创建的DJI2006实例
 */
DJI2006* DJI2006::Instance_Registry[DJI2006_End-DJI2006_Begin] = {nullptr};

/**
 * @brief 低ID CAN消息发送结构体
 */
CanMessage DJI2006::LowID_CanMsgSend = {
    .id = 0x200,
    .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .len = 8,
    .isExtended = false,
    .isRemote = false,
};

/**
 * @brief 高ID CAN消息发送结构体
 */
CanMessage DJI2006::HighID_CanMsgSend = {
    .id = 0x1FF,
    .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .len = 8,
    .isExtended = false,
    .isRemote = false,
};

/*================== DJI2006的成员函数定义 ==================*/

/*====================== 公有成员函数 ======================*/

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
DJI2006::DJI2006(DJI2006_ID motorID,
        USE_CanBus canBus,
        Can::CanBaudRate baudRate,
        PID_Param& Location_PID_Param,
        PID_Param& Speed_PID_Param,
        DJI2006_ControlMode controlMode,
        Can::CanMode mode):
        Location_PID(Location_PID_Param),
        Speed_PID(Speed_PID_Param)
{
    this->MotorID = motorID;
    this->CanBus = canBus;
    this->BaudRate = baudRate;
    this->Mode = mode;
    this->ControlMode = controlMode;
    memset(&MotorData,0,sizeof(MotorData));
    memset(&FeedBackMsg,0,sizeof(FeedBackMsg));
    /* 初始化资源申请状态为false */
    Resource_Inited = false;
    /* 初始化电机状态为离线 */
    Status = DJI2006_Offline;
    /* 注册电机实例到实例注册表,前提是电机ID在有效范围内 */
    if(MotorID>=DJI2006_Begin&&MotorID<DJI2006_End){
    Instance_Registry[MotorID-DJI2006_Begin] = this;
    }
    /* 绑定CanManager单例 */
    CanManagerPtr = &CanManager::GetInstance();
}

/**
 * @brief 析构函数
 * @note  取消订阅在Can总线上的回调函数,从实例注册表中移除实例，取消绑定CanManager单例
 */
DJI2006::~DJI2006(){
    /* 前提是电机ID在有效范围内 */
    if(MotorID>=DJI2006_Begin&&MotorID<DJI2006_End){   
    /*取消订阅在Can总线上的回调函数*/
    CanManagerPtr->UnSubscribe(CanBus,MotorID,DJI2006_CanMsgCallBack);
    /*从实例注册表中移除实例*/
    Instance_Registry[MotorID-DJI2006_Begin] = nullptr;
    }
    /*取消绑定CanManager单例*/
    CanManagerPtr = nullptr;
}

/**
 * @brief  初始化DJI2006电机
 * @return MW_Status 初始化结果
 * @note   该函数启动CAN总线资源并设置CAN接受回调函数
 */
MW_Status DJI2006::Init(){
    /* 检查电机ID是否正确 */
    if(MotorID<DJI2006_Begin||MotorID>=DJI2006_End){
        DJI2006_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查CAN总线是否存在 */
    if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DJI2006_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
        return MW_Status::INVALID_PARAM;
    }
    MW_Status res = MW_Status::ERROR;
    /* 启动CAN总线资源 */
    res = InitResource();
    if(res == MW_Status::SUCCESS){
        /* 设置CAN消息回调函数 */
        res = SetRouterCallBack();
    }
    return res;
}

/**
 * @brief 启动DJI2006电机控制任务
 * @return MW_Status 启动结果
 * @note   该函数会创建一个任务,用于控制电机(多个电机也依旧是这个任务去管理)
 */
MW_Status DJI2006::StartControlTask(void){
    /* 检查电机ID是否正确 */
    if(MotorID<DJI2006_Begin||MotorID>=DJI2006_End){
        DJI2006_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误,创建控制任务失败");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查CAN总线是否存在 */
    if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DJI2006_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
        return MW_Status::INVALID_PARAM;
    }
    MW_Status res = MW_Status::SUCCESS;
    BaseType_t xTaskCreateRes;
    /* 进入临界区,保证检查和创建任务的原子性 */
    taskENTER_CRITICAL();
    /* 如果控制任务函数句柄为空,则创建控制任务 */
    if(ControlTaskHandle == nullptr)
    {
        /* 启动控制任务 */
        xTaskCreateRes = xTaskCreate(ControlTask, "DJI2006 Control Task", 512, NULL, 24, &ControlTaskHandle);
        if(xTaskCreateRes != pdPASS)
        {
            res = MW_Status::ERROR;
        }    
    }
    /*退出临界区*/
    taskEXIT_CRITICAL();
    return res;
}

/**
 * @brief 依据电机控制模式设置电机期望值
 * @param exp 期望输出参数 
 * @return MW_Status 设置结果
 * @note  1.电机若是位置控制模式，输入位置环期望参数，单位为°
 * @note  2.电机若是速度控制模式，输入速度环期望参数，单位为rad/s
 * @note  3.电机若是开环电流模式，输入电流环期望参数，单位为mA
 * */
MW_Status DJI2006::SetExpect(float32_t exp){
    /* 依据控制模式设置MotorData中的期望输出 */
    switch (ControlMode)
    {
    /** 如果是位置控制模式 */
    case DJI2006_LocationLoopMode:
        MotorData.Exp_Angle = exp;
        MotorData.Exp_Rad = exp * (PI / 180.0f);
        break;
    /** 如果是速度控制模式 */
    case DJI2006_SpeedLoopMode:
        /* 将期望速度限制为减速箱输出的速度范围 */
        Constrain<float32_t>(exp,-DJI2006_CharacterParam::IDLE_GearBoxRad,DJI2006_CharacterParam::IDLE_GearBoxRad);
        MotorData.Exp_Speed = exp;
        break;
    /** 如果是开环电流模式 */
    case DJI2006_OpenLoopMode:
        /* 将期望电流限制为电机输出的电流范围 */
        Constrain<float32_t>(exp,-DJI2006_CharacterParam::Rated_Current_mA,DJI2006_CharacterParam::Rated_Current_mA);
        MotorData.Exp_Current = exp;
        break;
    default:
        break;
    }
    return MW_Status::SUCCESS;
}

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
MW_Status DJI2006::SetPIDControllerMode(DJI2006_PID_LOOP Loop,
                                PID_D_First_Mode D_First_Mode,
                                PID_I_Limit_Mode I_Limit_Mode,
                                PID_DeedZone_Mode DeedZone_Mode,
                                PID_I_Separate_Mode I_Separate_Mode,
                                PID_I_VarSpeed_Mode I_VarSpeed_Mode,
                                PID_Output_Limit_Mode Output_Limit_Mode,
                                PID_FeedForward_Mode FeedForward_Mode)
{
    MW_Status res = MW_Status::ERROR;
    /* 如果发现环路枚举大于控制模式,则返回错误 */
    if( (uint8_t)Loop > (uint8_t)ControlMode)
    {
        DJI2006_ASSERT(res,"设置PID控制器模式失败,环路枚举大于本身的控制模式");
        return res;
    }else{
        /* 根据给定的Loop设置环路模式 */
        switch (Loop)
        {
        case DJI2006_LocationPIDLoop:
            Location_PID.Set_Mode(D_First_Mode, I_Limit_Mode, DeedZone_Mode, I_Separate_Mode, I_VarSpeed_Mode, Output_Limit_Mode, FeedForward_Mode);
            break;
        case DJI2006_SpeedPIDLoop:
            Speed_PID.Set_Mode(D_First_Mode, I_Limit_Mode, DeedZone_Mode, I_Separate_Mode, I_VarSpeed_Mode, Output_Limit_Mode, FeedForward_Mode);
            break;
        }
        
        res = MW_Status::SUCCESS;
    }
    return res;
}

/**
 * @brief 获取DJI2006电机的C610反馈信息
 * @return C610_FeedBackMsg C610反馈信息结构体(只读引用)
 * @note  该函数用于获取电调C610反馈信息
 */
const C610_FeedBackMsg& DJI2006::getC610FeedBackMsg(){
    return this->FeedBackMsg;
}

/**
 * @brief 获取DJI2006电机的数据参数
 * @return DJI2006_Data 电机数据参数结构体(只读引用)
 * @note  该函数用于获取电机的实时数据参数
 */
const DJI2006_Data& DJI2006::getMotoData(){
    return this->MotorData;
}

/*====================== 私有成员函数 =======================*/

/**
 * @brief  向CAN总线发送CAN消息(用于测试，直接发送消息,测试的时候放置到public即可)
 * @param canMsg 要发送的CAN消息
 * @return MW_Status 发送结果
 * @details 向CAN总线发送CAN消息,根据电机ID选择低ID或高ID的CAN消息发送结构体
 */
MW_Status DJI2006::SendCanMsg(int16_t current)
{  
   /* 检查电机ID是否正确 */
   if(MotorID<DJI2006_Begin||MotorID>=DJI2006_End){
        DJI2006_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
   }
   /* 检查CAN总线是否存在 */
   if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DJI2006_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
        return MW_Status::INVALID_PARAM;
   }
   /* 如果CAN总线资源未初始化,则返回错误 */
   if(!Resource_Inited)
   {
        DJI2006_ASSERT(MW_Status::ERROR,"Can发送失败,CAN总线资源未初始化");
        return MW_Status::ERROR;
   }
   /* 拆分电流指令为高字节和低字节 */
   uint8_t current_high = (uint8_t)(current >> 8);
   uint8_t current_low = (uint8_t)(current);
   /* 计算当前电机ID对应的低ID索引 */
   uint8_t data_offset;
   if(this->MotorID<=DJI2006_IDLowMax)
   {
        data_offset = (MotorID-DJI2006_Begin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        LowID_CanMsgSend.data[data_offset] = current_high;
        LowID_CanMsgSend.data[data_offset+1] = current_low;
        return CanManagerPtr->sendMessage(CanBus, LowID_CanMsgSend);
   }
   else
   {
        data_offset = (MotorID-DJI2006_IDHighMin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        HighID_CanMsgSend.data[data_offset] = current_high;
        HighID_CanMsgSend.data[data_offset+1] = current_low;
        return CanManagerPtr->sendMessage(CanBus, HighID_CanMsgSend);
   }
}

/**
 * @brief  DJI2006电机初始化函数,申请CAN总线资源
 * @return MW_Status 初始化状态
 * @note   被Init调用,申请CAN总线资源，Init已经保证电机ID和CAN总线的有效性
 */
MW_Status DJI2006::InitResource()
{  
    /* 申请CAN总线资源 */
    MW_Status res = CanManagerPtr->AskResource(CanBus, BaudRate, Mode);

    /* 检查CAN总线资源申请结果 */
    DJI2006_ASSERT(res,"申请CAN总线资源失败,检查CAN总线配置和之前的配置是否冲突");
    /* 如果申请成功,则更新资源申请状态 */
    if(res == MW_Status::SUCCESS)
    {
        res = CanManagerPtr->StartResource(CanBus);
        /* 检查CAN总线资源启动结果 */
        DJI2006_ASSERT(res,"启动CAN总线资源失败");
        /* 如果启动成功,则更新资源申请状态 */
        if(res == MW_Status::SUCCESS)
        {
            /* 硬件资源初始化成功 */
            Resource_Inited = true;
        }
    }
    return res;
}

/**
 * @brief  向CANManager注册CAN消息回调函数
 * @return MW_Status 设置结果
 * @note   被Init调用,设置CAN消息回调函数，Init已经保证电机ID和CAN总线的有效性
 */
MW_Status DJI2006::SetRouterCallBack(){
    MW_Status res = MW_Status::ERROR;
    /* 检查CAN总线资源是否已初始化 */
    if(Resource_Inited == true)
    {
        /* 订阅CAN总线,向CANManger 注册CAN消息的回调函数,用于处理DJI2006电机返回的CAN消息 */
        res = CanManagerPtr->Subscribe(CanBus, MotorID,DJI2006_CanMsgCallBack);
        /* 检查CAN消息回调函数订阅结果 */
        DJI2006_ASSERT(res,"设置CAN消息回调函数失败,检查CAN总线资源是否已初始化且配置不同,或者回调数组已满");
    }
    return res;
}

/**
 * @brief  将计算出的控制输出加入到对应的发送CAN消息的结构体中
 * @param Instance 指向DJI3508实例的指针
 * @param controlOut 计算出的控制输出电流值
 * @note   被ControlTask调用,注册表已经保证了电机ID的有效性
 */
void DJI2006::AddControlOutToCanMsg(DJI2006* Instance, int16_t Input_cur)
{  
   DJI2006_ID MotorID = Instance->MotorID;
   /* 拆分电流指令为高字节和低字节 */
   uint8_t current_high = (uint8_t)(Input_cur >> 8);
   uint8_t current_low = (uint8_t)(Input_cur);
   /* 计算当前电机ID对应的低ID索引 */
   uint8_t data_offset;
   if(MotorID<=DJI2006_IDLowMax)
   {
        data_offset = (MotorID-DJI2006_Begin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        LowID_CanMsgSend.data[data_offset] = current_high;
        LowID_CanMsgSend.data[data_offset+1] = current_low;
   }
   else
   {
        data_offset = (MotorID-DJI2006_IDHighMin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        HighID_CanMsgSend.data[data_offset] = current_high;
        HighID_CanMsgSend.data[data_offset+1] = current_low;
   }
}

/*====================== 静态成员变量和函数 ======================*/

/**
 * @brief DJI3508电机的CAN消息回调函数
 * @param canId 收到的CAN消息ID
 * @param data 收到的CAN消息数据指针
 * @param len 收到的CAN消息数据长度
 * @details 根据反馈的C620消息，刷新电机数据
 */
void DJI2006::DJI2006_CanMsgCallBack(uint32_t canId, uint8_t* data, uint8_t len){
    /* 检查CAN消息数据是否有效 */ 
    if(data == nullptr||len != 8)
     {
        return;
     }
     /* 寻找当前电机实例,根据CAN消息ID确定电机实例 */
     DJI2006* temp = Instance_Registry[canId - DJI2006_Begin];
     if(temp != nullptr)
     {
        /* 关闭中断,确保数据更新时不被中断打断 */
        __disable_irq();
        /* 赋值C610反馈消息结构体 */
        temp->FeedBackMsg.Encoder = (uint16_t)(data[0] << 8 | data[1]);
        temp->FeedBackMsg.RPM = (uint16_t)(data[2] << 8 | data[3]);
        temp->FeedBackMsg.Current = (int16_t)(data[4] << 8 | data[5]);
        temp->FeedBackMsg.Reserved = (uint16_t)(data[6]<<8 | data[7] );

        /* 刷新电机数据 */
        int16_t Delta_Encoder = temp->FeedBackMsg.Encoder - temp->MotorData.Pre_Encoder;
        if(Delta_Encoder < -DJI2006_CharacterParam::Encoder_Per_Round/2){
            /* 正方向转了一圈 */
            temp->MotorData.Total_RotorRound++;
        }else if(Delta_Encoder > DJI2006_CharacterParam::Encoder_Per_Round/2){
            /* 反方向转了一圈 */
            temp->MotorData.Total_RotorRound--;
        }
        /* 总的编码器计数 = 总圈数 * 每圈编码器数 + 当前编码器值 */
        temp->MotorData.Total_Encoder =  temp->MotorData.Total_RotorRound* DJI2006_CharacterParam::Encoder_Per_Round + temp->FeedBackMsg.Encoder;
        /* 更新上一次的编码器值 = 当前编码器值 */
        temp->MotorData.Pre_Encoder = temp->FeedBackMsg.Encoder;
        /* 计算电机输出轴减速箱输出的速度 = 电机速度反馈值(rad/s) / 减速比 */
        temp->MotorData.Now_Speed = RPM2Rad((float)temp->FeedBackMsg.RPM)/DJI2006_CharacterParam::GearBoxRate;
        /* 计算电机电流(毫安) = 电机电流反馈值 (-16384 - +16384)转成(-10000 - +10000毫安) */
        temp->MotorData.Now_Current = Int16ToFloat(temp->FeedBackMsg.Current,-16384,16384,-DJI2006_CharacterParam::Max_C610Current,DJI2006_CharacterParam::Max_C610Current);
        /* 电机输出轴当前角度 = 总的编码器计数 / 每圈编码器数 * 360° /减速比 */
        temp->MotorData.Now_Angle = (float)temp->MotorData.Total_Encoder/(float)DJI2006_CharacterParam::Encoder_Per_Round*360.0f/DJI2006_CharacterParam::GearBoxRate;    
        /* 电机输出轴当前弧度 = 电机输出轴当前角度 * π / 180.0 */
        temp->MotorData.Now_Rad = temp->MotorData.Now_Angle * (PI / 180.0f);
        /* 电机在线统计重置为10,表示10ms内未收到反馈,则认为电机离线 */
        temp->MotorData.Online_CountFlag = 10;
        /* 收到消息,说明电机在线,重置电机状态 */
        temp->Status = DJI2006_Online;        
        /* 退出临界区 */
        __enable_irq();
    }
}

/**
 * @brief  控制任务函数,用于处理DJI2006电机的控制指令
 * @details  1. 从实例注册表中遍历所有已创建的DJI2006实例。
 *           2. 检查实例是否为nullptr,如果不为空且电机状态正常就执行数据运算任务
 *           3. 最后统一发送CAN消息
 *           4. 位置环单位rad，速度环单位rad/s，电流环单位mA
 */
void DJI2006::ControlTask(void* pvParameters){
    /* 任务运行频率为1ms */
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    /* 任务第一次运行时,获取当前时间 */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    /* 定义DJI2006电机实例指针(临时变量) */
    DJI2006* Instance = nullptr;
    DJI2006_ID MotorID = DJI2006_Begin;
    /*期望输出*/
    float32_t expect = 0.0f;
    /*反馈输出*/
    float32_t feedback = 0.0f;
    /*控制器输出*/
    float32_t controlOut = 0.0f;
    /*输入到电机的电流 */
    int16_t Input_cur = 0;
    /*需要在哪路CAN总线上发送CAN消息 */
    bool Can1NeedLowID = false;
    bool Can1NeedHighID = false;
    bool Can2NeedLowID = false;
    bool Can2NeedHighID = false;
    /*电机数据快照*/
    DJI2006_Data motorDataSnapshot;

    while(1){
        /* 刷新状态 */
        Can1NeedLowID = false;
        Can1NeedHighID = false;
        Can2NeedLowID = false;
        Can2NeedHighID = false;
        memset(LowID_CanMsgSend.data,0,sizeof(LowID_CanMsgSend.data));
        memset(HighID_CanMsgSend.data,0,sizeof(HighID_CanMsgSend.data));

        /* 遍历所有已经创建的DJI2006实例,每层循环执行PID运算任务并把其值加入到发送CAN消息的结构体中 */
        for(uint32_t i = DJI2006_Begin;i<DJI2006_End;i++)
        {
            /* 检查当前实例是否为nullptr */            
            Instance = Instance_Registry[i-DJI2006_Begin];
            /* 如果实例不为空*/
            if(Instance != nullptr)
            {   
                /* 原子操作: 检查电机状态, 处理离线, 并获取数据快照 */
                taskENTER_CRITICAL();
                /* 检查电机状态 */
                if (Instance->Status == DJI2006_Online)
                {         
                    /* 电机在线统计减一,消费者*/
                    Instance->MotorData.Online_CountFlag--;
                    if (Instance->MotorData.Online_CountFlag <= 0)
                    {
                        /* 电机刚刚掉线 */
                        Instance->Status = DJI2006_Offline;
                        AddControlOutToCanMsg(Instance, (int16_t)0x0000);
                        taskEXIT_CRITICAL();
                        continue;
                    }
                    /* 电机在线, 获取数据快照用于计算 */
                    motorDataSnapshot = Instance->MotorData;
                }
                /* 如果电机离线, 跳过处理*/
                else
                {
                    /* 电机离线, 跳过 */
                    taskEXIT_CRITICAL();
                    continue;
                }
                taskEXIT_CRITICAL();
                /* 获取电机ID */
                MotorID = Instance->MotorID;
                /* 检查当前实例是否需要在CAN1总线上发送CAN消息 */
                if(Instance->CanBus == USE_CAN1)
                {
                    if(MotorID <= DJI2006_IDLowMax)
                        Can1NeedLowID = true;
                    else
                        Can1NeedHighID = true;
                }
                /* 检查当前实例是否需要在CAN2总线上发送CAN消息 */
                else if(Instance->CanBus == USE_CAN2)
                {
                    if(MotorID >= DJI2006_IDHighMin)
                        Can2NeedHighID = true;
                    else
                        Can2NeedLowID = true;
                }

                /* 依据当前电机快照执行数据运算任务，计算出最终的输出 */
                switch (Instance->ControlMode)
                {
                /* 位置模式 */
                case DJI2006_LocationLoopMode:
                    /* 获取期望角度 */
                    expect = motorDataSnapshot.Exp_Rad;
                    /* 获取当前角度 */
                    feedback = motorDataSnapshot.Now_Rad;
                    /* 计算位置PID输出期望速度 */
                    controlOut = Instance->Location_PID.Calculate(expect,feedback);
                    /* 强制限制期望速度在最大速度范围内 */
                    Constrain<float32_t>(controlOut,-DJI2006_CharacterParam::IDLE_GearBoxRad, DJI2006_CharacterParam::IDLE_GearBoxRad);  
                    /* 写入期望速度到快照 */
                    motorDataSnapshot.Exp_Speed = controlOut;
                /* 速度模式 */
                case DJI2006_SpeedLoopMode:
                    /* 获取期望速度 */
                    expect = motorDataSnapshot.Exp_Speed;
                    /* 获取当前速度 */
                    feedback = motorDataSnapshot.Now_Speed;
                    /* 计算速度PID输出期望电流 */
                    controlOut = Instance->Speed_PID.Calculate(expect,feedback);
                    /* 写入期望电流到快照 */
                    motorDataSnapshot.Exp_Current = controlOut;
                /* 开环模式 */
                case DJI2006_OpenLoopMode:
                    /* 开环模式直接将期望电流作为输出 */
                    controlOut = motorDataSnapshot.Exp_Current;
                    /* 强制限制输出电流在额定电流范围 */
                    Constrain<float32_t>(controlOut,-DJI2006_CharacterParam::Rated_Current_mA, DJI2006_CharacterParam::Rated_Current_mA);  
                    break;                
                default:
                    break;
                }
                /* 原子操作,更新电机状态 */
                taskENTER_CRITICAL();
                switch (Instance->ControlMode)
                {
                /* 位置模式 */
                case DJI2006_LocationLoopMode:
                    /* 开环模式直接将期望电流作为输出 */
                    Instance->MotorData.Exp_Speed =motorDataSnapshot.Exp_Speed;
                /* 速度模式 */
                case DJI2006_SpeedLoopMode:
                    Instance->MotorData.Exp_Current =motorDataSnapshot.Exp_Current;
                /* 开环模式下，Exp_Current 是输入，无需写回 */
                case DJI2006_OpenLoopMode:
                    break;
                default:
                    break;
                }    
                taskEXIT_CRITICAL();
                /* 将期望输出转换为输入到电机的电流 */
                Input_cur = FloatToInt16(controlOut,-DJI2006_CharacterParam::Max_C610Current,DJI2006_CharacterParam::Max_C610Current,-16384,16384);
                AddControlOutToCanMsg(Instance,Input_cur);
            }
        }

        /*最后将电机控制指令发送到CanManager*/
        if(Can1NeedLowID)
        {
            CanManager::GetInstance().sendMessage(USE_CAN1, LowID_CanMsgSend);
        }
        if(Can1NeedHighID)
        {
            CanManager::GetInstance().sendMessage(USE_CAN1, HighID_CanMsgSend);
        }
        if(Can2NeedLowID)
        {
            CanManager::GetInstance().sendMessage(USE_CAN2, LowID_CanMsgSend);
        }
        if(Can2NeedHighID)  
        {
            CanManager::GetInstance().sendMessage(USE_CAN2, HighID_CanMsgSend);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}