/*===========================================================
* @file      DJI3508.cpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* DJI3508.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* 1. 定义并初始化了DJI3508类的静态成员变量(任务句柄、实例注册表、CAN发送消息结构体)
* 2. 实现了DJI3508类的成员函数,包括构造/析构、初始化、控制任务启动、期望设置、PID模式设置等
* 3. 实现了CAN消息回调函数(DJI3508_CanMsgCallBack),用于解析C620电调反馈数据并更新电机状态
* 4. 实现了电机控制任务(ControlTask),负责周期性地执行PID闭环计算、状态管理及CAN指令发送
* 5. 提供了私有辅助函数,如资源初始化(InitResource)、回调注册(SetCallBack)、CAN消息构建(AddControlOutToCanMsg)等
* ===========================================================
* @version   1.8
* @date      2025-12-14
* @copyright Copyright (c) 2025
============================================================*/

/*========================= 文件依赖 ========================*/
#include "DJI3508.hpp"
/*================= 为DJI3508的静态成员分配内存 ===============*/

/**
 * @brief 控制任务函数句柄,用于指向DJI3508的控制任务函数
 */
TaskHandle_t DJI3508::ControlTaskHandle = nullptr;

/**
 * @brief  DJI3508类的实例注册表，用于存储所有已创建的DJI3508实例
 */
DJI3508* DJI3508::Instance_Registry[DJI3508_End-DJI3508_Begin] = {nullptr};

/**
 * @brief 低ID CAN消息发送结构体
 */
CanMessage DJI3508::LowID_CanMsgSend = {
    .id = 0x200,
    .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .len = 8,
    .isExtended = false,
    .isRemote = false,
};

/**
 * @brief 高ID CAN消息发送结构体
 */
CanMessage DJI3508::HighID_CanMsgSend = {
    .id = 0x1FF,
    .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .len = 8,
    .isExtended = false,
    .isRemote = false,
};

/*================== DJI3508的成员函数定义 ==================*/

/*====================== 公有成员函数 ======================*/

/**
 * @brief 构造函数
 * @param motorID 电机ID
 * @param canBus 使用的CAN总线
 * @param baudRate CAN总线波特率
 * @param Location_PID_Param 位置PID参数
 * @param Speed_PID_Param 速度PID参数
 * @param mode CAN总线模式
 * @param controlMode 控制模式,默认开环电流模式
 * @details 对DJI3508电机的成员变量进行初始化，向静态实例表注册该实例
 */
DJI3508::DJI3508(DJI3508_ID motorID, 
                 USE_CanBus canBus,
                 Can::CanBaudRate baudRate,
                 PID_Param& Location_PID_Param,
                 PID_Param& Speed_PID_Param,
                 DJI3508_ControlMode controlMode,
                 Can::CanMode mode
                 ):
                 Location_PID(Location_PID_Param),
                 Speed_PID(Speed_PID_Param)
{   
    
    this->MotorID = motorID;
    this->CanBus = canBus;
    this->BaudRate = baudRate;
    this->Mode = mode;
    this->ControlMode = controlMode;
    memset(const_cast<void*>(static_cast<volatile void*>(&MotorData)),0,sizeof(MotorData));
    memset(const_cast<void*>(static_cast<volatile void*>(&FeedBackMsg)),0,sizeof(FeedBackMsg));
    /* 初始化资源申请状态为false */
    Resource_Inited = false;
    /* 初始化电机状态为离线 */
    Status = DJI3508_Offline;
    /* 注册电机实例到实例注册表,前提是电机ID在有效范围内 */
    if(MotorID>=DJI3508_Begin&&MotorID<DJI3508_End){
    Instance_Registry[MotorID - DJI3508_Begin] = this;
    }
    /* 绑定CanManager单例 */
    CanManagerPtr = &CanManager::GetInstance();
}

/**
 * @brief 析构函数
 * @note  取消订阅在Can总线上的回调函数，从实例注册表中移除该实例，取消绑定CanManager单例
 */
DJI3508::~DJI3508()
{   
    /* 前提是电机ID在有效范围内 */
    if(MotorID>=DJI3508_Begin&&MotorID<DJI3508_End){
    /*取消订阅在Can总线上的回调函数*/
    CanManagerPtr->UnSubscribe(CanBus,MotorID,DJI3508_CanMsgCallBack);
    /* 从实例注册表中移除该实例 */
    Instance_Registry[MotorID - DJI3508_Begin] = nullptr;
    }
    /* 取消绑定CanManager单例 */
    CanManagerPtr = nullptr;
}

/**
 * @brief 初始化DJI3508电机
 * @return MW_Status 初始化结果
 * @details 申请CAN总线资源,设置CAN消息回调函数
 */
MW_Status DJI3508::Init(){
    /* 检查电机ID是否正确 */
    if(MotorID<DJI3508_Begin||MotorID>=DJI3508_End){
        DJI3508_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查CAN总线是否存在 */
    if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DJI3508_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
        return MW_Status::INVALID_PARAM;
    }
    MW_Status res = MW_Status::ERROR;
    /* 启动CAN总线资源 */
    res = InitResource();
    if(res == MW_Status::SUCCESS)
    {
        /* 设置CAN消息回调函数 */
        res = SetCallBack();
    }
    return res;
}

/**
 * @brief 启动DJI3508电机控制任务
 * @details 启动DJI3508电机控制任务,任务中调用控制任务函数
 */
MW_Status DJI3508::StartControlTask(void)
{   
    /* 检查电机ID是否正确 */
    if(MotorID<DJI3508_Begin||MotorID>=DJI3508_End){
        DJI3508_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查CAN总线是否存在 */
    if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DJI3508_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
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
        xTaskCreateRes = xTaskCreate(ControlTask, "DJI3508 Control Task", 128, NULL, 24, &ControlTaskHandle);
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
 * @brief 设置DJI3508电机的期望输出，这里的期望输出是设置的最外环的期望输出
 * @param exp 期望输出参数
 * @return MW_Status 设置结果
 * @note  1.电机若是位置控制模式，输入位置环期望参数，单位为°
 * @note  2.电机若是速度控制模式，输入速度环期望参数，单位为rad/s
 * @note  3.电机若是开环电流模式，输入电流环期望参数，单位为mA
 */
MW_Status DJI3508::SetExpect(float32_t exp)
{   
    /* 依据控制模式设置MotorData中的期望输出 */
    switch (ControlMode)
    {
    /** 如果是位置控制模式 */
    case DJI3508_LocationLoopMode:
        MotorData.Exp_Angle = exp;
        MotorData.Exp_Rad = exp * (PI / 180.0f);
        break;
    /** 如果是速度控制模式 */
    case DJI3508_SpeedLoopMode:
        /* 将期望速度限制为减速箱输出的速度范围 */
        Constrain<float32_t>(exp,-DJI3508_CharacterParam::IDLE_GearBoxRad,DJI3508_CharacterParam::IDLE_GearBoxRad);
        MotorData.Exp_Speed = exp;
        break;
    /** 如果是开环电流模式 */
    case DJI3508_OpenLoopMode:
        /* 对电流进行限制，保护措施 */
        Constrain<float32_t>(exp,-DJI3508_CharacterParam::Rated_Current_mA,DJI3508_CharacterParam::Rated_Current_mA);
        MotorData.Exp_Current = exp;
        break;
    default:
        break;
    }
    return MW_Status::SUCCESS;
}

/**
 * @brief 设置DJI3508电机的哪个环路的PID控制器模式
 * @param Loop 环路枚举,用于指定设置哪个环路的PID控制器模式
 * @param D_First_Mode D项系数模式
 * @param I_Limit_Mode I项限制模式
 * @param DeedZone_Mode 死区模式
 * @param I_Separate_Mode I项分离模式
 * @param I_VarSpeed_Mode I项变速模式
 * @param Output_Limit_Mode 输出限制模式
 * @param FeedForward_Mode 前馈模式
 * @return MW_Status 设置结果
 */
MW_Status DJI3508::SetPIDControllerMode(DJI3508_PID_LOOP Loop,
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
        DJI3508_ASSERT(res,"设置PID控制器模式失败,环路枚举大于本身的控制模式");
        return res;
    }else{
        /* 根据给定的Loop设置环路模式 */
        switch (Loop)
        {
        case DJI3508_LocationPIDLoop:
            Location_PID.Set_Mode(D_First_Mode, I_Limit_Mode, DeedZone_Mode, I_Separate_Mode, I_VarSpeed_Mode, Output_Limit_Mode, FeedForward_Mode);
            break;
        case DJI3508_SpeedPIDLoop:
            Speed_PID.Set_Mode(D_First_Mode, I_Limit_Mode, DeedZone_Mode, I_Separate_Mode, I_VarSpeed_Mode, Output_Limit_Mode, FeedForward_Mode);
            break;
        }
        
        res = MW_Status::SUCCESS;
    }
    return res;
}

/**
 * @brief 获取C620反馈消息
 * @return C620_FeedBackMsg C620反馈消息结构体
 * @note  该函数用于获取电调C620反馈信息
 */
C620_FeedBackMsg DJI3508::getC620FeedBackMsg()const{
    C620_FeedBackMsg temp;
    /* 记录当前中断状态 */
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    /* 安全地拷贝 volatile 数据到临时变量 */
    temp = const_cast<const C620_FeedBackMsg&>(this->FeedBackMsg);
    /* 退出临界区（恢复中断）*/
    if (!primask_bit) {
        __enable_irq();
    }
    /* 返回副本，外部怎么用都安全 */
    return temp; 
};

/**
 * @brief 获取DJI3508电机的数据参数
 * @return DJI3508_Data 电机数据参数结构体
 * @note  该函数用于获取电机的实时数据参数
 */
DJI3508_Data DJI3508::getMotorData() const{
    DJI3508_Data temp;
    /* 记录当前中断状态 */
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    /* 安全地拷贝 volatile 数据到临时变量 */
    temp = const_cast<const DJI3508_Data&>(this->MotorData);
    /* 退出临界区（恢复中断）*/
    if (!primask_bit) {
        __enable_irq();
    }
    /* 返回副本，外部怎么用都安全 */
    return temp; 
};

/*====================== 私有成员函数 =======================*/

/**
 * @brief  向CAN总线发送CAN消息(用于测试，直接发送消息,测试的时候放置到public即可)
 * @param canMsg 要发送的CAN消息
 * @return MW_Status 发送结果
 * @details 向CAN总线发送CAN消息,根据电机ID选择低ID或高ID的CAN消息发送结构体
 */
MW_Status DJI3508::SendCanMsg(int16_t current)
{  
   /* 检查电机ID是否正确 */
   if(MotorID<DJI3508_Begin||MotorID>=DJI3508_End){
        DJI3508_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
   }
   /* 检查CAN总线是否存在 */
   if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DJI3508_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
        return MW_Status::INVALID_PARAM;
   }
   /* 如果CAN总线资源未初始化,则返回错误 */
   if(!Resource_Inited)
   {
        DJI3508_ASSERT(MW_Status::ERROR,"Can发送失败,CAN总线资源未初始化");
        return MW_Status::ERROR;
   }
   if(ControlTaskHandle!= nullptr){
        DJI3508_ASSERT(MW_Status::ERROR,"检测到开启了任务,不能直接发送CAN消息");
        return MW_Status::ERROR;
   }
   /* 拆分电流指令为高字节和低字节 */
   uint8_t current_high = (uint8_t)(current >> 8);
   uint8_t current_low = (uint8_t)(current);
   /* 计算当前电机ID对应的低ID索引 */
   uint8_t data_offset;
   if(this->MotorID<=DJI3508_IDLowMax)
   {
        data_offset = (MotorID-DJI3508_Begin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        LowID_CanMsgSend.data[data_offset] = current_high;
        LowID_CanMsgSend.data[data_offset+1] = current_low;
        return CanManagerPtr->sendMessage(CanBus, LowID_CanMsgSend);
   }
   else
   {
        data_offset = (MotorID-DJI3508_IDHighMin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        HighID_CanMsgSend.data[data_offset] = current_high;
        HighID_CanMsgSend.data[data_offset+1] = current_low;
        return CanManagerPtr->sendMessage(CanBus, HighID_CanMsgSend);
   }
}

/**
 * @brief  DJI3508电机CAN初始化函数,申请CAN总线资源
 * @return MW_Status 初始化状态
 * @note   被Init调用,申请CAN总线资源，Init已经保证电机ID和CAN总线的有效性
 */
MW_Status DJI3508::InitResource()
{   
    /* 申请CAN总线资源 */
    MW_Status res = CanManagerPtr->AskResource(CanBus, BaudRate, Mode);

    /* 检查CAN总线资源申请结果 */
    DJI3508_ASSERT(res,"申请CAN总线资源失败,检查CAN总线配置和之前的配置是否冲突");
    /* 如果申请成功,则启动CAN总线资源 */
    if(res == MW_Status::SUCCESS)
    {
        res = CanManagerPtr->StartResource(CanBus);
        /* 检查CAN总线资源启动结果 */
        DJI3508_ASSERT(res,"启动CAN总线资源失败");
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
MW_Status DJI3508::SetCallBack(){
    MW_Status res = MW_Status::ERROR;
    /* 检查CAN总线资源是否已初始化 */
    if(Resource_Inited == true)
    {
        /* 订阅CAN总线,向CanManager 注册CAN消息的回调函数,用于处理DJI3508电机返回的CAN消息 */
        res = CanManagerPtr->Subscribe(CanBus, MotorID,DJI3508_CanMsgCallBack);
        /* 检查CAN消息回调函数订阅结果 */
        DJI3508_ASSERT(res,"设置CAN消息回调函数失败,检查CAN总线资源是否已初始化且配置不同,或者回调数组已满");
    }
    return res;
}

/**
 * @brief  将计算出的控制输出加入到对应的发送CAN消息的结构体中
 * @param Instance 指向DJI3508实例的指针
 * @param controlOut 计算出的控制输出电流值
 * @note   被ControlTask调用,注册表已经保证了电机ID的有效性
 */
void DJI3508::AddControlOutToCanMsg(DJI3508* Instance, int16_t Input_cur)
{  
   DJI3508_ID MotorID = Instance->MotorID;
   /* 拆分电流指令为高字节和低字节 */
   uint8_t current_high = (uint8_t)(Input_cur >> 8);
   uint8_t current_low = (uint8_t)(Input_cur);
   /* 计算当前电机ID对应的低ID索引 */
   uint8_t data_offset;
   if(MotorID<=DJI3508_IDLowMax)
   {
        data_offset = (MotorID-DJI3508_Begin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        LowID_CanMsgSend.data[data_offset] = current_high;
        LowID_CanMsgSend.data[data_offset+1] = current_low;
   }
   else
   {
        data_offset = (MotorID-DJI3508_IDHighMin)*2;
        /* 电流指令高字节在低地址,低字节在高地址 */
        HighID_CanMsgSend.data[data_offset] = current_high;
        HighID_CanMsgSend.data[data_offset+1] = current_low;
   }
}

/*======================== 静态函数 ========================*/

/**
 * @brief DJI3508电机的CAN消息回调函数
 * @param canId 收到的CAN消息ID
 * @param data 收到的CAN消息数据指针
 * @param len 收到的CAN消息数据长度
 * @details 根据反馈的C620消息，刷新电机数据
 */
void DJI3508::DJI3508_CanMsgCallBack(uint32_t canId, uint8_t* data, uint8_t len){
    /* 检查CAN消息数据是否有效 */ 
    if(data == nullptr||len != 8)
     {
        return;
     }
     /* 寻找当前电机实例,根据CAN消息ID确定电机实例 */
     DJI3508* temp = Instance_Registry[canId - DJI3508_Begin];
     if(temp != nullptr)
     {
        /* 关闭中断,确保数据更新时不被中断打断 */
        __disable_irq();
        /* 赋值C620反馈消息结构体 */
        temp->FeedBackMsg.Encoder = (uint16_t)(data[0] << 8 | data[1]);
        temp->FeedBackMsg.RPM = (uint16_t)(data[2] << 8 | data[3]);
        temp->FeedBackMsg.Current = (int16_t)(data[4] << 8 | data[5]);
        temp->FeedBackMsg.Temperature = data[6];
        temp->FeedBackMsg.Reserved = data[7];

        /* 刷新电机数据 */
        int16_t Delta_Encoder = temp->FeedBackMsg.Encoder - temp->MotorData.Pre_Encoder;
        if(Delta_Encoder < -DJI3508_CharacterParam::Encoder_Per_Round/2){
            /* 正方向转了一圈 */
            temp->MotorData.Total_RotorRound++;
        }else if(Delta_Encoder > DJI3508_CharacterParam::Encoder_Per_Round/2){
            /* 反方向转了一圈 */
            temp->MotorData.Total_RotorRound--;
        }
        /* 总的编码器计数 = 总圈数 * 每圈编码器数 + 当前编码器值 */
        temp->MotorData.Total_Encoder =  temp->MotorData.Total_RotorRound* DJI3508_CharacterParam::Encoder_Per_Round + temp->FeedBackMsg.Encoder;
        /* 更新上一次的编码器值 = 当前编码器值 */
        temp->MotorData.Pre_Encoder = temp->FeedBackMsg.Encoder;
        /* 计算电机输出轴减速箱输出的速度 = 电机速度反馈值(rad/s) / 减速比 */
        temp->MotorData.Now_Speed = RPM2Rad((float)temp->FeedBackMsg.RPM)/DJI3508_CharacterParam::GearBoxRate;
        /* 计算电机电流(毫安) = 电机电流反馈值 (-16384 - +16384)转成(-20000 - +20000毫安) */
        temp->MotorData.Now_Current = Int16ToFloat(temp->FeedBackMsg.Current,-16384,16384,-DJI3508_CharacterParam::Max_C620Current,DJI3508_CharacterParam::Max_C620Current);
        /* 电机输出轴当前角度 = 总的编码器计数 / 每圈编码器数 * 360° /减速比 */
        temp->MotorData.Now_Angle = (float)temp->MotorData.Total_Encoder/(float)DJI3508_CharacterParam::Encoder_Per_Round*360.0f/DJI3508_CharacterParam::GearBoxRate;    
        /* 电机输出轴当前弧度 = 电机输出轴当前角度 * π / 180.0 */
        temp->MotorData.Now_Rad = temp->MotorData.Now_Angle * (PI / 180.0f);
        /* 电机在线统计重置为10,表示10ms内未收到反馈,则认为电机离线 */
        temp->MotorData.Online_CountFlag = 10;
        /* 收到消息,说明电机在线,重置电机状态 */
        temp->Status = temp->Status!=DJI3508_OverTemperature?DJI3508_Online:temp->FeedBackMsg.Temperature<=DJI3508_CharacterParam::MaxTemperature-10?DJI3508_Online:DJI3508_OverTemperature;        
        /* 退出临界区 */
        __enable_irq();
    }
}

/**
 * @brief  控制任务函数,用于处理DJI3508电机的控制指令
 * @details  1. 从实例注册表中遍历所有已创建的DJI3508实例。
 *           2. 检查实例是否为nullptr,如果不为空且电机状态正常就执行数据运算任务
 *           3. 最后统一发送CAN消息
 *           4. 位置环单位rad，速度环单位rad/s，电流环单位mA
 */
void DJI3508::ControlTask(void* pvParameters){
    /* 任务运行频率为1ms */
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    /* 任务第一次运行时,获取当前时间 */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    /* 定义DJI3508电机实例指针(临时变量) */
    DJI3508* Instance = nullptr;
    DJI3508_ID MotorID = DJI3508_Begin;
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
    DJI3508_Data motorDataSnapshot;
    
    while(1){
        /* 刷新状态 */
        Can1NeedLowID = false;
        Can1NeedHighID = false;
        Can2NeedLowID = false;
        Can2NeedHighID = false;
        memset(LowID_CanMsgSend.data,0,sizeof(LowID_CanMsgSend.data));
        memset(HighID_CanMsgSend.data,0,sizeof(HighID_CanMsgSend.data));

        /* 遍历所有已经创建的DJI3508实例,每层循环执行PID运算任务并把其值加入到发送CAN消息的结构体中 */
        for(uint32_t i = DJI3508_Begin;i<DJI3508_End;i++)
        {
            /* 检查当前实例是否为nullptr */            
            Instance = Instance_Registry[i-DJI3508_Begin];
            /* 如果实例不为空*/
            if(Instance != nullptr)
            {   
                /* 原子操作: 检查电机状态, 处理离线, 并获取数据快照 */
                taskENTER_CRITICAL();
                /* 检查电机状态 */
                if (Instance->Status == DJI3508_Online)
                {         
                    /* 电机在线统计减一,消费者*/
                    Instance->MotorData.Online_CountFlag--;
                    if (Instance->MotorData.Online_CountFlag <= 0)
                    {
                        /* 电机刚刚掉线 */
                        Instance->Status = DJI3508_Offline;
                        AddControlOutToCanMsg(Instance, (int16_t)0x0000);
                        taskEXIT_CRITICAL();
                        continue;
                    }
                    if(Instance->FeedBackMsg.Temperature > DJI3508_CharacterParam::MaxTemperature)
                    {
                        /* 电机温度过高, 跳过 */
                        Instance->Status = DJI3508_OverTemperature;
                        AddControlOutToCanMsg(Instance, (int16_t)0x0000);
                        taskEXIT_CRITICAL();
                        continue;
                    }
                    /* 电机在线, 获取数据快照用于计算 */
                    motorDataSnapshot = const_cast<DJI3508_Data&>(Instance->MotorData);
                }
                /* 如果电机离线或过温, 跳过处理*/
                else
                {
                    /* 电机离线/过温, 跳过 */
                    taskEXIT_CRITICAL();
                    continue;
                }
                taskEXIT_CRITICAL();
                /* 获取电机ID */
                MotorID = Instance->MotorID;
                /* 检查当前实例是否需要在CAN1总线上发送CAN消息 */
                if(Instance->CanBus == USE_CAN1)
                {
                    if(MotorID <= DJI3508_IDLowMax)
                        Can1NeedLowID = true;
                    else
                        Can1NeedHighID = true;
                }
                /* 检查当前实例是否需要在CAN2总线上发送CAN消息 */
                else if(Instance->CanBus == USE_CAN2)
                {
                    if(MotorID >= DJI3508_IDHighMin)
                        Can2NeedHighID = true;
                    else
                        Can2NeedLowID = true;
                }

                /* 依据当前电机快照执行数据运算任务，计算出最终的输出 */
                switch (Instance->ControlMode)
                {
                /* 位置模式 */
                case DJI3508_LocationLoopMode:
                    /* 获取期望角度 */
                    expect = motorDataSnapshot.Exp_Rad;
                    /* 获取当前角度 */
                    feedback = motorDataSnapshot.Now_Rad;
                    /* 计算位置PID输出期望速度 */
                    controlOut = Instance->Location_PID.Calculate(expect,feedback);
                    /* 强制限制期望速度在最大速度范围内 */
                    Constrain<float32_t>(controlOut,-DJI3508_CharacterParam::IDLE_GearBoxRad, DJI3508_CharacterParam::IDLE_GearBoxRad);  
                    /* 写入期望速度到快照 */
                    motorDataSnapshot.Exp_Speed = controlOut;
                /* 速度模式 */
                case DJI3508_SpeedLoopMode:
                    /* 获取期望速度 */
                    expect = motorDataSnapshot.Exp_Speed;
                    /* 获取当前速度 */
                    feedback = motorDataSnapshot.Now_Speed;
                    /* 计算速度PID输出期望电流 */
                    controlOut = Instance->Speed_PID.Calculate(expect,feedback);
                    /* 写入期望电流到快照 */
                    motorDataSnapshot.Exp_Current = controlOut;
                /* 开环模式 */
                case DJI3508_OpenLoopMode:
                    /* 开环模式直接将期望电流作为输出 */
                    controlOut = motorDataSnapshot.Exp_Current;
                    /* 强制限制输出电流在额定电流范围 */
                    Constrain<float32_t>(controlOut,-DJI3508_CharacterParam::Rated_Current_mA, DJI3508_CharacterParam::Rated_Current_mA);  
                    break;                
                default:
                    break;
                }
                /* 原子操作,更新电机状态 */
                taskENTER_CRITICAL();
                switch (Instance->ControlMode)
                {
                /* 位置模式 */
                case DJI3508_LocationLoopMode:
                    /* 开环模式直接将期望电流作为输出 */
                    Instance->MotorData.Exp_Speed =motorDataSnapshot.Exp_Speed;
                /* 速度模式 */
                case DJI3508_SpeedLoopMode:
                    Instance->MotorData.Exp_Current =motorDataSnapshot.Exp_Current;
                /* 开环模式下，Exp_Current 是输入，无需写回 */
                case DJI3508_OpenLoopMode:
                    break;
                default:
                    break;
                }    
                taskEXIT_CRITICAL();

                /* 将期望输出转换为输入到电机的电流 */
                Input_cur = FloatToInt16(controlOut,-DJI3508_CharacterParam::Max_C620Current,DJI3508_CharacterParam::Max_C620Current,-16384,16384);
                
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