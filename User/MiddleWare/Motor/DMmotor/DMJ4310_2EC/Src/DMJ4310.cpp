/*===========================================================
* @file      DMJ4310.cpp
* @author    MRZHENG
* ===========================================================
* @brief
* 该文件依赖
* DMJ4310.hpp
* ===========================================================
* 该文件功能表述(先声明后定义):
* ===========================================================
* @version   0.8
* @date      2025-12-14
* @copyright Copyright (c) 2025
============================================================*/

/*========================= 文件依赖 ========================*/
#include "DMJ4310.hpp"

/*================= 为DMJ4310的静态成员分配内存 ===============*/

/** @brief 控制任务函数句柄,指向DMJ4310的控制任务函数ControlTask */
TaskHandle_t DMJ4310::ControlTaskHandle = nullptr;

/** @brief 静态实例注册表，存储所有DMJ4310实例的指针 */
DMJ4310* DMJ4310::Instance_Registry[DMJ4310_End-DMJ4310_Begin] = {nullptr};

/*=================== DMJ4310的成员函数定义 ==================*/

/*======================= 公有成员函数 =======================*/

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
DMJ4310::DMJ4310(DMJ4310_ID motorid,
                 DMJ4310_MST_ID feedbackid,
                 USE_CanBus canBus,
                 Can::CanBaudRate baudrate,
                 Can::CanMode mode,
                 Map_t pmax,
                 Map_t vmax,
                 Map_t tmax,
                 DMJ4310_ControlMode controlMode)   
{
    this->Motor_ID = motorid;
    this->Feedback_ID = feedbackid;
    this->CanBus = canBus;
    this->BaudRate = baudrate;
    this->Mode = mode;
    this->PMAX = pmax;
    this->VMAX = vmax;
    this->TMAX = tmax;
    this->ControlMode = controlMode;
    /* 硬件状态为未初始化 */
    this->Resource_Inited = false;
    /* 电机状态为失能 */
    this->Status = DMJ4310_Status::DMJ4310_DISABLE;
    /* 设置DMJ4310ID偏移量 */
    switch (ControlMode)
    {
    case DMJ4310_MIT_Location_Mode:
    case DMJ4310_MIT_Speed_Mode:
    case DMJ4310_MIT_Torque_Mode:
        this->Motor_IDoffset = DMJ4310_IDoffset::MIT_Offset;
        break;
    case DMJ4310_Cascade_Loc_Mode:
        this->Motor_IDoffset = DMJ4310_IDoffset::Cascade_LocPI_Offset;
        break;
    case DMJ4310_Speed_Mode:
        this->Motor_IDoffset = DMJ4310_IDoffset::Speed_PI_Offset;
        break;
    case DMJ4310_Force_Loc_Mode:
        this->Motor_IDoffset = DMJ4310_IDoffset::Cascade_LocPI_Limit_Offset;
        break;
    }
    /* 清空电机反馈 */
    memset(const_cast<void *>(static_cast<volatile void*>(&this->FeedBackMsg)), 0, sizeof(this->FeedBackMsg));
    /* 清空电机数据 */
    memset(const_cast<void *>(static_cast<volatile void*>(&this->MotorData)), 0, sizeof(this->MotorData));
    /* 绑定CanManager指针 */
    this->CanManagerPtr = &CanManager::GetInstance();
    /* 注册电机实例到注册表 */
    Instance_Registry[Motor_ID - DMJ4310_Begin] = this;
}


/**
 * @brief 析构函数
 */
DMJ4310::~DMJ4310(){
    /* 失能电机 */
    
    /* 从实例注册表中移除 */
    Instance_Registry[Motor_ID - DMJ4310_Begin] = nullptr;
    /* 取消绑定CanManager单例 */
    CanManagerPtr = nullptr;
}


/**
 * @brief  初始化函数
 * @return MW_Status 初始化状态
 * @note   该函数启动CAN总线资源并设置CAN接受回调函数,并使能电机
 */
MW_Status DMJ4310::Init(){
    /* 检查电机ID是否正确 */
    if(Motor_ID<DMJ4310_Begin || Motor_ID>=DMJ4310_End){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查CAN总线是否正确 */
    if(CanBus<USE_CAN_BEGIN || CanBus>=USE_CAN_END){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
        return MW_Status::INVALID_PARAM;
    }
    MW_Status res = MW_Status::ERROR;
    /* 启动CAN总线资源 */
    res = InitResource();
    if(res == MW_Status::SUCCESS){
        /* 设置CAN消息回调函数 */
        res = SetCallBack();
    }
    /* 发送使能信号 */
    res = Send_Command(DMJ4310_Command::Enable_Command);
    return res;
}

/**
 * @brief 启动DMJ4310电机控制任务
 * @return MW_Status 启动结果
 * @note   该函数会创建一个任务,用于控制电机(多个电机也依旧是这个任务去管理)
 */
MW_Status DMJ4310::Start_ControlTask(){
    /* 检查电机ID是否正确 */
    if(Motor_ID<DMJ4310_Begin||Motor_ID>=DMJ4310_End){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"电机ID错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查CAN总线是否存在 */
    if(CanBus<USE_CAN_BEGIN||CanBus>=USE_CAN_END){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"电机绑定的CAN总线不存在");
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
        xTaskCreateRes = xTaskCreate(ControlTask, "DMJ4310 Control Task", 512, NULL, 24, &ControlTaskHandle);
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
 * @brief  发送命令函数
 * @param command 命令结构体引用
 * @return MW_Status 发送状态
 * @note   用于发送DMJ4310命令,将命令填充到ConmandMsg中
 */
MW_Status DMJ4310::Send_Command(const DMJ4310_Command& command){
    /* 检查CAN总线资源是否已初始化 */
    if(Resource_Inited == false){
        DMJ4310_ASSERT(MW_Status::ERROR,"CAN总线资源未初始化");
        return MW_Status::ERROR;
    }
    /* 创建临时变量 */
    CanMessage Msg;
    /* 根据命令类型构建不同的命令 */
    switch (command)
    {
    case DMJ4310_Command::Enable_Command:
        /* 构建使能命令 */
        Build_EnableCommand(Msg);
        break;
    case DMJ4310_Command::Disable_Command:
        /* 构建失能命令 */
        Build_DisableCommand(Msg);
        break;
    case DMJ4310_Command::SaveZero_Command:
        /* 构建保存零点命令 */
        Build_SaveZeroPosCommand(Msg);
        break;
    case DMJ4310_Command::ClearErr_Command:
        /* 构建清除错误命令 */
        Build_ClearErrCommand(Msg);
        break;
    default:
        break;
    }
    /* 发送命令 */
    return CanManagerPtr->sendMessage(CanBus,const_cast<const CanMessage&>(Msg));
}

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
MW_Status DMJ4310::Set_MIT_Location_Mode_Expect(float32_t Exp_Rad,float32_t Exp_Speed,float32_t Exp_Torque,float32_t Kp,float32_t Kd)
{
    /* 检查是否是MIT_Location_Mode模式 */
    if(ControlMode != DMJ4310_MIT_Location_Mode){
        DMJ4310_ASSERT(MW_Status::INVALID_OPERATION,"不是MIT_Location_Mode模式,操作失败");
        return MW_Status::INVALID_OPERATION;
    }
    /* 检查Exp_Rad参数是否合法 */
    if(Exp_Rad<-PMAX || Exp_Rad>PMAX){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Rad参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Exp_Speed参数是否合法 */
    if(Exp_Speed<-VMAX || Exp_Speed>VMAX){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Speed参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Exp_Torque参数是否合法 */
    if(Exp_Torque<-TMAX || Exp_Torque>TMAX){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Torque参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Kp参数是否合法 */
    if(Kp<=0 || Kp>500){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Kp参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Kd参数是否合法 */
    if(Kd<=0 || Kd>5){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Kd参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 更新DMJ4310_Data中的用户设置期望值 */
    MotorData.Exp_Rad = Exp_Rad;
    MotorData.Exp_Angle = Rad2Deg(Exp_Rad);
    MotorData.Exp_Speed = Exp_Speed;
    MotorData.Exp_Torque = Exp_Torque;
    MotorData.Kp = Kp;
    MotorData.Kd = Kd;

    /* 转换成发包的数据 */
    /* 位置期望映射值(16位数据) = (Exp_Rad + PMAX) / (2.0f * PMAX) * 65535.0f */
    MotorData.MIT_p_des = (uint16_t)FloatToUint(MotorData.Exp_Rad,-PMAX,PMAX,16);
    /* 速度期望映射值(12位数据) = (Exp_Speed + VMAX) / (2.0f * VMAX) * 4095.0f */
    MotorData.MIT_v_des = (uint16_t)FloatToUint(MotorData.Exp_Speed,-VMAX,VMAX,12);
    /* 扭矩期望映射值(12位数据) = (Exp_Torque + TMAX) / (2.0f * TMAX) * 4095.0f */
    MotorData.MIT_t_ff = (uint16_t)FloatToUint(MotorData.Exp_Torque,-TMAX,TMAX,12);
    /* Kp参数(12位数据) = Kp / 500.0f*4095.0f */
    MotorData.MIT_Kp_Map = (uint16_t)FloatToUint(MotorData.Kp,DMJ4310_CharacterParam::KpMin,DMJ4310_CharacterParam::KPMax,12);
    /* Kd参数(12位数据) = Kd / 5.0f*4095.0f */
    MotorData.MIT_Kd_Map = (uint16_t)FloatToUint(MotorData.Kd,DMJ4310_CharacterParam::KdMin,DMJ4310_CharacterParam::KdMax,12);
    return MW_Status::SUCCESS;
};

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
MW_Status DMJ4310::Set_MIT_Speed_Mode_Expect(float32_t Exp_Speed,float32_t Exp_Torque, float32_t Kd)
{
    /* 检查是否是MIT_Speed_Mode模式 */
    if(ControlMode != DMJ4310_MIT_Speed_Mode){
        DMJ4310_ASSERT(MW_Status::INVALID_OPERATION,"不是MIT_Speed_Mode模式,操作失败");
        return MW_Status::INVALID_OPERATION;
    }
    /* 检查Exp_Speed参数是否合法 */
    if(Exp_Speed<-VMAX || Exp_Speed>VMAX){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Speed参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Exp_Torque参数是否合法 */
    if(Exp_Torque<-TMAX || Exp_Torque>TMAX){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Torque参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Kd参数是否合法 */
    if(Kd<=0 || Kd>5){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Kd参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 更新DMJ4310_Data中的用户设置期望值 */
    /* 因为是MIT速度模式,位置期望随便设置都不影响因为kp为0 */
    MotorData.Exp_Rad = 0.0f;
    MotorData.Exp_Angle = 0.0f;
    MotorData.Exp_Speed = Exp_Speed;
    MotorData.Exp_Torque = Exp_Torque;
    /* 因为是MIT速度模式,所以设置Kp参数为0 */
    MotorData.Kp = 0.0f;
    MotorData.Kd = Kd;

    /* 转换成发包的数据 */
    /* 位置期望映射值(16位数据) = (Exp_Rad + PMAX) / (2.0f * PMAX) * 65535.0f */
    MotorData.MIT_p_des = (uint16_t)FloatToUint(MotorData.Exp_Rad,-PMAX,PMAX,16);
    /* 速度期望映射值(12位数据) = (Exp_Speed + VMAX) / (2.0f * VMAX) * 4095.0f */
    MotorData.MIT_v_des = (uint16_t)FloatToUint(MotorData.Exp_Speed,-VMAX,VMAX,12);
    /* 扭矩期望映射值(12位数据) = (Exp_Torque + TMAX) / (2.0f * TMAX) * 4095.0f */
    MotorData.MIT_t_ff = (uint16_t)FloatToUint(MotorData.Exp_Torque,-TMAX,TMAX,12);
    /* Kp参数(12位数据) = Kp / 500.0f*4095.0f */
    MotorData.MIT_Kp_Map = (uint16_t)FloatToUint(MotorData.Kp,DMJ4310_CharacterParam::KpMin,DMJ4310_CharacterParam::KPMax,12);
    /* Kd参数(12位数据) = Kd / 5.0f*4095.0f */
    MotorData.MIT_Kd_Map = (uint16_t)FloatToUint(MotorData.Kd,DMJ4310_CharacterParam::KdMin,DMJ4310_CharacterParam::KdMax,12);
    return MW_Status::SUCCESS;
}

/**
 * @brief  设置MIT_Torque_Mode模式下的期望值
 * @param Exp_Torque 扭矩期望 单位:N.m [-TMAX,TMAX]
 * @return MW_Status 设置状态
 * @note   用于设置MIT_Torque_Mode模式下的扭矩期望
 * @warning 该模式下,Exp_Torque的范围为[-TMAX,TMAX]
 */
MW_Status DMJ4310::Set_MIT_Torque_Mode_Expect(float32_t Exp_Torque)
{
    /* 检查是否是MIT_Torque_Mode模式 */
    if(ControlMode != DMJ4310_MIT_Torque_Mode){
        DMJ4310_ASSERT(MW_Status::INVALID_OPERATION,"不是MIT_Torque_Mode模式,操作失败");
        return MW_Status::INVALID_OPERATION;
    }
    /* 检查Exp_Torque参数是否合法 */
    if(Exp_Torque<-TMAX || Exp_Torque>TMAX){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Torque参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 更新DMJ4310_Data中的用户设置期望值 */
    MotorData.Exp_Rad = 0.0f;
    MotorData.Exp_Angle = 0.0f;
    MotorData.Exp_Speed = 0.0f;
    MotorData.Exp_Torque = Exp_Torque;
    MotorData.Kp = 0.0f;
    MotorData.Kd = 0.0f;

    /* 转换成发包的数据 */
    /* 位置期望映射值(16位数据) = (Exp_Rad + PMAX) / (2.0f * PMAX) * 65535.0f */
    MotorData.MIT_p_des = (uint16_t)FloatToUint(MotorData.Exp_Rad,-PMAX,PMAX,16);
    /* 速度期望映射值(12位数据) = (Exp_Speed + VMAX) / (2.0f * VMAX) * 4095.0f */
    MotorData.MIT_v_des = (uint16_t)FloatToUint(MotorData.Exp_Speed,-VMAX,VMAX,12);
    /* 扭矩期望映射值(12位数据) = (Exp_Torque + TMAX) / (2.0f * TMAX) * 4095.0f */
    MotorData.MIT_t_ff = (uint16_t)FloatToUint(MotorData.Exp_Torque,-TMAX,TMAX,12);
    /* Kp参数(12位数据) = Kp / 500.0f*4095.0f */
    MotorData.MIT_Kp_Map = (uint16_t)FloatToUint(MotorData.Kp,DMJ4310_CharacterParam::KpMin,DMJ4310_CharacterParam::KPMax,12);
    /* Kd参数(12位数据) = Kd / 5.0f*4095.0f */
    MotorData.MIT_Kd_Map = (uint16_t)FloatToUint(MotorData.Kd,DMJ4310_CharacterParam::KdMin,DMJ4310_CharacterParam::KdMax,12);
    return MW_Status::SUCCESS;
}

/**
 * @brief  设置级联位置速度模式下的期望值
 * @param Exp_Rad    位置期望 单位:rad
 * @param SpeedMax   最高速度 单位:rad/s(为正数)
 * @return MW_Status 设置状态
 * @note   用于设置级联位置模式下的位置期望,速度期望
 * @warning SpeedMax范围为(0,IDLE_GearBoxRad]
 */
MW_Status DMJ4310::Set_Cascade_Loc_Mod_Expect(float32_t Exp_Rad,float32_t SpeedMax)
{
    /* 检查是否是Cascade_Loc_Mode模式 */
    if(ControlMode != DMJ4310_Cascade_Loc_Mode){
        DMJ4310_ASSERT(MW_Status::INVALID_OPERATION,"不是Cascade_Loc_Mode模式,操作失败");
        return MW_Status::INVALID_OPERATION;
    }
    /* 检查SpeedMax参数是否合法 */
    if(SpeedMax<=0.0f || SpeedMax>DMJ4310_CharacterParam::IDLE_GearBoxRad){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"SpeedMax参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 更新DMJ4310_Data中的用户设置期望值 */
    MotorData.Exp_Rad = Exp_Rad;
    MotorData.Exp_Angle = Rad2Deg(Exp_Rad);
    MotorData.Speed_Limit = SpeedMax;
   
    return MW_Status::SUCCESS;
}

/**
 * @brief  设置速度PI模式下的期望值
 * @param Exp_Speed  速度期望 单位:rad/s
 * @return MW_Status 设置状态
 * @note   用于设置速度PI模式下的速度期望
 * @warning 该模式下,Exp_Speed不可超过空载转速[-IDLE_GearBoxRad,IDLE_GearBoxRad]
 */
MW_Status DMJ4310::Set_Speed_Mode_Expect(float32_t Exp_Speed){
    /* 检查是否是Speed_Mode模式 */
    if(ControlMode != DMJ4310_Speed_Mode){
        DMJ4310_ASSERT(MW_Status::INVALID_OPERATION,"不是Speed_Mode模式,操作失败");
        return MW_Status::INVALID_OPERATION;
    }
    /* 检查Exp_Speed参数是否合法 */
    if(Exp_Speed<-DMJ4310_CharacterParam::IDLE_GearBoxRad || Exp_Speed>DMJ4310_CharacterParam::IDLE_GearBoxRad){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Exp_Speed参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 更新DMJ4310_Data中的用户设置期望值 */
    MotorData.Exp_Speed = Exp_Speed;
    return MW_Status::SUCCESS;
}

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
MW_Status DMJ4310::Set_Force_Loc_Mode_Expect(float32_t Exp_Rad,float32_t Speed_Limit,float32_t Torque_MAX_Rate)
{
    /* 检查是否是Force_Loc_Mode模式 */
    if(ControlMode != DMJ4310_Force_Loc_Mode){
        DMJ4310_ASSERT(MW_Status::INVALID_OPERATION,"不是Force_Loc_Mode模式,操作失败");
        return MW_Status::INVALID_OPERATION;
    }
    /* 检查Speed_Limit参数是否合法 */
    if(Speed_Limit<=0.0f || Speed_Limit>100.0f){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Speed_Limit参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 检查Torque_MAX_Rate参数是否合法 */
    if(Torque_MAX_Rate<0.0f || Torque_MAX_Rate>1.0f){
        DMJ4310_ASSERT(MW_Status::INVALID_PARAM,"Torque_MAX_Rate参数错误");
        return MW_Status::INVALID_PARAM;
    }
    /* 更新DMJ4310_Data中的用户设置期望值 */
    MotorData.Exp_Rad = Exp_Rad;
    MotorData.Exp_Angle = Rad2Deg(Exp_Rad);
    MotorData.Speed_Limit = Speed_Limit;
    MotorData.Torque_Max_Rate = Torque_MAX_Rate;

    /* 转换成发包的数据 */   
    /* V_des：限速值，单位rad/s，放大100倍，类型为无符号16位，低位在前，高位在后，
    范围为0-10000，超过10000会限制在10000，故对应的实际速度限定幅值为0~100rad/s； */
    MotorData.FL_V_des = (uint16_t)(MotorData.Speed_Limit * 100.0f);
    /* I_des：扭矩电流限定标幺值，放大10000倍，类型为无符号16位，低位在前，高位在后，
     范围为0-10000，超过10000会限制在10000，对应的实际电流限定标幺幅值为0-1.0 */
    MotorData.FL_I_des = (uint16_t)(MotorData.Torque_Max_Rate * 10000.0f);
    return MW_Status::SUCCESS;
}

/**
 * @brief 获取DMJ4310反馈消息
 * @return DMJ4310_FeedBackMsg DMJ4310反馈消息结构体
 * @note  该函数用于获取电机反馈信息
 */
DMJ4310_FeedBackMsg DMJ4310::getFeedBackMsg() const{
    DMJ4310_FeedBackMsg temp;
    /* 记录当前中断状态 */
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    /* 安全地拷贝 volatile 数据到临时变量 */
    temp = const_cast<const DMJ4310_FeedBackMsg&>(this->FeedBackMsg);
    /* 退出临界区（恢复中断）*/
    if (!primask_bit) {
        __enable_irq();
    }
    /* 返回副本，外部怎么用都安全 */
    return temp; 
}

/**
 * @brief 获取DMJ4310电机的数据参数
 * @return DMJ4310_Data 电机数据参数结构体
 * @note  该函数用于获取电机的实时数据参数
 */
DMJ4310_Data DMJ4310::getMotorData() const{
    DMJ4310_Data temp;
    /* 记录当前中断状态 */
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    /* 安全地拷贝 volatile 数据到临时变量(一次拷贝) */
    temp = const_cast<const DMJ4310_Data&>(this->MotorData);
    /* 退出临界区（恢复中断）*/
    if (!primask_bit) {
        __enable_irq();
    }
    /* 返回副本，外部怎么用都安全 */
    return temp; 
}
/*======================= 私有成员函数 =======================*/

/**
 * @brief  DMJ4310电机CAN初始化函数,申请CAN总线资源
 * @return MW_Status 初始化状态
 * @note   被Init调用,申请CAN总线资源，Init已经保证电机ID和CAN总线的有效性
 */
MW_Status DMJ4310::InitResource()
{
    /* 申请CAN总线资源 */
    MW_Status res = CanManagerPtr->AskResource(CanBus,BaudRate,Mode);

    /* 检查CAN总线资源申请结果 */
    DMJ4310_ASSERT(res,"申请CAN总线资源失败,检查CAN总线配置和之前配置是否冲突");
    /* 如果申请成功,则启动CAN总线资源 */
    if(res == MW_Status::SUCCESS)
    {
        res = CanManagerPtr->StartResource(CanBus);
        /* 检查CAN总线资源启动结果 */
        DMJ4310_ASSERT(res,"启动CAN总线资源失败");
        /* 如果CAN总线资源启动成功 */
        if(res == MW_Status::SUCCESS){
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
MW_Status DMJ4310::SetCallBack(){
    MW_Status res = MW_Status::ERROR;
    /* 检查CAN总线资源是否已初始化 */
    if(Resource_Inited == true){
        /* 订阅CAN总线,向CanManager 注册CAN消息的回调函数,用于处理DMJ4310电机返回的CAN消息 */
        res = CanManagerPtr->Subscribe(CanBus,Feedback_ID,DMJ4310_CanMsgCallBack);
        /* 检查CAN消息回调函数订阅结果 */
        DMJ4310_ASSERT(res,"订阅CAN消息回调函数失败");
    }
    return res;
}

/**
 * @brief  构建使能命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建使能命令,将使能命令填充到ConmandMsg中
 */
void DMJ4310::Build_EnableCommand(CanMessage& ConmandMsg){
    ConmandMsg.id = (uint32_t)Motor_ID + (uint32_t)Motor_IDoffset;
    ConmandMsg.isExtended = false;
    ConmandMsg.isRemote = false;
    ConmandMsg.len = 8;
    ConmandMsg.data[0] = 0xFF;
    ConmandMsg.data[1] = 0xFF;
    ConmandMsg.data[2] = 0xFF;
    ConmandMsg.data[3] = 0xFF;
    ConmandMsg.data[4] = 0xFF;
    ConmandMsg.data[5] = 0xFF;
    ConmandMsg.data[6] = 0xFF;
    ConmandMsg.data[7] = 0xFC;
}

/**
 * @brief  构建失能命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建禁用命令,将禁用命令填充到ConmandMsg中
 */
void DMJ4310::Build_DisableCommand(CanMessage& ConmandMsg){
    ConmandMsg.id = (uint32_t)Motor_ID + (uint32_t)Motor_IDoffset;
    ConmandMsg.isExtended = false;
    ConmandMsg.isRemote = false;
    ConmandMsg.len = 8;
    ConmandMsg.data[0] = 0xFF;
    ConmandMsg.data[1] = 0xFF;
    ConmandMsg.data[2] = 0xFF;
    ConmandMsg.data[3] = 0xFF;
    ConmandMsg.data[4] = 0xFF;
    ConmandMsg.data[5] = 0xFF;
    ConmandMsg.data[6] = 0xFF;
    ConmandMsg.data[7] = 0xFD;
}

/**
 * @brief  构建保存零点命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建保存零点命令,将保存零点命令填充到ConmandMsg中
 */
void DMJ4310::Build_SaveZeroPosCommand(CanMessage& ConmandMsg){
    ConmandMsg.id = (uint32_t)Motor_ID + (uint32_t)Motor_IDoffset;
    ConmandMsg.isExtended = false;
    ConmandMsg.isRemote = false;
    ConmandMsg.len = 8;
    ConmandMsg.data[0] = 0xFF;
    ConmandMsg.data[1] = 0xFF;
    ConmandMsg.data[2] = 0xFF;
    ConmandMsg.data[3] = 0xFF;
    ConmandMsg.data[4] = 0xFF;
    ConmandMsg.data[5] = 0xFF;
    ConmandMsg.data[6] = 0xFF;
    ConmandMsg.data[7] = 0xFE;
}

/**
 * @brief  构建清除错误命令
 * @param ConmandMsg 命令消息结构体引用
 * @note   用于构建清除错误命令,将清除错误命令填充到ConmandMsg中
 */
void DMJ4310::Build_ClearErrCommand(CanMessage& ConmandMsg){
    ConmandMsg.id = (uint32_t)Motor_ID + (uint32_t)Motor_IDoffset;
    ConmandMsg.isExtended = false;
    ConmandMsg.isRemote = false;
    ConmandMsg.len = 8;
    ConmandMsg.data[0] = 0xFF;
    ConmandMsg.data[1] = 0xFF;
    ConmandMsg.data[2] = 0xFF;
    ConmandMsg.data[3] = 0xFF;
    ConmandMsg.data[4] = 0xFF;
    ConmandMsg.data[5] = 0xFF;
    ConmandMsg.data[6] = 0xFF;
    ConmandMsg.data[7] = 0xFB;
}

/**
 * @brief  控制函数,根据控制模式,发送对应CAN命令
 * @note   被ControlTask调用,根据控制模式,发送对应CAN命令
 * @note   使用的消息包不再是临时变量
 * @return MW_Status 发送状态
 */
MW_Status DMJ4310::Send_ControlCommand(){
    /* 检查CAN总线资源是否已初始化 */
    if(Resource_Inited == false){
        return MW_Status::ERROR;
    }
    
    /* 创建局部变量用于发送消息 (栈分配，速度极快，线程安全) */
    CanMessage Msg;
    Msg.id = (uint32_t)Motor_ID + (uint32_t)Motor_IDoffset;
    Msg.isExtended = false;
    Msg.isRemote = false;
    Msg.len = 8;
    /* 依据不同模式发送数据包 */
    switch (ControlMode)
    {
    /* MIT模式*/
    case DMJ4310_MIT_Location_Mode:
    case DMJ4310_MIT_Speed_Mode:
    case DMJ4310_MIT_Torque_Mode:
        {
            /* D[0] = p_des[15:8] */
            Msg.data[0] = (uint8_t)((MotorData.MIT_p_des & 0xFF00) >> 8);
            /* D[1] = p_des[7:0] */
            Msg.data[1] = (uint8_t)((MotorData.MIT_p_des & 0x00FF));
            /* D[2] = v_des[11:4] */
            Msg.data[2] = (uint8_t)((MotorData.MIT_v_des & 0x0FF0) >> 4);
            /* D[3] = v_des[3:0]|Kp[11:8] */
            Msg.data[3] = (uint8_t)((MotorData.MIT_v_des & 0x000F)<<4 |(MotorData.MIT_Kp_Map& 0x0F00)>>8);
            /* D[4] = Kp[7:0] */
            Msg.data[4] = (uint8_t)((MotorData.MIT_Kp_Map & 0x00FF));
            /* D[5] = Kd[11:4] */
            Msg.data[5] = (uint8_t)((MotorData.MIT_Kd_Map & 0x0FF0)>>4);
            /* D[6] = Kd[3:0]|t_ff[11:8] */
            Msg.data[6] = (uint8_t)((MotorData.MIT_Kd_Map & 0x000F)<<4 |(MotorData.MIT_t_ff &0x0F00)>>8);
            /* D[7] = t_ff[7:0] */
            Msg.data[7] = (uint8_t)((MotorData.MIT_t_ff & 0x00FF));
        }
        break;
    /* 串级位置速度PI模式*/
    case DMJ4310_Cascade_Loc_Mode:
        {
            /* 使用指针别名访问float的字节表示 */
            uint8_t* pExpRad = (uint8_t*)&MotorData.Exp_Rad;
            uint8_t* pSpeedLimit = (uint8_t*)&MotorData.Speed_Limit;
            
            /* P_des: 位置给定, 浮点型, 低位在前 */
            Msg.data[0] = pExpRad[0];
            Msg.data[1] = pExpRad[1];
            Msg.data[2] = pExpRad[2];
            Msg.data[3] = pExpRad[3];
            
            /* V_des: 速度给定, 浮点型, 低位在前 */
            Msg.data[4] = pSpeedLimit[0];
            Msg.data[5] = pSpeedLimit[1];
            Msg.data[6] = pSpeedLimit[2];
            Msg.data[7] = pSpeedLimit[3];
        }
        break;
    /* 速度PI模式*/
    case DMJ4310_Speed_Mode:
        {
            /* 数据包大小改成4字节 */
            Msg.len = 4;
            /* 使用指针别名访问float的字节表示 */
            uint8_t* v_des = (uint8_t*)&MotorData.Exp_Speed;
            /* V_des: 速度给定, 浮点型, 低位在前 */
            Msg.data[0] = v_des[0];
            Msg.data[1] = v_des[1];
            Msg.data[2] = v_des[2];
            Msg.data[3] = v_des[3];
        }
        break;
    /* 力位混控模式*/
    case DMJ4310_Force_Loc_Mode:
        {
            uint8_t* p_des = (uint8_t*)&MotorData.Exp_Rad;
            /* P_des: 位置给定, 浮点型, 低位在前 */
            Msg.data[0] = p_des[0];
            Msg.data[1] = p_des[1];
            Msg.data[2] = p_des[2];
            Msg.data[3] = p_des[3];
            /* V_des: 限速值,类型为无符号16位,低位在前 */
            Msg.data[4] = MotorData.FL_V_des & 0xFF;
            Msg.data[5] = (MotorData.FL_V_des >> 8) & 0xFF;
            /* i_des: 扭矩电流限定标幺值,限速值,类型为无符号16位,低位在前 */
            Msg.data[6] = MotorData.FL_I_des & 0xFF;
            Msg.data[7] = (MotorData.FL_I_des >> 8) & 0xFF;
        }
        break;
    default:
        return MW_Status::INVALID_PARAM;
    }
    
    /* 发送消息 */
    return CanManagerPtr->sendMessage(CanBus, Msg);
}

/*======================== 静态函数 ========================*/

/**
 * @brief DMJ4310电机的CAN消息回调函数
 * @param canId 收到的CAN消息ID
 * @param data 收到的CAN消息数据指针
 * @param len 收到的CAN消息数据长度
 * @details 根据反馈的C620消息，刷新电机数据
 * @note  工作流程：校验数据格式 -> 赋值给fanDMJ4310反馈消息结构体 -> 解析数据 -> 刷新电机数据
 */
void DMJ4310::DMJ4310_CanMsgCallBack(uint32_t canId, uint8_t* data, uint8_t len){
    /* 检查CAN消息数据是否有效 */
    if(data == nullptr || len != 8){
        return ;
    }
    /* 检查ID是否在范围内 */
    if(canId < DMJ4310_MST_Begin || canId >= DMJ4310_MST_End){
        return;
    }
    /* 寻找当前电机实例,根据CAN消息ID确定电机实例 */
    DMJ4310* temp = Instance_Registry[canId - DMJ4310_MST_Begin];
    if(temp != nullptr){

        /* 赋值给DMJ4310反馈消息结构体 */
        temp->FeedBackMsg.ERR_ID        = data[0];
        temp->FeedBackMsg.Pos15_8       = data[1];
        temp->FeedBackMsg.Pos7_0        = data[2];
        temp->FeedBackMsg.Speed11_4     = data[3];
        temp->FeedBackMsg.Speed3_0_T11_8= data[4];
        temp->FeedBackMsg.Torque7_0     = data[5];
        temp->FeedBackMsg.T_MOS         = data[6];
        temp->FeedBackMsg.T_Rotor       = data[7];
    
        /* 解析数据 */
        /* 提取反馈帧的ERR */
        uint8_t ERR = temp->FeedBackMsg.ERR_ID & 0xF0;
        /* 提取反馈帧的POS */
        uint16_t POS = ((uint16_t)(temp->FeedBackMsg.Pos15_8) << 8) | (uint16_t)(temp->FeedBackMsg.Pos7_0);
        /* 提取反馈帧的VEL */
        uint16_t VEL = (uint16_t(temp->FeedBackMsg.Speed11_4<<4)|((temp->FeedBackMsg.Speed3_0_T11_8 & 0xF0)>>4));
        /* 提取反馈帧的TORQUE 注意这里必须&0x0F,否则会出错,如果不&0x0F,高位的Speed3_0会被揉到这个数据里 */
        uint16_t T =  (uint16_t((temp->FeedBackMsg.Speed3_0_T11_8&0x0F)<<8)|(temp->FeedBackMsg.Torque7_0));

        /* 刷新电机状态 */
        temp->Status = (DMJ4310_Status)(ERR);
        /* 刷新电机上一次POS值 */
        temp->MotorData.Pre_Encoder = temp->MotorData.Now_Encoder;
        /* 刷新电机当前POS值 */
        temp->MotorData.Now_Encoder = POS;
        /* 首次接收数据处理：同步Pre_Encoder，防止Delta计算错误(因为是绝对值编码器所以需要加这个判断) */
        if(temp->MotorData.FirstGetMSG_Flag == 0){
            temp->MotorData.Pre_Encoder = temp->MotorData.Now_Encoder;
            temp->MotorData.FirstGetMSG_Flag = 1;
        }
        /* 计算POS值增量 */
        int32_t Delta_Encoder = temp->MotorData.Now_Encoder - temp->MotorData.Pre_Encoder;
        /* 计算电机POS的圈数 */
        /* POS增量 < -Encoder_Per_Round/2, 则POS正方向转了一圈 */
        if(Delta_Encoder < -DMJ4310_CharacterParam::Encoder_Per_Round/2){
            /* POS正方向转了一圈 */
            temp->MotorData.Total_EncoderRound++;
        }/* POS增量 > Encoder_Per_Round/2, 则POS负方向转了一圈 */
        else if(Delta_Encoder > DMJ4310_CharacterParam::Encoder_Per_Round/2){
            /* POS负方向转了一圈 */
            temp->MotorData.Total_EncoderRound--;
        }
        /* 总的编码器计数 = 总圈数 * 每圈编码器数 + 当前编码器值 */
        temp->MotorData.Total_Encoder = temp->MotorData.Total_EncoderRound * DMJ4310_CharacterParam::Encoder_Per_Round + temp->MotorData.Now_Encoder;
        /* 电机输出轴当前弧度  = 总的编码器圈数*2*PMAX + 当前编码器值/(每圈编码器最大值-1)*2*PMAX-PMAX */
        temp->MotorData.Now_Rad = temp->MotorData.Total_EncoderRound * 2 * temp->PMAX + temp->MotorData.Now_Encoder / (float32_t)(DMJ4310_CharacterParam::Encoder_Per_Round-1) * 2 * temp->PMAX - temp->PMAX;
        /* 电机总的输出轴圈数 = 电机输出轴当前弧度 / 2*PI */
        temp->MotorData.Total_Round = temp->MotorData.Now_Rad / (2 * PI);
        /* 电机输出轴当前角度 = 电机输出轴当前弧度 / PI * 180 */
        temp->MotorData.Now_Angle = temp->MotorData.Now_Rad  / PI * 180;
        /* 电机输出轴速度 = VEL * 2 * VMAX/(每圈速度最大值-1) - VMAX*/
        temp->MotorData.Now_Speed =  VEL * 2 * temp->VMAX / (float32_t)(DMJ4310_CharacterParam::Speed_Per_Round-1) - temp->VMAX;
        /* 电机输出轴扭矩 = T * 2 * TMAX/(每圈扭矩最大值-1) - TMAX*/
        temp->MotorData.Now_Torque = T* 2 * temp->TMAX / (float32_t)(DMJ4310_CharacterParam::Torque_Per_Round-1) - temp->TMAX;
        /* 电机MOS平均温度 */
        temp->MotorData.Mos_Temp = temp->FeedBackMsg.T_MOS;
        /* 电机转子平均温度 */
        temp->MotorData.Rotor_Temp = temp->FeedBackMsg.T_Rotor;
    }
}


/**
 * @brief 控制任务函数,用于根据控制模式,控制所有注册的DMJ4310电机
 */
void DMJ4310::ControlTask(void* pvParameters){
    /* 任务运行频率为1ms */
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    /* 任务第一次运行时,获取当前时间 */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    /* 定义DMJ4310电机实例指针(临时变量)*/
    DMJ4310* Instance = nullptr;
    while(1)
    {
        /* 遍历所有的DMJ4310电机实例 */
        for(uint8_t index = DMJ4310_Begin;index<DMJ4310_End;index++){
            Instance = Instance_Registry[index - DMJ4310_Begin];
            if(Instance != nullptr){
                /* 调用电机实例的控制函数 */
                Instance->Send_ControlCommand();
            }
        }
        /* 绝对延时,保证任务运行频率为1ms */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}