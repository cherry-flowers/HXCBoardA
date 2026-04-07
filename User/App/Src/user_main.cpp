
#include <limits>

#include "UserMain.h" // 包含用户主函数头文件
#include "BspDevice.h"
#include "BspTimer.h"
#include "BspUart.h"
#include "BspPwm.h"
#include "BspCan.h"
#include "Log.h"
//用户库
#include "DJI3508.hpp"
#include "DJI2006.hpp"
#include "DMJ4310.hpp"
#include "elrs.hpp"


extern "C" 
{
  #include "task.h"
}

enum class Catch_weapon : uint8_t
{
    START,
    TRY_FIND_TARGET,
    FIND_TARGET_NOW,
    TRY_CATCH,
    CATCH_SUCCESS,
    CATCH_FAIL,
    RETURN_BACK,
    END
};

/*============================
3508*2 ---> 同步控制夹爪上下移动
3508   ---> 控制夹爪左右移动
4310   --->控制夹爪旋转
2006   --->控制夹爪前后移动
电磁阀 --->控制夹爪开合
===============================
*/

PID_Param Location_PID_Param = {0.2f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_Param Speed_PID_Param = {500.2, 100.3f, 0.0f, 0.0f, 0.001f, 0.1f, DJI3508_CharacterParam::Rated_Current_mA, 1000.0f, 2000.0f, 400.0f, 1000.0f};


DJI3508 motor1(DJI3508_1, USE_CAN1, Can::CanBaudRate::BAUD_1M,
                Location_PID_Param,
                Speed_PID_Param,
                DJI3508_ControlMode::DJI3508_LocationLoopMode,
                Can::CanMode::MODE_NORMAL);

DJI3508 motor2(DJI3508_2, USE_CAN1, Can::CanBaudRate::BAUD_1M,
                Location_PID_Param,
                Speed_PID_Param,
                DJI3508_ControlMode::DJI3508_LocationLoopMode,
                Can::CanMode::MODE_NORMAL);                
DJI3508 motor3(DJI3508_3, USE_CAN1, Can::CanBaudRate::BAUD_1M,
                Location_PID_Param,
                Speed_PID_Param,
                DJI3508_ControlMode::DJI3508_LocationLoopMode,
                Can::CanMode::MODE_NORMAL);

DJI2006 motor4(DJI2006_1,USE_CAN1,Can::CanBaudRate::BAUD_1M,
                Location_PID_Param,
                Speed_PID_Param,
                DJI2006_ControlMode::DJI2006_LocationLoopMode,
                Can::CanMode::MODE_NORMAL
);                

DMJ4310 motor5(
  DMJ4310_ID::DMJ4310_1,
  DMJ4310_MST_ID::DMJ4310_MST_1,
  USE_CAN1,
  Can::CanBaudRate::BAUD_1M,
  Can::CanMode::MODE_NORMAL,
  18.850f,   //PMAX 3圈
  50.0f,   //VMAX 50rad/s
  5.0f,    //TMAX 5N.m
  DMJ4310_ControlMode::DMJ4310_MIT_Location_Mode
);

class MOTOR_CATCH
{
  public:
    MOTOR_CATCH(DJI3508 &m1, DJI3508 &m2, DJI3508 &m3, DJI2006 &m4, DMJ4310 &m5)
    : motor1(m1), motor2(m2), motor3(m3), motor4(m4), motor5(m5)
    {

    }

    void Init()
    {
      motor1.Init();
      motor2.Init();
      motor3.Init();
      motor4.Init();
      motor5.Init();
    }

    void Raise_SetPosition(float32_t pos1, float32_t pos2)
    {
      motor1.SetExpect(pos1);
      motor2.SetExpect(pos2);
    }

    void Slide_SetPosition(float32_t pos)
    {
      motor3.SetExpect(pos);
    }

    void Push_SetPosition(float32_t pos)
    {
      motor4.SetExpect(pos);
    }
    void Rotate_SetPosition(float32_t pos, float32_t speed, float32_t torque, float32_t Kp, float32_t Kd)
    {
      motor5.Set_MIT_Location_Mode_Expect(pos, speed, torque, Kp, Kd);
    }

    void StartControlTask()
    {
      motor1.StartControlTask();
      motor2.StartControlTask();
      motor3.StartControlTask();
      motor4.StartControlTask();
      motor5.Start_ControlTask();
    }

  private:
    DJI3508 &motor1;
    DJI3508 &motor2;
    DJI3508 &motor3;
    DJI2006 &motor4;
    DMJ4310 &motor5;
};

MOTOR_CATCH motor_catch(motor1, motor2, motor3, motor4, motor5);

Uart uartDebug(DEVICE_USART_6);
Can can1(DEVICE_CAN_1);

void userMain() 
{
  Log::Init(uartDebug);
  Log::Print("HXCBordA ready\n");
  
  can1.Init(Can::CanBaudRate::BAUD_1M, Can::CanMode::MODE_NORMAL);
  can1.Start();

  motor_catch.Init();
  
  HAL_Delay(10); // 等待电机初始化完成

  motor_catch.StartControlTask();


  while (1)
  {

    Delay(10);
  }
}

