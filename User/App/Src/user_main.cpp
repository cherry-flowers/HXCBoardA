
#include <limits>

#include "UserMain.h" // 包含用户主函数头文件
#include "BspDevice.h"
#include "BspTimer.h"
#include "BspUart.h"
#include "BspPwm.h"
#include "BspCan.h"
#include "Log.h"
extern "C" 
{
  #include "task.h"
}

Uart uartDebug(DEVICE_USART_6);

void userMain() 
{
  Log::Init(uartDebug);
  Log::Print("HXCBordA ready\n");
  
  while (1)
  {

    Delay(10);
  }
}

