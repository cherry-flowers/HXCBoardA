
// #include <limits>

// #include "UserMain.h" // 包含用户主函数头文件
// #include "ws2812.h" // 包含WS2812驱动头文件
// #include "BspDevice.h"
// #include "BspTimer.h"
// #include "BspUart.h"
// #include "BspPwm.h"
// #include "BspCan.h"
// #include "DJI3508.hpp"

// extern "C" 
// {
//   #include "VOFA.h" 
//   #include "task.h"
// }

// /**
//  * @brief  处理断言失败的回调函数
//  * @param status 断言失败的状态码
//  * @param msg 断言失败的消息
//  * @param file 断言失败的文件名
//  * @param line 断言失败的行号
//  * @details 当断言失败时,调用该函数打印错误信息并进入死循环
//  */
// void MW_AssertStatusFailedHandle(MW_Status status, const char* msg, const char* file, int line){
//     const char* statusMsg = MW_StatusToString(status);
//     Printf("断言失败: %s, 文件: %s, 行号: %d, 状态: %s\n", msg, file, line, statusMsg);
//     while(1);
// }


// Uart uart6(DEVICE_USART_VOFA); // 使用USART6作为示例
// Uart uart4(DEVICE_USART_4); // 使用USART4作为示例
// Timer tim7(DEVICE_TIMER_7); // 使用tim7去负责利用串口发送matlab数据

// PID_Param Location_PID_Param = {0.2f,0.0f,0.0f,0.0f,0.001f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
// PID_Param Sepeed_PID_Param = {300.2,100.3f,0.0f,0.0f,0.001f,0.1f,DJI3508_CharacterParam::Rated_Current_mA,1000.0f,2000.0f,400.0f,1000.0f};

// DJI3508 motor1(DJI3508_1,USE_CAN1, Can::CanBaudRate::BAUD_1M,
//               Location_PID_Param,
//               Sepeed_PID_Param,
//               DJI3508_ControlMode::DJI3508_OpenLoopMode,
//               Can::CanMode::MODE_NORMAL
//               );

// DJI3508 motor2(DJI3508_2,USE_CAN1, Can::CanBaudRate::BAUD_1M,
//               Location_PID_Param,
//               Sepeed_PID_Param,
//               DJI3508_ControlMode::DJI3508_OpenLoopMode,
//               Can::CanMode::MODE_NORMAL
//               );


// /*======================== 用于采样 =========================*/

// /*全局采样数组*/
// #define SAMPLE_BUFFER_SIZE 202
// #define SAMPLE_COUNT_MAX 10
// #define BYTES_PER_SAMPLE 6

// /*存储采样的数据，最多存储二十个数据包*/
// uint8_t sampleData[SAMPLE_BUFFER_SIZE] = {0xff,0xff};
// /*采样次数*/
// uint32_t sampleCount = 0;
// /*采样的数据存储到的数组索引*/
// uint8_t sampleIndex = 2;
// /*采样数据*/
// C620_FeedBackMsg sampleMsg;
// /*时间*/
// float32_t timeCount = 0.0f;

// /** 采集电流数据的回调 */
// void sampleCallback()
// { 
//   // 边界检查：确保不会数组越界
//   if (sampleIndex + BYTES_PER_SAMPLE > SAMPLE_BUFFER_SIZE) {
//     // 如果空间不足，强制发送当前数据并重置
//     uart4.SendData(sampleData, sampleIndex);
//     sampleIndex = 2;
//     sampleCount = 0;
//     return;
//   }
//   sampleMsg = motor1.getC620FeedBackMsg();
//   sampleCount++;
//   // 写入电流数据（高字节在前，低字节在后）
//   sampleData[sampleIndex++] = (uint8_t)(sampleMsg.Current >> 8);
//   sampleData[sampleIndex++] = (uint8_t)(sampleMsg.Current & 0xFF);
  
//   // 达到预定采样次数后发送数据
//   if (sampleCount >= SAMPLE_COUNT_MAX)
//   {
//     // 发送固定长度的数据包：起始符 + 10组电流数据
//     uart4.SendData(sampleData, sampleIndex);
//     sampleIndex = 2; // 重置索引，保留起始符 0xff 0xff
//     sampleCount = 0;
//   }
// }


// volatile int16_t g_cmd_current_int16 = 0; // 记录当前下发的电流指令（C620标记作用）
// void ChirpSampleCallback()
// {
//     // ... existing code ...
//     // 边界检查：确保不会数组越界
//     if (sampleIndex + BYTES_PER_SAMPLE > SAMPLE_BUFFER_SIZE) {
//         uart4.SendData(sampleData, sampleIndex);
//         sampleIndex = 2;
//         sampleCount = 0;
//         return;
//     }

//     sampleMsg = motor1.getC620FeedBackMsg();
//     sampleCount++;

//     // 写入三路数据：指令电流、反馈转速、反馈电流（高字节在前）
//     sampleData[sampleIndex++] = (uint8_t)(g_cmd_current_int16 >> 8);
//     sampleData[sampleIndex++] = (uint8_t)(g_cmd_current_int16 & 0xFF);

//     sampleData[sampleIndex++] = (uint8_t)(sampleMsg.RPM >> 8);
//     sampleData[sampleIndex++] = (uint8_t)(sampleMsg.RPM & 0xFF);

//     sampleData[sampleIndex++] = (uint8_t)(sampleMsg.Current >> 8);
//     sampleData[sampleIndex++] = (uint8_t)(sampleMsg.Current & 0xFF);

//     // 达到预定采样次数后发送数据
//     if (sampleCount >= SAMPLE_COUNT_MAX)
//     {
//         uart4.SendData(sampleData, sampleIndex);
//         sampleIndex = 2; // 保留起始符 0xff 0xff
//         sampleCount = 0;
//     }
//     // ... existing code ...
// }


// void ChirpTask(void *pvParameters){
//     const TickType_t period = pdMS_TO_TICKS(1);
//     TickType_t last = xTaskGetTickCount();

//     const float f0 = 0.1f;      // 起始频率（Hz）
//     const float f1 = 20.0f;     // 终止频率（Hz）(可以适当提高回20Hz)
//     const float T  = 20.0f;     // 扫频时长（s）
//     const float amp_mA  = 600.0f; // 幅值（mA）
//     const float bias_mA = 800.0f;  // 直流偏置（mA）
//     const float kf = (f1 - f0) / T;

//     const float PI_F = 3.14159265358979323846f;

//     uint32_t k = 0;
//     while (1) {
//         float t = k * 0.001f; // 1 ms 步进
//         float phase = 2.0f * PI_F * (f0 * t + 0.5f * kf * t * t); // 2 pi *(f0 + kf*0.5*t)*t
//         float u_mA = bias_mA + amp_mA * sinf(phase);


//         // 如需只测一台电机，可仅对 motor1 下发指令
//         motor1.SetExpect(u_mA);
//         // 记录“指令电流”的 C620 标度，供采样打包
//         g_cmd_current_int16 = FloatToInt16(u_mA, -20000.f, 20000.f, -16384, 16384);

//         k++;
//         if (t >= T) { k = 0; } // 循环扫频
//         vTaskDelayUntil(&last, period);
//     }
// }

// void sampletest(void *pvParameters) 
// {

//   Delay(3000);

//   VOFA_Init();
   
//   auto uartInitResult = uart6.Init(460800);

//   if (!uartInitResult.ok())
//   {
//     BspLogError(uartInitResult, "UART Init");
//     return;
//   }
//   uart6.EnableRxDMA();
//   uart6.ShowInfo();
  
  
  
//   tim7.Init(2000); 
//   //tim7.SetCallback(sampleCallback); 
//   tim7.SetCallback(ChirpSampleCallback); 
//   //tim7.ShowInfo(); 
//   tim7.Start();


//   uart4.Init(460800);
//   uart4.EnableRxDMA();
//   uart4.ShowInfo();
  

//   CanManager& canManager = CanManager::GetInstance();

//   motor1.Init();
//   motor1.StartControlTask(); 
//   xTaskCreate(ChirpTask, "Chirp Task", 512, NULL, 23, NULL);
//   //while (1)
//   {
//   //  motor1.SendCanMsg(idle_current);
//   //  motor2.SendCanMsg(idle_current);
//   //  Delay(1);
//   }
// }





