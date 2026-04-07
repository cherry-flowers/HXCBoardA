#include "elrs.hpp"
#include "usart.h"

Uart uartElrs(DEVICE_USART_7);


float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
   float output_value;
   if (input_value < input_min)
   {
       output_value = output_min;
   }
   else if (input_value > input_max)
   {
       output_value = output_max;
   }
   else
   {
       output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
   }
   return output_value;
}
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
   float output_median = (output_max - output_min) / 2 + output_min;
   if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
   {
       return output_min;
   }

   if (input_value < median)
   {
       return float_Map(input_value, input_min, median, output_min, output_median);
   }
   else
   {
       return float_Map(input_value, median, input_max, output_median, output_max);
   }
}

uint8_t elrs_rx_buffer[64] = {0}; 
void ELRS_Init(void)
{
    uartElrs.Init(420000);
    uartElrs.EnableRxDMA();
}

ELRS_Data elrs_data;

void ELRS_UARTE_RxCallback(uint16_t dmaCurrentPos)
{

    auto result = uartElrs.ReceiveData(elrs_rx_buffer, dmaCurrentPos);
    uint16_t realSize = result.value;

    if (realSize == 0) return;
    for (int i = 0; i < realSize - 2; i++)
    {
        // 寻找 CRSF 包头 (0xC8)
        if (elrs_rx_buffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            // 获取这一包的长度 (Len 字节包含 Type, Payload, CRC)
            uint8_t payloadLen = elrs_rx_buffer[i+1];
            uint8_t fullPacketLen = payloadLen + 2; // Header(1) + Len(1) + Payload(Len)

            // 检查缓冲区里是否有完整的一包数据
            if (i + fullPacketLen > realSize) 
            {
                continue; // 数据不完整，跳过，等待下一帧
            }

            // === 找到完整包，开始解析 ===
            // 使用 frame 指针指向包的起始位置
            uint8_t* frame = &elrs_rx_buffer[i];
            uint8_t frameType = frame[2];

            if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                // 注意：这里所有的下标都要基于 frame，而不是 elrs_rx_buffer
                elrs_data.channels[0] = ((uint16_t)frame[3] >> 0 | ((uint16_t)frame[4] << 8)) & 0x07FF;
                elrs_data.channels[1] = ((uint16_t)frame[4] >> 3 | ((uint16_t)frame[5] << 5)) & 0x07FF;
                elrs_data.channels[2] = ((uint16_t)frame[5] >> 6 | ((uint16_t)frame[6] << 2) | ((uint16_t)frame[7] << 10)) & 0x07FF;
                elrs_data.channels[3] = ((uint16_t)frame[7] >> 1 | ((uint16_t)frame[8] << 7)) & 0x07FF;
                elrs_data.channels[4] = ((uint16_t)frame[8] >> 4 | ((uint16_t)frame[9] << 4)) & 0x07FF;
                elrs_data.channels[5] = ((uint16_t)frame[9] >> 7 | ((uint16_t)frame[10] << 1) | ((uint16_t)frame[11] << 9)) & 0x07FF;
                elrs_data.channels[6] = ((uint16_t)frame[11] >> 2 | ((uint16_t)frame[12] << 6)) & 0x07FF;
                elrs_data.channels[7] = ((uint16_t)frame[12] >> 5 | ((uint16_t)frame[13] << 3)) & 0x07FF;
                elrs_data.channels[8] = ((uint16_t)frame[14] >> 0 | ((uint16_t)frame[15] << 8)) & 0x07FF;
                elrs_data.channels[9] = ((uint16_t)frame[15] >> 3 | ((uint16_t)frame[16] << 5)) & 0x07FF;
                elrs_data.channels[10] = ((uint16_t)frame[16] >> 6 | ((uint16_t)frame[17] << 2) | ((uint16_t)frame[18] << 10)) & 0x07FF;
                elrs_data.channels[11] = ((uint16_t)frame[18] >> 1 | ((uint16_t)frame[19] << 7)) & 0x07FF;
                elrs_data.channels[12] = ((uint16_t)frame[19] >> 4 | ((uint16_t)frame[20] << 4)) & 0x07FF;
                elrs_data.channels[13] = ((uint16_t)frame[20] >> 7 | ((uint16_t)frame[21] << 1) | ((uint16_t)frame[22] << 9)) & 0x07FF;
                elrs_data.channels[14] = ((uint16_t)frame[22] >> 2 | ((uint16_t)frame[23] << 6)) & 0x07FF;
                elrs_data.channels[15] = ((uint16_t)frame[23] >> 5 | ((uint16_t)frame[24] << 3)) & 0x07FF;

                elrs_data.Left_X = float_Map_with_median(elrs_data.channels[3], 174, 1808, 992, -100, 100);
                elrs_data.Left_Y = float_Map_with_median(elrs_data.channels[2], 174, 1811, 992, -100, 100);
                elrs_data.Right_X = float_Map_with_median(elrs_data.channels[0], 174, 1811, 992, -100, 100);
                elrs_data.Right_Y = float_Map_with_median(elrs_data.channels[1], 174, 1808, 992, -100, 100);

                elrs_data.A = elrs_data.channels[8] > 1000 ? 1 : 0;
                elrs_data.B = elrs_data.channels[6] == 191 ? 1 : (elrs_data.channels[6] == 997 ? 0 : 2);
                elrs_data.C = elrs_data.channels[7] == 191 ? 1 : (elrs_data.channels[7] == 997 ? 0 : 2);
                elrs_data.D = elrs_data.channels[9] > 1000 ? 1 : 0;
                elrs_data.E = elrs_data.channels[4] == 191 ? 1 : (elrs_data.channels[4] == 997 ? 0 : 2);
                elrs_data.F = elrs_data.channels[5] == 191 ? 1 : (elrs_data.channels[5] == 997 ? 0 : 2);
                
                // 成功解析一包后，可以选择 return，也可以继续循环找下一包（如果有粘包）
                // 这里选择 return，因为通常一次中断只处理一包最新的
                return; 
            }
            else if (frameType == CRSF_FRAMETYPE_LINK_STATISTICS)
            {
                elrs_data.uplink_RSSI_1 = frame[3];
                elrs_data.uplink_RSSI_2 = frame[4];
                elrs_data.uplink_Link_quality = frame[5];
                elrs_data.uplink_SNR = frame[6];
                elrs_data.active_antenna = frame[7];
                elrs_data.rf_Mode = frame[8];
                elrs_data.uplink_TX_Power = frame[9];
                elrs_data.downlink_RSSI = frame[10];
                elrs_data.downlink_Link_quality = frame[11];
                elrs_data.downlink_SNR = frame[12];
                return;
            }
            else if (frameType == CRSF_FRAMETYPE_HEARTBEAT)
            {
                elrs_data.heartbeat_counter = frame[3];
                return;
            }
        }
    }
    
    // 如果循环结束都没找到头，说明这批数据全是垃圾数据，或者包头被截断了
    // 清空 buffer 准备下一次
    memset(elrs_rx_buffer, 0, sizeof(elrs_rx_buffer));
}

void ELRS_Task(void)
{
   
    ELRS_Init();
    uartElrs.SetRxCallback(ELRS_UARTE_RxCallback);
}