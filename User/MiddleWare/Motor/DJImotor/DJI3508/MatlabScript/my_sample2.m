%% DJI3508电机电流数据采集与绘图脚本
% 功能：从STM32下位机接收电流数据，进行类型转换并实时绘图
% 数据格式：起始符'0xFFFF' + 10组int16电流数据(共22字节)
% 电流单位：毫安(mA) - 已从安培(A)转换为毫安(mA)以提高显示精度
% 作者：郑涵缤
% 日期：2025-11-2

% 清理工作区
clear all;
close all;
clc;

% 串口参数配置
SERIAL_PORT = 'COM9';       %端口号
BAUD_RATE   = 460800;       %波特率
PACKET_SIZE = 22    ;       %数据包大小
PACKET_HEAD = 2     ;       %数据包头
CURRENT_SAMPLES = 10;       %每个数据包的电流采样点数目

% 采样频率和时间设置
ACTUAL_SAMPLING_FREQ_HZ = 2000;    %C++端实际采样频率
PACKET_SEND_FREQ_HZ = 200;         %C++端数据包发送频率(2000/10=200)
SAMPLING_TIME =  1;               %采样总时间(20s)
SAMPLING_INTERVAL = 1/ACTUAL_SAMPLING_FREQ_HZ; %单个采样点间隔
 
% 定义缓冲区容量（基于实际采样频率）
BUFFER_SIZE =  ACTUAL_SAMPLING_FREQ_HZ*SAMPLING_TIME;

% 定义t轴和current轴
t_Buffer =  zeros(1,BUFFER_SIZE);
current_Buffer = zeros(1,BUFFER_SIZE);

% 当前的数据存入点
sample_index = 1;

fprintf('缓冲区大小：%d 个数据点，可存储 %.1f 秒的数据\n',BUFFER_SIZE,SAMPLING_TIME);

% 创建串口
try
    %删除存在串口
    delete(serialportfind);
    %寻找串口
    serial = serialport(SERIAL_PORT,BAUD_RATE);
    %打印消息
    fprintf('串口 %s 已成功打开，波特率：%d\n', SERIAL_PORT, BAUD_RATE);
catch ME
    fprintf('创建串口失败,%s',ME.message);
end

% 计算预期的数据包数量
EXPECTED_PACKETS = PACKET_SEND_FREQ_HZ * SAMPLING_TIME;
packet_count = 0;

fprintf('预期接收数据包数量：%d 个\n', EXPECTED_PACKETS);

% 基于数据包数量和缓冲区大小进行采集
while (packet_count < EXPECTED_PACKETS && sample_index+CURRENT_SAMPLES-1<=BUFFER_SIZE)
    % 寻找数据包头
    isFound = findPacketstart(serial);
    % 找到数据包
    if(isFound == true)
        % 读取数据包
        sample_buffer = serial.read(PACKET_SIZE-PACKET_HEAD,"uint8");
        % 记录数据包接收时间（用于计算每个采样点的实际时间）
        packet_time = packet_count / PACKET_SEND_FREQ_HZ;
        
        for i = 1 :CURRENT_SAMPLES
            % 获取当前数据包的两个字节数据
            highByte = sample_buffer(2*i-1);
            lowByte  = sample_buffer(2*i);
            % 合并数据包成单个电流数据
            currentByte = bitor(bitshift(int16(highByte),8),int16(lowByte));
            % 电流float数据
            current = transform(currentByte);

            % 计算每个采样点的实际时间
            % 每个数据包内的10个点是在5ms内连续采样的
            point_offset = (i-1) * SAMPLING_INTERVAL;
            t_Buffer(sample_index) = packet_time + point_offset;
            
            % 赋值对应时间的电流
            current_Buffer(sample_index) = current;
            % 更新当前采样数据包
            sample_index = sample_index + 1;
        end
        
        % 增加数据包计数
        packet_count = packet_count + 1;
        
        % 每50个数据包显示一次进度
        if mod(packet_count, 50) == 0
            progress = packet_count / EXPECTED_PACKETS * 100;
            elapsed_time = packet_count / PACKET_SEND_FREQ_HZ;
            fprintf('进度：%.1f%% (%d/%d包)，已耗时：%.2f秒\n', ...
                    progress, packet_count, EXPECTED_PACKETS, elapsed_time);
        end
    end
end
fprintf('采样完成\r\n');
% 创建图形窗口
%f1 = figure('Name','DJI3508电机电流采样(mA)','NumberTitle','off');
%p1 = plot(t_Buffer,current_Buffer);
%grid on;
%xlabel('时间(ms)');
%ylabel('电流(ma)');
% 图形标题
%title('DJI3508电机电流采样波形(毫安)');

% 更详细的频域分析
figure('Name','DJI3508电机电流频域分析','NumberTitle','off');

% 计算FFT
% 采样频率
Fs = ACTUAL_SAMPLING_FREQ_HZ;
% 采样周期
T  = 1/Fs;
% 采样点数
L = length(current_Buffer);
% 对电流数据做傅里叶变换
current_fft = fft(current_Buffer);




% 子图1：时域信号
subplot(2,1,1);
plot(t_Buffer, current_Buffer,'Marker','.','MarkerSize',3,'LineStyle','-');
title('时域信号');
xlabel('时间 (s)');
ylabel('电流 (mA)');
grid on;

% 子图2 频域信号
subplot(2,1,2);
plot(Fs/L*(0:L-1), abs(current_fft),'Marker','.','MarkerSize',3,'LineStyle','-');
title('幅度谱');
xlabel('频率 (Hz)');
ylabel('幅度 (fft(current))');
grid on;

%% AI生成的代码

% 设计并应用巴特沃斯低通滤波器（常规单向滤波）
LPF_CUTOFF_HZ = 70;   % 截止频率，可按需调整（建议 80~150 Hz）
LPF_ORDER     = 4;     % 滤波器阶数（2~4 之间较稳妥）
Wn = LPF_CUTOFF_HZ / (Fs/2);           % 归一化截止频率（0~1）
Wn = min(max(Wn, 1e-6), 0.999999);     % 保护边界
[b, a] = butter(LPF_ORDER, Wn, 'low');
current_filt = filter(b, a, current_Buffer);    % 常规滤波（有相位延迟）
current_fft_filt = fft(current_filt);

% 滤波后时/频域对比
figure('Name','DJI3508电机电流低通滤波后分析','NumberTitle','off');

% 子图1：时域信号（原始 vs 低通）
subplot(2,1,1);
plot(t_Buffer, current_Buffer, 'Color', [0.7 0.7 0.7],'Marker','.','MarkerSize',3,'LineStyle','-'); hold on;
plot(t_Buffer, current_filt, 'r','Marker','.','MarkerSize',3,'LineStyle','-');
title(sprintf('时域信号（巴特沃斯低通：%d Hz，阶数 %d）', LPF_CUTOFF_HZ, LPF_ORDER));
xlabel('时间 (s)'); ylabel('电流 (mA)'); grid on;
legend('原始','低通滤波');

% 子图2：频域信号（原始 vs 低通）
subplot(2,1,2);
plot(Fs/L*(0:L-1), abs(current_fft), 'Color', [0.7 0.7 0.7],'Marker','.','MarkerSize',3,'LineStyle','-'); hold on;
plot(Fs/L*(0:L-1), abs(current_fft_filt), 'r','Marker','.','MarkerSize',3,'LineStyle','-');
title('幅度谱（原始 vs 低通）');
xlabel('频率 (Hz)'); ylabel('幅度'); grid on;
legend('原始','低通滤波');



% 是否找到包头
function [isFound] = findPacketstart(serialObj)
    isFound = false;
    maxSerch = 50;  % 减少最大搜索次数，避免过长等待
    for i = 1:maxSerch
        if serialObj.NumBytesAvailable >= 2  % 确保至少有2个字节可读
            byte = bitor(bitshift(read(serialObj,1,'uint8'),8,"uint16"),read(serialObj,1,"uint8"));
            if byte == 0xFFFF
                isFound = true;
                break;  % 找到包头后立即退出循环
            end
        else
            pause(0.001);  % 减少暂停时间从10ms到1ms
        end
    end
end 

% 转换成-20A到+20A的数据
function [current] = transform(currentByte)
    CURRENT_SCALE_MA = 1.220703125;  % mA per LSB (20000/16384，精确值)
    current = double(currentByte)*CURRENT_SCALE_MA;
end
