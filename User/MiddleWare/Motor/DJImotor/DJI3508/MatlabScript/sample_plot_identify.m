%% DJI3508 串口采集、单位映射、绘图与系统辨识（基于 MCU 62 字节帧）
% 功能：
% - 串口按帧采集：帧头 0xFF 0xFF + 10 个样本，每样本 3 路 int16（指令电流、RPM、反馈电流）
% - 单位转换：电流 int16→mA（映射到 -20000 mA ~ +20000 mA），速度 RPM→输出轴 rad/s（除以减速比）
% - 绘图：时间序列三通道（指令电流、角速度、反馈电流）
% - 系统辨识：输入=指令电流（A，去均值），输出=角速度（rad/s，去均值），拟合一阶/二阶传递函数并打印

clear; close all; clc;

%% 基本配置（根据你的 MCU 采样与通信设置）
SERIAL_PORT = 'COM9';           % 串口号（按需修改）
BAUD_RATE   = 460800;           % 波特率（与 MCU 保持一致）

ACTUAL_SAMPLING_FREQ_HZ = 2000; % MCU 端采样频率（tim7 = 2kHz）
SAMPLE_TIME_SEC          = 20;   % 采样时长（建议与扫频时长一致）

SAMPLES_PER_FRAME = 10;         % 每帧样本数（与你的打包逻辑一致）
BYTES_PER_SAMPLE  = 6;          % 每样本 3 路 int16（高字节在前），共 6 字节
FRAME_LEN         = 2 + SAMPLES_PER_FRAME * BYTES_PER_SAMPLE;  % 2 字节帧头 + 10*6 = 62 字节
EXPECTED_PACKETS  = (ACTUAL_SAMPLING_FREQ_HZ / SAMPLES_PER_FRAME) * SAMPLE_TIME_SEC; % 每秒 400 帧

% 物理单位映射常量
CURRENT_SCALE_MA = 20000/16384; % 每 LSB 对应的 mA（±20000mA 满量程）
CURRENT_SCALE_A  = 20/16384;    % 每 LSB 对应的 A（±20A 满量程）
GearBoxRate      = 3591/187;    % 说明书减速比（电机轴/输出轴）
Fs = ACTUAL_SAMPLING_FREQ_HZ;   % 采样频率
Ts = 1/Fs;                      % 采样周期

%% 打开串口并开始采集
fprintf('串口采集：%s @ %d baud，共 %d 帧（%d 秒）...\n', SERIAL_PORT, BAUD_RATE, EXPECTED_PACKETS, SAMPLE_TIME_SEC);
try
    delete(serialportfind("Port", SERIAL_PORT));
catch
end
sp = serialport(SERIAL_PORT, BAUD_RATE);
configureCallback(sp, "off");
flush(sp);

bytes = zeros(EXPECTED_PACKETS*FRAME_LEN,1,'uint8'); % 预分配存储所有帧的字节缓存
wpos  = 1;                                           % 写入位置指针

tic;
for pk = 1:EXPECTED_PACKETS
    % 查找帧头 0xFF 0xFF（高字节在前）
    b1 = uint8(0); b2 = uint8(0);
    while true
        b1 = read(sp,1,"uint8");
        if isempty(b1), continue; end
        if b1 == 255
            b2 = read(sp,1,"uint8");
            if ~isempty(b2) && b2 == 255
                break; % 找到帧头，跳出循环
            end
        end
    end

    % 读取剩余 payload（FRAME_LEN-2 字节）
    payload = read(sp, FRAME_LEN-2, "uint8");
    if numel(payload) ~= FRAME_LEN-2
        warning('帧长度不一致，期望 %d，实际 %d；跳过该帧。', FRAME_LEN-2, numel(payload));
        continue;
    end

    % 写入 bytes 缓存（含帧头）
    bytes(wpos)   = uint8(255);
    bytes(wpos+1) = uint8(255);
    bytes(wpos+2:wpos+FRAME_LEN-1) = payload;
    wpos = wpos + FRAME_LEN;

    % 进度提示（每 50 帧）
    if mod(pk,50)==0
        progress = pk/EXPECTED_PACKETS*100;
        elapsed  = toc; % 实时耗时
        fprintf('进度：%.1f%% (%d/%d帧)，已耗时：%.2fs\n', progress, pk, EXPECTED_PACKETS, elapsed);
    end
end
clear sp;

% 截断至有效采集长度
bytes = bytes(1:wpos-1);
fprintf('采集完成，共读取 %d 字节。\n', numel(bytes));

%% 解析帧为三路时间序列（指令电流、RPM、反馈电流）
n_frames = floor(numel(bytes)/FRAME_LEN);
bytes    = bytes(1:n_frames*FRAME_LEN);
data     = reshape(bytes, FRAME_LEN, n_frames); % 每列一帧

payload  = data(3:end,:); % 去掉帧头
% 使用typecast和swapbytes进行稳健的大端序int16转换
val = swapbytes(typecast(uint8(payload(:)'), 'int16'));
val = reshape(val, 3, []); % 每列3个元素（指令、速度、反馈）

u_cmd_i16 = double(squeeze(val(1,:,:)));  % 指令电流（C620 标度）
rpm_i16   = double(squeeze(val(2,:,:)));  % 转速（RPM）
cur_i16   = double(squeeze(val(3,:,:)));  % 反馈电流（C620 标度）

% 展平为一维列向量（与时间对齐）
u_cmd_i16 = u_cmd_i16(:);
rpm_i16   = rpm_i16(:);
cur_i16   = cur_i16(:);
N = numel(u_cmd_i16);
t = (0:N-1)'/Fs;

%% 单位转换到物理量
u_mA   = u_cmd_i16 * CURRENT_SCALE_MA;            % 指令电流（mA，映射到 ±20000 mA）
cur_mA = cur_i16   * CURRENT_SCALE_MA;            % 反馈电流（mA）
y_rad_s = (rpm_i16 * 2*pi/60) / GearBoxRate;      % 输出轴角速度（rad/s）

% 小信号辨识前，去除 DC 偏置（可选）
u_A_d     = detrend(u_cmd_i16 * CURRENT_SCALE_A, 'constant'); % 输入（A）去均值
y_rad_s_d = detrend(y_rad_s, 'constant');                      % 输出（rad/s）去均值

%% 绘制时间序列（与 my_sample2.m 类似风格，多子图）
figure(1); clf;
subplot(3,1,1); plot(t, u_mA, 'b-'); grid on;
xlabel('时间 (s)'); ylabel('指令电流 (mA)'); title('指令电流时间序列');

subplot(3,1,2); plot(t, y_rad_s, 'r-'); grid on;
xlabel('时间 (s)'); ylabel('角速度 (rad/s)'); title('输出轴角速度时间序列');

subplot(3,1,3); plot(t, cur_mA, 'k-'); grid on;
xlabel('时间 (s)'); ylabel('反馈电流 (mA)'); title('反馈电流时间序列');

%% 频响估计与 Bode 图（输入=指令电流 A，输出=角速度 rad/s）
data_id = iddata(y_rad_s_d, u_A_d, Ts);
frf = spa(data_id);
figure(2); clf; bode(frf); grid on; % 绘制 Bode（幅值/相位）

%% 传递函数拟合（一阶与二阶）并打印
sys1 = tfest(data_id, 1); % 一阶：K/(T s + 1)
sys2 = tfest(data_id, 2); % 二阶：K*wn^2/(s^2 + 2*zeta*wn*s + wn^2)

% 时域对比验证
figure(3); clf; compare(data_id, sys1, sys2); grid on; title('辨识模型时域对比（去均值）');

% 打印传递函数多项式形式
[num1, den1] = tfdata(sys1, 'v');
[num2, den2] = tfdata(sys2, 'v');
fprintf('\n一阶传递函数（tfest）:\nG1(s) = (%s) / (%s)\n', polystr(num1), polystr(den1));
fprintf('二阶传递函数（tfest）:\nG2(s) = (%s) / (%s)\n\n', polystr(num2), polystr(den2));

% 保存结果，便于复现与后处理
save('id_results.mat','sys1','sys2','Fs','Ts','GearBoxRate');

%% 辅助函数：把多项式系数向量格式化为字符串
function s = polystr(c)
    var = 's'; n = numel(c); s = '';
    for i = 1:n
        coef = c(i); pow = n - i;
        if abs(coef) < 1e-12, continue; end
        if isempty(s)
            prefix = '';
        elseif coef >= 0
            prefix = ' + ';
        else
            prefix = ' - '; coef = -coef;
        end
        if pow == 0
            term = sprintf('%.6g', coef);
        elseif pow == 1
            term = sprintf('%.6g*%s', coef, var);
        else
            term = sprintf('%.6g*%s^%d', coef, var, pow);
        end
        s = [s prefix term]; %#ok<AGROW>
    end
    if isempty(s), s = '0'; end
end

