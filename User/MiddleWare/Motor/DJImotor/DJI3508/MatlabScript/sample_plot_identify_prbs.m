% PRBS 采集与系统辨识脚本
% - 与 MCU 端 62 字节帧格式对齐：帧头 0xFF 0xFF + 10 样本/帧，每样本 3 路 int16（指令电流、RPM、反馈电流）
% - 单位映射：电流映射到 ±20000 mA（±20 A），RPM 映射到输出轴角速度（rad/s）
% - 数据处理：去均值（detrend）后进行频响估计（spa）与传递函数拟合（tfest）
clear; close all; clc;

% 串口配置（需与 MCU 保持一致）
SERIAL_PORT = 'COM9';
BAUD_RATE   = 460800;

% 采样频率与采样时长（与 MCU tim7 设置一致）
ACTUAL_SAMPLING_FREQ_HZ = 2000; % 2 kHz
SAMPLE_TIME_SEC          = 12;  % 采集总时长（秒）

% 帧结构（与 MCU 打包一致）
SAMPLES_PER_FRAME = 10;                    % 每帧样本数
BYTES_PER_SAMPLE  = 6;                     % 每样本 3 路 int16，共 6 字节（高字节在前）
FRAME_LEN         = 2 + SAMPLES_PER_FRAME * BYTES_PER_SAMPLE;  % 62 字节
EXPECTED_PACKETS  = (ACTUAL_SAMPLING_FREQ_HZ / SAMPLES_PER_FRAME) * SAMPLE_TIME_SEC; % 每秒 200 帧

% 物理常量映射
CURRENT_SCALE_MA = 20000/16384; % mA/LSB（±20000 mA 满量程）
CURRENT_SCALE_A  = 20/16384;    %  A/LSB（±20 A 满量程）
GearBoxRate      = 3591/187;    % 减速比（电机轴/输出轴）
Fs = ACTUAL_SAMPLING_FREQ_HZ;   % 采样频率
Ts = 1/Fs;                      % 采样周期

% 打开串口（如存在旧对象先清理）
try
    delete(serialportfind("Port", SERIAL_PORT));
catch
end
sp = serialport(SERIAL_PORT, BAUD_RATE);
configureCallback(sp, "off");
flush(sp);

% 打印采集计划并启动计时器
fprintf('串口采集：%s @ %d baud，共 %d 帧（%d 秒）...\n', SERIAL_PORT, BAUD_RATE, EXPECTED_PACKETS, SAMPLE_TIME_SEC);
ticId = tic; % 用于统计已耗时

% 预分配原始字节缓存（按帧存储）
bytes = zeros(EXPECTED_PACKETS*FRAME_LEN,1,'uint8');
wpos  = 1; % 当前写入位置

% 逐帧采集（寻找帧头 0xFF 0xFF，随后读取 62 字节载荷）
for pk = 1:EXPECTED_PACKETS
    b1 = uint8(0); b2 = uint8(0);
    while true
        b1 = read(sp,1,"uint8");
        if isempty(b1), continue; end
        if b1 == 255
            b2 = read(sp,1,"uint8");
            if ~isempty(b2) && b2 == 255
                break; % 找到帧头
            end
        end
    end
    payload = read(sp, FRAME_LEN-2, "uint8");
    if numel(payload) ~= FRAME_LEN-2
        continue; % 长度异常，跳过该帧
    end
    bytes(wpos)   = uint8(255);
    bytes(wpos+1) = uint8(255);
    bytes(wpos+2:wpos+FRAME_LEN-1) = payload;
    wpos = wpos + FRAME_LEN;

    % 采集进度打印（每 50 帧一次）
    if mod(pk,50) == 0
        progress = pk / EXPECTED_PACKETS * 100;
        elapsed  = toc(ticId);
        fprintf('进度：%.1f%% (%d/%d帧)，已耗时：%.2fs\n', progress, pk, EXPECTED_PACKETS, elapsed);
    end
end
clear sp;

bytes = bytes(1:wpos-1);

n_frames = floor(numel(bytes)/FRAME_LEN);
bytes    = bytes(1:n_frames*FRAME_LEN);
data     = reshape(bytes, FRAME_LEN, n_frames);

payload  = data(3:end,:);
val = swapbytes(typecast(uint8(payload(:)'), 'int16'));
val = reshape(val, 3, []);

u_cmd_i16 = double(squeeze(val(1,:,:)));
rpm_i16   = double(squeeze(val(2,:,:)));
cur_i16   = double(squeeze(val(3,:,:)));

u_cmd_i16 = u_cmd_i16(:);
rpm_i16   = rpm_i16(:);
cur_i16   = cur_i16(:);
N = numel(u_cmd_i16);
t = (0:N-1)'/Fs;

u_mA   = u_cmd_i16 * CURRENT_SCALE_MA;
cur_mA = cur_i16   * CURRENT_SCALE_MA;
y_rad_s = (rpm_i16 * 2*pi/60) / GearBoxRate;

u_A_d     = detrend(u_cmd_i16 * CURRENT_SCALE_A, 'constant');
y_rad_s_d = detrend(y_rad_s, 'constant');

figure(1); clf;
subplot(3,1,1); plot(t, u_mA, 'b-'); grid on;
xlabel('时间 (s)'); ylabel('指令电流 (mA)');
subplot(3,1,2); plot(t, y_rad_s, 'r-'); grid on;
xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
subplot(3,1,3); plot(t, cur_mA, 'k-'); grid on;
xlabel('时间 (s)'); ylabel('反馈电流 (mA)');

data_id = iddata(y_rad_s_d, u_A_d, Ts);
frf = spa(data_id);
figure(2); clf; bode(frf); grid on;

sys1 = tfest(data_id, 1);
sys2 = tfest(data_id, 2);
figure(3); clf; compare(data_id, sys1, sys2); grid on;

[num1, den1] = tfdata(sys1, 'v');
[num2, den2] = tfdata(sys2, 'v');
fprintf('\n一阶传递函数:\n'); disp(sys1);
fprintf('\n二阶传递函数:\n'); disp(sys2);