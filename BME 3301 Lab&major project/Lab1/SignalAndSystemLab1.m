% function analyze_BOLD_signal()
%     % 设置基本参数
%     T = 512;  % 总时间长度(s)
%     fs_list = [0.1, 0.2, 0.5];  % 采样频率列表(Hz)
% 
%     % 创建图形窗口
%     figure('Position', [100 100 1200 800]);
% 
%     % 对每个采样频率进行分析
%     for i = 1:length(fs_list)
%         fs = fs_list(i);
%         t = 0:1/fs:T-1/fs;  % 时间序列
% 
%         % 生成信号
%         signal = 1 + 3*sin(2*pi*8/512*t) + cos(2*pi*4/512*t) + sin(2*pi*32/512*t);
% 
%         % 绘制时域信号
%         subplot(2, length(fs_list), i);
%         plot(t, signal, 'b-');
%         title(['Time Domain Signal (fs = ', num2str(fs), ' Hz)']);
%         xlabel('Time (s)');
%         ylabel('Amplitude');
%         grid on;
% 
%         % 计算FFT
%         N = length(signal);
%         Y = fft(signal);
%         f = fs*(0:(N/2))/N;  % 频率轴
%         P2 = abs(Y);
%         P1 = P2(1:floor(N/2)+1);
%         P1(2:end-1) = 2*P1(2:end-1);
% 
%         % 绘制频谱
%         subplot(2, length(fs_list), i+length(fs_list));
%         plot(f, P1, 'r-');
%         title(['Frequency Spectrum (fs = ', num2str(fs), ' Hz)']);
%         xlabel('Frequency (Hz)');
%         ylabel('Amplitude');
%         grid on;
% 
%         % 设置频率轴范围
%         xlim([0 fs/2]);  % 显示到奈奎斯特频率
%     end
% end


% T = 512;
% 
% [t, signal] = sampleSignal(512, T);
% [t1, signal1] = sampleSignal(0.1, T);
% [t2, signal2] = sampleSignal(0.2, T);
% [t3, signal3] = sampleSignal(0.5, T);
% 
% figure();
% 
% subplot(4, 1, 1);
% plot(t,signal);
% xlim([0, 512]);
% xlabel('time(s)', 'FontSize', 12);
% ylabel('signal', 'FontSize', 12);
% title('Original Signal', 'FontSize', 14);
% 
% 
% subplot(4, 1, 2);
% stem(t1, signal1);
% xlim([0, 512]);
% xlabel('time(s)', 'FontSize', 12);
% ylabel('signal', 'FontSize', 12);
% title('Signal at sampling Freq 0.1Hz', 'FontSize', 14);
% 
% 
% subplot(4, 1, 3);
% stem(t2,signal2);
% xlim([0, 512]);
% xlabel('time(s)', 'FontSize', 12);
% ylabel('signal', 'FontSize', 12);
% title('Signal at sampling Freq 0.2Hz', 'FontSize', 14);
% 
% subplot(4, 1, 4);
% stem(t3,signal3);
% xlim([0, 512]);
% xlabel('time(s)', 'FontSize', 12);
% ylabel('signal', 'FontSize', 12);
% title('Signal at sampling Freq 0.5Hz', 'FontSize', 14);
% 
% 
% [omega, amplitude] = Fourier(signal, 512);
% [omega1, amplitude1] = Fourier(signal1, 0.1);
% [omega2, amplitude2] = Fourier(signal2, 0.2);
% [omega3, amplitude3] = Fourier(signal3, 0.5);
% 
% figure();
% 
% subplot(4, 1, 1);
% plot(omega, amplitude);
% 
% subplot(4, 1, 2);
% plot(omega1, amplitude1);
% 
% subplot(4, 1, 3);
% plot(omega2, amplitude2);
% 
% subplot(4, 1, 4);
% plot(omega3, amplitude3);
% 
% 
% function [sampX, signal] = sampleSignal(Fs, T)
%     sampX = linspace(0, T, T * Fs);
%     signal = 1+3*sin(2*pi*8/512*sampX)+cos(2*pi*4/512*sampX)+sin(2*pi*32/512*sampX);
% end
% 
% function [omegaX, amplitude] =  Fourier(signal, Fs)
%     N = length(signal);
%     S = fft(signal);
%     omega = Fs*(0:(N-1))/N;  % 频率轴
%     amplitude = abs(S);
%     omegaX = omega(1:floor(N/2)+1);
%     omegaX(2:end-1) = 2*omegaX(2:end-1);
% end


T = 512;

[t, signal] = sampleSignal(2, T);
[t1, signal1] = sampleSignal(0.1, T);
[t2, signal2] = sampleSignal(0.2, T);
[t3, signal3] = sampleSignal(0.5, T);
[t4, signal4] = sampleSignal(0.125, T);

% 第一个图：时域信号
figure();

subplot(4, 1, 1);
plot(t,signal);
xlim([0, 512]);
xlabel('time(s)', 'FontSize', 12);
ylabel('signal', 'FontSize', 12);
title('Original Signal', 'FontSize', 14);

subplot(4, 1, 2);
stem(t1, signal1);
xlim([0, 512]);
xlabel('time(s)', 'FontSize', 12);
ylabel('signal', 'FontSize', 12);
title('Signal at sampling Freq 0.1Hz', 'FontSize', 14);

subplot(4, 1, 3);
stem(t2,signal2);
xlim([0, 512]);
xlabel('time(s)', 'FontSize', 12);
ylabel('signal', 'FontSize', 12);
title('Signal at sampling Freq 0.2Hz', 'FontSize', 14);

subplot(4, 1, 4);
stem(t3,signal3);
xlim([0, 512]);
xlabel('time(s)', 'FontSize', 12);
ylabel('signal', 'FontSize', 12);
title('Signal at sampling Freq 0.5Hz', 'FontSize', 14);

% 第二个图：频谱分析
figure();

subplot(4, 1, 1);
[f, P1] = Fourier(signal, 2);
plot(f, P1);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum of Original Signal', 'FontSize', 14);
grid on;

subplot(4, 1, 2);
[f1, P1_1] = Fourier(signal4, 0.125);
plot(f1, P1_1);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum at sampling Freq 0.125Hz', 'FontSize', 14);
grid on;

subplot(4, 1, 3);
[f2, P1_2] = Fourier(signal2, 0.2);
plot(f2, P1_2);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum at sampling Freq 0.2Hz', 'FontSize', 14);
grid on;

subplot(4, 1, 4);
[f3, P1_3] = Fourier(signal3, 0.5);
plot(f3, P1_3);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum at sampling Freq 0.5Hz', 'FontSize', 14);
grid on;

function [sampX, signal] = sampleSignal(Fs, T)
    sampX = 0:1/Fs:T-1/Fs;
    signal = 1+3*sin(2*pi*8/512*sampX)+cos(2*pi*4/512*sampX)+sin(2*pi*32/512*sampX);
end

function [f, P2] = Fourier(signal, Fs)
    N = length(signal);
    Y = fft(signal);
    P2 = abs(Y);
    P2 = P2(1:floor(N/2)+1);
    %P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(N/2))/N;
end




















