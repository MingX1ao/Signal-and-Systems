% 设置参数
length = 256;
fs = 0.5;
t = linspace(0, length / fs, length);
noise = randn(1, length);
signal = 1 + 3*sin(2*pi*8/521*t) + cos(2*pi*4/512*t) + sin(2*pi*32/512*t);
signal_and_noise = signal + noise;
S = abs(fft(signal));
S = S(1:length/2+1);
% 滤波器的模拟要求
flp = 0.1;
fls = 0.15;
fhs = 0.2;
As = -50;
Ap = -0.02;
% 第一张图
figure;
subplot(2, 1, 1);
plot(t, signal_and_noise, t, signal);
title('the waveform of signal plus noise');
xlabel('time/s');
ylabel('signal plus noise');

S_a_N = abs(fft(signal_and_noise));
S_a_N = S_a_N(1:length/2+1);
f = fs * (0:(length+1)/2)/length;
subplot(2, 1, 2);
plot(f, S_a_N, f, S);
title('the FFT waveform of signal and noise');
xlabel('frequency/Hz');
ylabel('amplitude');

% 1. 窗函数法FIR滤波器
% 根据通带波纹和阻带衰减，选择Hamming窗
deltaW = 2*pi*(fls-flp)/fs;
N = ceil(3.3*2*pi/deltaW);
Wc = (flp+fls)/2/fs*2*pi;
B = fir1(N-1, Wc/pi, "low", hamming(N));
[ham_filter, f1] = freqz(B, 1, 512, fs);
% 第二张图
figure;
% 幅频响应
subplot(2, 1, 1);
plot(f1, 20*log10(abs(ham_filter)));
title('Amplitude-frequency response of the filter');
xlabel('frequency/Hz');
ylabel('gain/dB');
% 相频响应
subplot(2, 1, 2);
plot(f1, unwrap(angle(ham_filter)));
title('Amplitude-frequency response of the filter');
xlabel('frequency/Hz');
ylabel('phase_shift/rad');
% 滤波
lp_filtered_signal = filter(B, 1, signal_and_noise, []);
% 第三张图
figure;
subplot(2, 1, 1);
plot(t, lp_filtered_signal, t, signal);
title('the waveform of filtered signal plus noise');
xlabel('time/s');
ylabel('filtered signal plus noise');

S_a_N_f = abs(fft(lp_filtered_signal));
S_a_N_f = S_a_N_f(1:length/2+1);
subplot(2, 1, 2);
plot(f, S_a_N_f, f, S);
title('the FFT waveform of filtered signal and noise');
xlabel('frequency/Hz');
ylabel('amplitude');


% 2. IIR滤波器
% 使用双线性变换法
wlp = 2*fs*tan(2*pi*flp/fs/2);
wls = 2*fs*tan(2*pi*fls/fs/2);
whs = 2*fs*tan(2*pi*fhs/fs/2);
% 没给通带衰减，用buttord计算，都用的模拟量
[N1, Wc2] = buttord(wlp, wls, Ap, As, 's');
[z, p] = butter(N1, Wc2, 'low', 's');
[b, a] = bilinear(z, p, fs);
[hz, f2] = freqz(b, a, 512, fs);
% 第四张图
figure;
% 幅频响应
subplot(2, 1, 1);
plot(f2, 20*log10(abs(hz)));
title('Amplitude-frequency response of the filter');
xlabel('frequency/Hz');
ylabel('gain/dB');
% 相频响应
subplot(2, 1, 2);
plot(f2, unwrap(angle(hz)));
title('Amplitude-frequency response of the filter');
xlabel('frequency/Hz');
ylabel('phase_shift/rad');

% 滤波
IIR_filtered_signal = filter(b, a, signal_and_noise, []);
% 第五张图
figure;
subplot(2, 1, 1);
plot(t, IIR_filtered_signal, t, signal);
title('the waveform of filtered signal plus noise');
xlabel('time/s');
ylabel('filtered signal plus noise');

S_a_N_IIR = abs(fft(IIR_filtered_signal));
S_a_N_IIR = S_a_N_IIR(1:length/2+1);
subplot(2, 1, 2);
plot(f, S_a_N_IIR, f, S);
title('the FFT waveform of filtered signal and noise');
xlabel('frequency/Hz');
ylabel('amplitude');