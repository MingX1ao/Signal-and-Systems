% 实验二：傅里叶变换实验


% 采样频率
fs = 0.5; % Hz
Ts = 1/fs; % 采样周期

% 定义时间向量
t_256 = 0:Ts:256-Ts;
t_128 = 0:Ts:128-Ts;
t_64 = 0:Ts:64-Ts;

% 生成原始信号
sig_256 = generateSignal(t_256);
sig_128 = generateSignal(t_128);
sig_64 = generateSignal(t_64);

% 绘制图一：不同长度信号的频谱
figure(1);
plotSpectrum(sig_256, fs, 'SpectrumOfN=256', 3, 1, 1);
plotSpectrum(sig_128, fs, 'SpectrumOfN=128', 3, 1, 2);
plotSpectrum(sig_64, fs, 'SpectrumOfN=64', 3, 1, 3);

% 补零处理并绘制图二
figure(2);
subplot(2,1,1);
plotSpectrum(sig_64, fs, 'SpectrumOfN=64', 2, 1, 1);

sig_64ZeroPadding = [sig_64, zeros(1, 32)];
subplot(2,1,2);
plotSpectrum(sig_64ZeroPadding, fs, 'SpectrumOfN=64 With ZeroPadding', 2, 1, 2);


% 信号分段和卷积
h = [1, 1, 1, 1];
segments = {};
segments{1} = sig_128(1:16); % 每段16点
segments{2} = sig_128(17:32);
segments{3} = sig_128(33:48);
segments{4} = sig_128(49:64);

% 绘制图三：分段卷积后的频谱
figure(3);
for i = 1:4
    seg_conv = conv(segments{i}, h);
    plotSpectrum(seg_conv, fs, sprintf('the Spectrum Of No.%d`s conv', i), 4, 1, i);
end


function signal = generateSignal(t)
    signal = 1 + 3*sin(2*pi*8*t/512) + cos(2*pi*4*t/512) + sin(2*pi*32*t/512);
end



function plotSpectrum(sig, fs, titleStr, m, n, p)
    N = length(sig);
    f = (0:N/2-1)*fs/N;
    Y = fft(sig)/2;
    subplot(m, n, p);
    stem(f, 2*abs(Y(1:N/2)));
    xlim([0 0.25]);
    title(titleStr);
    xlabel('Frequency(Hz)');
    ylabel('Amplitude');
    grid on;
end



