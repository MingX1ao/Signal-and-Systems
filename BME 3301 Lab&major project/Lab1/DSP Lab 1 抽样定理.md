# DSP Lab 1 抽样定理

## 1 时域信号的采样

代码如下

```matlab
T = 512;

[t, signal] = sampleSignal(2, T);
[t1, signal1] = sampleSignal(0.1, T);
[t2, signal2] = sampleSignal(0.2, T);
[t3, signal3] = sampleSignal(0.5, T);

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

function [sampX, signal] = sampleSignal(Fs, T)
    sampX = 0:1/Fs:T-1/Fs;
    signal = 1+3*sin(2*pi*8/512*sampX)+cos(2*pi*4/512*sampX)+sin(2*pi*32/512*sampX);
end
```

得到的结果如下

<center><img src="time domi.bmp"></center>

抽样的频率越高，得到的信号越还原原信号，符合预期



## 2 频谱分析

FFT后`[0,Fs/2]`和`[Fs/2,Fs]`的图像是对称的

代码如下

```matlab
% 第二个图：频谱分析
figure();

subplot(4, 1, 1);
[omega, S] = Fourier(signal, 2);
plot(omega, S);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum of Original Signal', 'FontSize', 14);
grid on;

subplot(4, 1, 2);
[omega1, S1] = Fourier(signal1, 0.1);
plot(omega1, S1);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum at sampling Freq 0.1Hz', 'FontSize', 14);
grid on;

subplot(4, 1, 3);
[omega2, S2] = Fourier(signal2, 0.2);
plot(omega2, S2);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum at sampling Freq 0.2Hz', 'FontSize', 14);
grid on;

subplot(4, 1, 4);
[omega3, S3] = Fourier(signal3, 0.5);
plot(omega3, S3);
xlim([0, 0.25]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
title('Frequency Spectrum at sampling Freq 0.5Hz', 'FontSize', 14);
grid on;

function [omega, S] = Fourier(signal, Fs)
    N = length(signal);
    Y = fft(signal);
    S = abs(Y);
    S = S(1:floor(n/2)+1);
    omega = Fs*(0:N/2)/N;	
end
```

得到的结果如下

<center><img src="freq domi.bmp"></center>

原信号是包括了$f=\frac{4}{512},\frac{8}{512},\frac{32}{512},0\mathsf{Hz}$的信号，只有使用$f_s>0.125\mathsf{Hz}$的采样信号才能完全采集所有频率的分量。

当使用$f_s=0.1\mathsf{Hz}$的冲击串采样时，在0.0375~0.1Hz的频率区间采样信号发生混叠，在图上就表现为本应在$f=0.0625\mathsf{Hz}$出现的冲激信号，出现在了$f=0.1-0.0625=0.0375\mathsf{Hz}$的区域

当使用$f_s=0.2\mathsf{Hz}$或$f_s=0.5\mathsf{Hz}$的冲击串采样时，就能在频谱上完全找到所有对应的冲激信号，但是还是不完全一样，也许和FFT的近似算法有关





## 3 无失真地恢复的采样频率

信号的最大频率是$\frac{32}{512}\mathsf{Hz}$，需要两倍的抽样频率才行，故理论上需要$>\frac{64}{512}=0.125\mathsf{Hz}$的频率抽样才能完全分离，刚好相等也不行

做了一个0.125Hz的采样，结果如下：

<center><img src="1.bmp"></center>

位于0.0625Hz的峰正好消失

当采样频率为0.126Hz时，峰正好出现，不赘述