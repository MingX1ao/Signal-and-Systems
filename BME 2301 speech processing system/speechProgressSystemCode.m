classdef speechProgressSystemCode < matlab.apps.AppBase
    %SPEECHPROGRESSSYSTEM 状态按钮

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure          matlab.ui.Figure
        GridLayout        matlab.ui.container.GridLayout
        Graphic           matlab.ui.container.Panel
        FqcMap            matlab.ui.control.UIAxes
        TimeMap           matlab.ui.control.UIAxes
        Botton            matlab.ui.container.Panel
        restrcutEff       matlab.ui.control.Button
        undoAll           matlab.ui.control.Button
        playOrigin        matlab.ui.control.Button
        kidsVoice         matlab.ui.control.Button
        inverse           matlab.ui.control.Button
        HenceSpeech       matlab.ui.control.Button
        Display           matlab.ui.control.Button
        FrequencySlider   matlab.ui.control.Slider
        Slider_2Label     matlab.ui.control.Label
        SpeedSlider       matlab.ui.control.Slider
        Label             matlab.ui.control.Label
        unOrRecordBotton  matlab.ui.control.StateButton
        title             matlab.ui.control.Label
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    properties (Access = private)
        audioWave double    %原始的声音数组
        audioWaveBackUp double %原始音频的备份，前者用于处理
        time double          %储存时间信息
        timeSignal double   %储存时域上的信号
        omega double        %储存频率信息
        fqcSignal double     %储存频域上的信号
        FS = 44100          %采样频率
        recorder audiorecorder  %录音机
    end
    
    methods (Access = private)
        
        function startRecord(app)       %开始录音函数
            app.recorder = audiorecorder(app.FS,16,1);
            app.recorder.record();

        end
        
        function store(app)           %储存录音函数
            app.recorder.pause();
            app.audioWave = app.recorder.getaudiodata();
            app.audioWaveBackUp = app.audioWave;
            app.changeWave();
        end
        
        function drawMap(app)           %画图函数
            plot(app.TimeMap,app.time,app.timeSignal);
            plot(app.FqcMap,app.omega,app.fqcSignal);
        end
        
       
        
        function changeWave(app)        %创建时域和频域上的声音波形
            fs = app.FS;
            app.time =  linspace(0,length(app.audioWave)/fs,length(app.audioWave));
            app.timeSignal = app.audioWave;

            fqcsignal = abs(fft(app.audioWave));
            app.fqcSignal = fqcsignal;
            app.omega = fs/length(app.audioWave)*(0:length(app.audioWave)-1);
        end
        
        function enhanceSpeech(app)       %语音增强 
            fs = app.FS;
            noiseAndSignal = app.audioWave;        %将原始信号作为带噪语音
            noise = noiseAndSignal(0.5*fs:fs);      %将前50ms的信号作为噪音
            fft_noiseAndSignal = fft(noiseAndSignal); %对带噪语音做fft
            phase_fft_noiseAndSignal = angle(fft_noiseAndSignal); %因为噪音相比语音幅值小
                                                                  %对相位影响小，故认为相位不变
            fft_noise = fft(noise);                %对噪音fft
            aveAmp = mean(abs(fft_noise));     %求噪音的平均幅度
            fft_signal_Amp = zeros(length(fft_noiseAndSignal),1);
            for i = 1:length(fft_noiseAndSignal)
                if abs(fft_noiseAndSignal(i))^2-aveAmp^2 > 0                         %幅度不能小于0
                   fft_signal_Amp(i) = sqrt(abs(fft_noiseAndSignal(i))^2-aveAmp^2);      %还原语音的幅度                                                                   
                end
            end
            fft_signal = fft_signal_Amp .* exp(1i .* phase_fft_noiseAndSignal);%还原相位
            signal = ifft(fft_signal);                          %还原信号
            app.audioWave = real(signal);

            app.changeWave();
            app.drawMap();
        end
        
        
        
        
        function changeSpeed(app,speed)
            app.speechproc(app.audioWave,1,speed,1);
        end
      

        function speechproc(app,speech,mode,speed,pitch)     
            %定义常数
            FL = app.FS / 100;      %帧长
            WL = 3 * FL;            %窗长
            P = 800;                %预测系数个数
            s = speech;             %赋值
            L = length(s);
            FN = floor(L/FL) - 2;   %计算帧数
            
            %预测和重建滤波器
            exc = zeros(L,1);       %预测误差e（激励信号）
            zi_pre = zeros(P,1);    %预测滤波器的状态

            %变调不变速滤波器
            s_syn_f = zeros(L,1);   %合成语音
           
            %变速不变调滤波器
            exc_syn_s = zeros(ceil(speed*L),1); %变速激励
            s_syn_s = zeros(ceil(speed*L),1);   %合成语音
            last_syn_s = 0;                     %记录位置
            zi_syn_s = zeros(P,1);              %合成滤波器

            hw = hamming(WL);       %汉明窗
            
            %依次处理每帧语言
            if mode == 1    %变速不变调
                for n = 3:FN
                    %计算预测系数
                    s_w = s(n*FL-WL+1:n*FL) .* hw;   %汉明窗加权后的语音
                    %A是预测系数，E被用于计算合成激励的能量
                    [A, E] = lpc(s_w,P);         %用LPC计算P个预测系数
                    s_f = s((n-1)*FL+1:n*FL);   %处理这帧语言
                    %计算激励
                    %使用filter函数s_f计算激励，注意保持滤波器状态

                    [exc((n-1)*FL+1:n*FL),zi_pre] = filter([0 -A(2:end)],1,s_f,zi_pre);  %上面求出的激励就是这帧的激励

                    %下面的代码只有在得到exc后才能计算正确
                    s_Pitch = exc(n*FL-WL+1:n*FL);
                    PT = app.findpitch(s_Pitch,WL);    %计算基音周期PT
                    G = sqrt(E*PT);                    %计算合成激励的能量G



                    %不改变基音周期和预测系数，将合成激励的长度变为原先的speed倍
                    %作为filter的输入得到新的合成语音，达到变速不变调
                    FL_s = ceil(FL*speed);
                    index_s = (1:n*FL_s-last_syn_s)';
                    exc_syn1_s = zeros(length(index_s),1);
                    exc_syn1_s(mod(index_s,PT)==0) = G; %用周期脉冲信号作为激励，保持PT不变
                    exc_syn_s((n-1)*FL_s+1:n*FL_s) = exc_syn1_s((n-1)*FL_s-last_syn_s+1:n*FL_s-last_syn_s);%计算得到的加长合成激励
                    [s_syn_s((n-1)*FL_s+1:n*FL_s),zi_syn_s] = filter(1,A,exc_syn_s((n-1)*FL_s+1:n*FL_s),zi_syn_s);%计算得到的加长合成语音
                    last_syn_s = last_syn_s+PT*floor((n*FL_s-last_syn_s)/PT);       %更新位置

                end
            else                      %变调不变速
                %改变采样频率，得到变速变调的语音，再用变速不变调的算法还原速度
                fs = ceil(app.FS*pitch);
                s_syn_f = resample(app.audioWave,fs,app.FS);
            end
       
            if mode == 1
                app.audioWave = s_syn_s / max(s_syn_s);
            else
                app.audioWave = s_syn_f / max(s_syn_f);
            end
            app.changeWave();
            app.drawMap();
        end


            
        function PT=findpitch(~,s,WL)
            [B,A] = butter(5,700/4000);
            s = filter(B,A,s);
            R = zeros(WL-1,1);
            for k = 1:WL-1
                R(k) = s(WL:end)' * s(WL-k:end-k);
            end
            [~,T] = max(R);
            PT = T;
        end

        
        function invervse(app)
            app.audioWave = flip(app.audioWave);
            app.changeWave();
            app.drawMap();
        end
    
        
        function changePitch(app, pitch)
            app.speechproc(app.audioWave,2,1,pitch);
            app.speechproc(app.audioWave,1,1/pitch,1);
        end
        
        function kidVoice(app)
            sound(app.audioWave, 2*app.FS);
        end
        
        function restructSpeech(app,speech)
            %定义常数
            FL = app.FS / 100;      %帧长
            WL = 3 * FL;            %窗长
            P = 800;                %预测系数个数
            s = speech / max(speech);             %赋值
            L = length(s);
            FN = floor(L/FL) - 2;   %计算帧数
            last_syn = 0;           %记录
            %预测和重建滤波器
            exc = zeros(L,1);       %预测误差e（激励信号）
            zi_pre = zeros(P,1);    %预测滤波器的状态
            s_rec = zeros(L,1);     %重建语音
            zi_rec = zeros(P,1);    %重建语音
            %合成滤波器
            zi_syn = zeros(P,1);    % 合成滤波器的状态
            exc_syn = zeros(L,1);   %合成的激励信号
            s_syn = zeros(L,1);     %合成语音
            
            hw = hamming(WL);       %汉明窗
            for n = 3:FN
                %计算预测系数
                s_w = s(n*FL-WL+1:n*FL) .* hw;   %汉明窗加权后的语音
                %A是预测系数，E被用于计算合成激励的能量
                [A, E] = lpc(s_w,P);         %用LPC计算P个预测系数
                s_f = s((n-1)*FL+1:n*FL);   %处理这帧语言
                
                %计算激励
                %使用filter函数s_f计算激励，注意保持滤波器状态

                [exc((n-1)*FL+1:n*FL),zi_pre] = filter([0 -A(2:end)],1,s_f,zi_pre);  %上面求出的激励就是这帧的激励
                
                %重建语言
                %使用filter函数和exc重建语言，注意保持滤波器状态
                [s_rec((n-1)*FL+1:n*FL),zi_rec] = filter(1,A,exc((n-1)*FL+1:n*FL),zi_rec); %上面重建的语言就是这帧的重建语言
                
                s_Pitch = exc(n*FL-WL+1:n*FL);
                PT = app.findpitch(s_Pitch,WL);    %计算基音周期PT
                G = sqrt(E*PT);                 %计算合成激励的能量G

                
                index = (1:n*FL-last_syn);
                exc_syn1 = zeros(length(index),1);
                exc_syn1(mod(index,PT)==0) = G;         %用周期信号作为激励
                exc_syn((n-1)*FL+1:n*FL) = exc_syn1((n-1)*FL-last_syn+1:n*FL-last_syn);%计算得到的合成激励
                [s_syn((n-1)*FL+1:n*FL),zi_syn] = filter(1,A,exc_syn((n-1)*FL+1:n*FL),zi_syn); %得到合成语音
                last_syn = last_syn+PT*floor((n*FL-last_syn)/PT); %计算本次算到的帧数，通过能容纳多少个基音周期

            end
            app.audioWave = s_syn;
            app.changeWave();
            app.drawMap();
        end
    
    
    
    
    
    
    
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Value changed function: unOrRecordBotton
        function unOrRecordBottonValueChanged(app, event)
            value = app.unOrRecordBotton.Value;
            if value == 1
                app.unOrRecordBotton.Text = "按下结束";
                set(app.FrequencySlider, 'Value', 0);
                set(app.SpeedSlider, 'Value', 0);
                app.startRecord();
            end
            if value == 0
                app.unOrRecordBotton.Text = "按下录音";
                app.store();
                app.drawMap();
            end
        end

        % Button pushed function: Display
        function DisplayButtonPushed(app, event)
            sound(app.audioWave,app.FS);
        end

        % Button pushed function: HenceSpeech
        function HenceSpeechButtonPushed(app, event)
            app.enhanceSpeech();
            app.HenceSpeech.Text = "增强成功";
            pause(1);           %显示一会
            app.HenceSpeech.Text = "语音增强";
        end

        % Value changed function: SpeedSlider
        function SpeedSliderValueChanged(app, event)
            value = -app.SpeedSlider.Value;
            speed = 2 ^ value;
            app.changeSpeed(speed);
            app.changeWave();
            app.drawMap();
        end

        % Button pushed function: inverse
        function inverseButtonPushed(app, event)
            app.invervse();
            app.inverse.Text = "倒放成功";
            pause(1);
            app.inverse.Text = "倒放";
        end

        % Value changed function: FrequencySlider
        function FrequencySliderValueChanged(app, event)
            value = -app.FrequencySlider.Value;
            pitch = 2 ^ value;
            app.changePitch(pitch);
            app.changeWave();
            app.omega = app.omega * pitch;
            app.drawMap();
        end

        % Button pushed function: kidsVoice
        function kidsVoiceButtonPushed(app, event)
            app.kidsVoice.Text = "小孩版";
            app.kidVoice();
            pause(1);
            app.kidsVoice.Text = "变速变调";
        end

        % Button pushed function: playOrigin
        function playOriginButtonPushed(app, event)
            app.playOrigin.Text = "正在播放";
            sound(app.audioWaveBackUp, app.FS);
            app.playOrigin.Text = "原始音频";
        end

        % Button pushed function: undoAll
        function undoAllButtonPushed(app, event)
            app.undoAll.Text = "正在还原";
            app.audioWave = app.audioWaveBackUp;
            set(app.SpeedSlider, 'Value', 0);
            set(app.FrequencySlider, 'Value', 0);
            app.changeWave();
            app.drawMap();
            pause(0.5);
            app.undoAll.Text = "还原音频";
        end

        % Button pushed function: restrcutEff
        function restrcutEffButtonPushed(app, event)
            app.restructSpeech(app.audioWave);
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {480, 480};
                app.GridLayout.ColumnWidth = {'1x'};
                app.Botton.Layout.Row = 2;
                app.Botton.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {408, '1x'};
                app.Botton.Layout.Row = 1;
                app.Botton.Layout.Column = 2;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {408, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create Graphic
            app.Graphic = uipanel(app.GridLayout);
            app.Graphic.Layout.Row = 1;
            app.Graphic.Layout.Column = 1;

            % Create TimeMap
            app.TimeMap = uiaxes(app.Graphic);
            title(app.TimeMap, '时域信号')
            xlabel(app.TimeMap, 't/s')
            zlabel(app.TimeMap, 'Z')
            app.TimeMap.YGrid = 'on';
            app.TimeMap.Position = [45 244 300 185];

            % Create FqcMap
            app.FqcMap = uiaxes(app.Graphic);
            title(app.FqcMap, '频域信号')
            xlabel(app.FqcMap, 'ω/Hz')
            zlabel(app.FqcMap, 'Z')
            app.FqcMap.YGrid = 'on';
            app.FqcMap.Position = [45 29 300 185];

            % Create Botton
            app.Botton = uipanel(app.GridLayout);
            app.Botton.Layout.Row = 1;
            app.Botton.Layout.Column = 2;

            % Create title
            app.title = uilabel(app.Botton);
            app.title.FontSize = 24;
            app.title.Position = [17 386 198 31];
            app.title.Text = '语音信号处理系统';

            % Create unOrRecordBotton
            app.unOrRecordBotton = uibutton(app.Botton, 'state');
            app.unOrRecordBotton.ValueChangedFcn = createCallbackFcn(app, @unOrRecordBottonValueChanged, true);
            app.unOrRecordBotton.Text = '按下录音';
            app.unOrRecordBotton.Position = [13 326 100 23];

            % Create Label
            app.Label = uilabel(app.Botton);
            app.Label.HorizontalAlignment = 'right';
            app.Label.Position = [13 191 29 22];
            app.Label.Text = '速度';

            % Create SpeedSlider
            app.SpeedSlider = uislider(app.Botton);
            app.SpeedSlider.Limits = [-2 2];
            app.SpeedSlider.MajorTicks = [-2 -1 0 1 2];
            app.SpeedSlider.ValueChangedFcn = createCallbackFcn(app, @SpeedSliderValueChanged, true);
            app.SpeedSlider.Position = [63 201 150 3];

            % Create Slider_2Label
            app.Slider_2Label = uilabel(app.Botton);
            app.Slider_2Label.HorizontalAlignment = 'right';
            app.Slider_2Label.Position = [13 130 29 22];
            app.Slider_2Label.Text = '音调';

            % Create FrequencySlider
            app.FrequencySlider = uislider(app.Botton);
            app.FrequencySlider.Limits = [-2 2];
            app.FrequencySlider.MajorTicks = [-2 -1 0 1 2];
            app.FrequencySlider.ValueChangedFcn = createCallbackFcn(app, @FrequencySliderValueChanged, true);
            app.FrequencySlider.Position = [63 139 150 3];

            % Create Display
            app.Display = uibutton(app.Botton, 'push');
            app.Display.ButtonPushedFcn = createCallbackFcn(app, @DisplayButtonPushed, true);
            app.Display.Position = [66 49 100 23];
            app.Display.Text = '播放';

            % Create HenceSpeech
            app.HenceSpeech = uibutton(app.Botton, 'push');
            app.HenceSpeech.ButtonPushedFcn = createCallbackFcn(app, @HenceSpeechButtonPushed, true);
            app.HenceSpeech.Position = [119 326 100 23];
            app.HenceSpeech.Text = '语音增强';

            % Create inverse
            app.inverse = uibutton(app.Botton, 'push');
            app.inverse.ButtonPushedFcn = createCallbackFcn(app, @inverseButtonPushed, true);
            app.inverse.Position = [13 285 100 23];
            app.inverse.Text = '倒放';

            % Create kidsVoice
            app.kidsVoice = uibutton(app.Botton, 'push');
            app.kidsVoice.ButtonPushedFcn = createCallbackFcn(app, @kidsVoiceButtonPushed, true);
            app.kidsVoice.Position = [119 285 100 23];
            app.kidsVoice.Text = '变调变速';

            % Create playOrigin
            app.playOrigin = uibutton(app.Botton, 'push');
            app.playOrigin.ButtonPushedFcn = createCallbackFcn(app, @playOriginButtonPushed, true);
            app.playOrigin.Position = [13 243 100 23];
            app.playOrigin.Text = '原始音频';

            % Create undoAll
            app.undoAll = uibutton(app.Botton, 'push');
            app.undoAll.ButtonPushedFcn = createCallbackFcn(app, @undoAllButtonPushed, true);
            app.undoAll.Position = [119 242 100 23];
            app.undoAll.Text = '还原音频';

            % Create restrcutEff
            app.restrcutEff = uibutton(app.Botton, 'push');
            app.restrcutEff.ButtonPushedFcn = createCallbackFcn(app, @restrcutEffButtonPushed, true);
            app.restrcutEff.Visible = 'off';
            app.restrcutEff.Position = [63 428 100 23];
            app.restrcutEff.Text = '重建效果';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = speechProgressSystemCode

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end