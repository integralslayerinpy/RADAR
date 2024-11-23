% Automated Doors Visualizer
% For user with mmWave_SDK 3.3.x

clear, clc, close all
delete(instrfind)

%% Input for COM ports and angle
num_COM_UART = input('User/UART Serial Port: COM');
num_COM_DATA = input('Data Serial Port: COM');

%% SET, Parse and load CFG FILE
LOAD_CFG = 1;
configFile = "xwr16xx_processed_stream_2024_10_20T17_39_06_173.cfg";
[DATA_sphandle, UART_sphandle, ConfigParameters] = radarSetup16XX(configFile, num_COM_UART, num_COM_DATA);
platformType = hex2dec('1642');


%% Initialize the figure


% Polar plot figure setup açılı grafik çizimi
% figure(1);
% set(gcf, 'Position', get(0, 'Screensize'));
% H1 = uicontrol('Style', 'PushButton', ...
%               'String', 'Stop', ...
%               'Callback', 'stopVal = 1', 'Position', [100 600 100 30]);
% h(1) = polarscatter([], [], 'filled');
% thetalim([30, 150]);
% rlim([0, 1]);
% title('Raw data');


% Figure 1: Polar Plot 3D
% figure(1);
% set(gcf, 'Position', get(0, 'Screensize'));
% H = uicontrol('Style', 'PushButton', ...
%               'String', 'Stop', ...
%               'Callback', 'stopVal = 1', ...
%               'Position', [100 600 100 30]);
% ax1 = axes; % Ekseni tanımlayın
% h = scatter3(ax1, [], [], [], 'filled');
% axis(ax1, [-0.5, 0.5, 0, 1.5]);
% xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)')
% grid(ax1, 'minor');

% Figure 1: Polar Plot 2D
figure(1);
set(gcf, 'Position', get(0, 'Screensize'));
H = uicontrol('Style', 'PushButton', ...
              'String', 'Stop', ...
              'Callback', 'stopVal = 1', ...
              'Position', [100 600 100 30]);
ax1 = axes; % Ekseni tanımlayın
h_2D = scatter(ax1, [], [], 'filled');
axis(ax1, [-0.5, 0.5, 0, 2]);
xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)');
grid(ax1, 'minor');

% Figure 2: Range Profile Plot
figure(2);
set(gcf, 'Position', get(0, 'Screensize'));
H2 = uicontrol('Style', 'PushButton', ...
               'String', 'Stop', ...
               'Callback', 'stopVal = 1', ...
               'Position', [100, 600, 100, 30]);
ax2 = axes; % Yeni ekseni tanımlayın
rangePlot = plot(ax2, nan, nan); % Range profile grafiğini yeni eksene çizin
xlabel(ax2, 'Range (m)');
ylabel(ax2, 'Amplitude');
title(ax2, 'Range Profile');
grid(ax2, 'on');

% Figure 3: Noise Profile Plot
figure(3);
set(gcf, 'Position', get(0, 'Screensize')); % Tam ekran yap
H6 = uicontrol('Style', 'PushButton', ...
                  'String', 'Stop', ...
                  'Callback', 'stopVal = 1', ...
                  'Position', [100, 600, 100, 30]);
ax3 = axes;
NoisePlot = plot(ax3, nan, nan);
% Eksen ayarları
xlabel(ax3, 'Range (m)');
ylabel(ax3, 'Noise Power (dB)');
title(ax3, 'Noise Profile');
grid(ax3, 'on');
axis(ax3, 'tight'); % Grafiği eksenlerle hizalar


% Figure 4: Azimuth Static Heat Map
figure(4);
set(gcf, 'Position', get(0, 'Screensize'));
H3 = uicontrol('Style', 'PushButton', ...
               'String', 'Stop', ...
               'Callback', 'stopVal = 1', ...
               'Position', [100, 600, 100, 30]);
ax4 = axes; % Yeni ekseni tanımlayın
colorbar(ax4); % Renk skalası ekleyin
xlabel(ax4, 'Azimuth Angle [degree]');
ylabel(ax4, 'Range [m]');
title(ax4, 'Azimuth Static Heat Map');
hHeatMap = imagesc(ax4, [], [], []); % Heatmap grafiğini yeni eksene çizin
colormap(jet); % Renk haritası
colorbar;
axis(ax4, 'tight');
set(ax4, 'YDir', 'normal'); % Y eksenini ters çevir


% Figure 5: Doppler Heat Map
figure(5);
    set(gcf, 'Position', get(0, 'Screensize')); % Tam ekran yap
    H4 = uicontrol('Style', 'PushButton', ...
                  'String', 'Stop', ...
                  'Callback', 'stopVal = 1', ...
                  'Position', [100, 600, 100, 30]);

    dopplerAxis = (-ConfigParameters.numDopplerBins/2 : ConfigParameters.numDopplerBins/2 - 1) * ConfigParameters.dopplerResolutionMps;
    rangeAxis = (0 : ConfigParameters.numRangeBins - 1) * ConfigParameters.rangeResolutionMeters;
    ax5 = axes; % Yeni eksen oluştur
    hRangeDoppler = imagesc(ax5,rangeAxis, dopplerAxis, abs(nan)); % Başlangıçta boş bir grafik oluştur
    colorbar; % Renk skalasını ekleyin
    xlabel(ax5, 'Range (m)' );
    ylabel(ax5,'Doppler Velocity (m/s)' );
    title(ax5, 'Range-Doppler Heat Map');
    set(ax5, 'YDir', 'normal'); % Y eksenini ters çevir



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%                   MAIN   LOOP              %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%
myInd = 1;
frame = [];
lastreadTime = 0;
stopVal = 0;
dataOk = 0;

tic

while 1
    dataOk = 0;
    timeNow = toc;
    elapsedTime = timeNow - lastreadTime;

    % Read the data from the radar:
    [dataOk_1, dataOk_2, dataOk_3, dataOk_4, dataOk_5, frameNumber, detObj, rp, noiseProfile_dB, theta, range, QQ, rangeDoppler] = readAndParseData16XX(DATA_sphandle, ConfigParameters);
    lastreadTime = timeNow;


    if dataOk_1 == 1
        % Update polar plot
        h.XData = -detObj.x;
        h.YData = detObj.y;
        h.ZData = detObj.z;
        drawnow limitrate;

        h_2D.XData = -detObj.x;
        h_2D.YData =  detObj.y;
        drawnow limitrate;
    end

    if dataOk_2 == 1
        % Update range profile plot 
        rangeAxis = (0:length(rp) - 1) * ConfigParameters.rangeResolutionMeters;
        set(rangePlot, 'XData', rangeAxis, 'YData', rp); % Update range profile data
        drawnow;
    end

    if dataOk_3 == 1
        % Update Noise profile plot 
        numNoiseBins = ConfigParameters.numRangeBins;
        numNoiseBins = (0:numNoiseBins-1) * ConfigParameters.rangeResolutionMeters;
        set(NoisePlot, 'XData', numNoiseBins, 'YData', noiseProfile_dB); % Update range profile data
        drawnow;
    end

    if dataOk_4 == 1
        set(hHeatMap, 'XData', theta, 'YData', range, 'CData', QQ);
        set(ax4, 'XLim', [min(theta), max(theta)], 'YLim', [min(range), max(range)]/2);
        drawnow limitrate;
    end



    if dataOk_5 == 1
        set(hRangeDoppler, 'CData', abs(rangeDoppler)); % Veri ile grafiği güncelle
        set(ax5, 'XLim', [min(rangeAxis), max(rangeAxis)], 'YLim', [min(dopplerAxis), max(dopplerAxis)]);
        drawnow limitrate;
    end



    pause(0.03);
    if stopVal == 1
        stopVal = 0;
        break;
    end
end

fprintf(UART_sphandle, 'sensorStop');
delete(instrfind);
clear cam
close all
