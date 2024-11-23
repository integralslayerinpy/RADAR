function [dataOk_1, dataOk_2, dataOk_3,  dataOk_4, dataOk_5, frameNumber, detObj, rp, noiseProfile_dB, theta, range, QQ, rangeDoppler] = readAndParseData16XX(DATA_sphandle, ConfigParameters)
    OBJ_STRUCT_SIZE_BYTES = 16;
    BYTE_VEC_ACC_MAX_SIZE = 2^16;
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    MMWDEMO_UART_MSG_NOISE_PROFILE   = 3;
    MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP = 4;
    MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP = 5;
    MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO  = 7;
    maxBufferSize = 2^16;
    NUM_ANGLE_BINS = 64;
    
    detObj = [];
    rp = [];
    noiseProfile_dB = [];
    theta=[];
    range=[];
    QQ=[];
    rangeDoppler=[];


    frameNumber = 0;
      
    persistent byteBuffer
    if isempty(byteBuffer)
        byteBuffer = zeros(maxBufferSize,1);
    end
    
    persistent byteBufferLength
    if isempty(byteBufferLength)
        byteBufferLength = 0;
    end
    
    persistent magiNotOkCounter
    if isempty(magiNotOkCounter)
        magiNotOkCounter = 0;
    end
    
    magicOk = 0;
    dataOk_1 = 0;
    dataOk_2 = 0;
    dataOk_3 = 0;
    dataOk_4 = 0;
    dataOk_5 = 0;
    
    bytesToRead = get(DATA_sphandle,'BytesAvailable');
    if (bytesToRead ~= 0)
        % Read the Data Serial Port
        [bytevec, byteCount] = fread(DATA_sphandle, bytesToRead, 'uint8');
        
         % Check if the buffer is not full, and then add the data to the buffer:
        if(byteBufferLength + byteCount < maxBufferSize)
            byteBuffer(byteBufferLength+1:byteBufferLength + byteCount) = bytevec(1:byteCount);
            byteBufferLength = byteBufferLength + byteCount;
        end
        
    end
 
    % Check that the buffer is not empty:
    if byteBufferLength > 16
        byteBufferStr = char(byteBuffer);
        
        % Search for the magic number inside the buffer and check that at least one magic number has been found:
        startIdx = strfind(byteBufferStr', char([2 1 4 3 6 5 8 7]));
        if ~isempty(startIdx)
            
            % Check the position of the first magic number and put it at
            % the beginning of the buffer
            if length(startIdx) >= 2
                if startIdx(end-1) > 1
                    byteBuffer(1:byteBufferLength-(startIdx(1)-1)) = byteBuffer(startIdx(1):byteBufferLength);
                    byteBufferLength = byteBufferLength - (startIdx(1)-1);
                end
            else
                if startIdx(1) > 1
                    byteBuffer(1:byteBufferLength-(startIdx(1)-1)) = byteBuffer(startIdx(1):byteBufferLength);
                    byteBufferLength = byteBufferLength - (startIdx(1)-1);
                end
            end
            if byteBufferLength < 0
                byteBufferLength = 0;
            end
            
            totalPacketLen = sum(byteBuffer(8+4+[1:4]) .* [1 256 65536 16777216]');
            if ((byteBufferLength >= totalPacketLen) && (byteBufferLength ~= 0)) 
                magicOk = 1;
            else
                magicOk = 0;
            end
        end
    end
    
    if (magicOk == 1)
        % saveFilePath = 'byteBufferData.mat'; % Kayıt edilecek dosyanın adı
        % 
        % % Kaydetme işlemi
        % save(saveFilePath, 'byteBuffer', '-v7.3'); % byteBuffer ve boyutunu kaydeder
        % fprintf('Radar verileri "%s" dosyasına başarıyla kaydedildi.\n', saveFilePath);

       

        %%%%% HEADER
        word = [1 256 65536 16777216]';
        idx = 0;
        magicNumber = byteBuffer(idx + 1:8);
        idx = idx + 8;
        Header.version = dec2hex(sum(byteBuffer(idx+[1:4]) .* word));
        idx = idx + 4;
        Header.totalPacketLen = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.platform = dec2hex(sum(byteBuffer(idx+[1:4]) .* word));
        idx = idx + 4;
        Header.frameNumber = sum(byteBuffer(idx+[1:4]) .* word);
        frameNumber = Header.frameNumber;
        idx = idx + 4;
        Header.timeCpuCycles = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.numDetectedObj = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.numTLVs = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        Header.subFrameNumber = sum(byteBuffer(idx+[1:4]) .* word);
        idx = idx + 4;
        
        
        %%%%% TLV
        
        % Analyze each of TLV messages:
        for tlvIdx = 1:Header.numTLVs
            word = [1 256 65536 16777216]';
            % First, analyze the TLV header (TLV type and length):
            tlv.type = sum(byteBuffer(idx+(1:4)) .* word);
            idx = idx + 4;
            tlv.length = sum(byteBuffer(idx+(1:4)) .* word);
            idx = idx + 4;
            
            % Check that the TLV message is of the right type (Detected objects):
            switch tlv.type
                case MMWDEMO_UART_MSG_DETECTED_POINTS
                    % detObj =[];
                    % detObj.numObj = 0; 
                    % 
                    % if tlv.length > 0
                    %     %Get detected object descriptor
                    %     word = [1 256]';
                    %     % detObj.numObj = sum(byteBuffer(idx+(1:2)) .* word);
                    %     detObj.numObj = (tlv.length) / 16
                    % 
                    %     % idx = idx + 2;
                    %     xyzQFormat = 2^sum(byteBuffer(idx+(1:4)) .* word);
                    % 
                    %     idx = idx + 4;
                    % 
                    %     %Get detected array of detected objects
                    %     bytes = byteBuffer(idx+(1:detObj.numObj*OBJ_STRUCT_SIZE_BYTES));
                    %     idx = idx + detObj.numObj*OBJ_STRUCT_SIZE_BYTES;
                    % 
                    %     bytes = reshape(bytes, OBJ_STRUCT_SIZE_BYTES, detObj.numObj);
                    %     detObj.rangeIdx = (bytes(1,:)+bytes(2,:)*256);
                    %     %getObj.doppler 
                    %     detObj.range = detObj.rangeIdx * ConfigParameters.rangeIdxToMeters; %convert range index to range (in meters)
                    %     detObj.dopplerIdx = (bytes(3,:)+bytes(4,:)*256); %convert doppler index to doppler (meters/sec)
                    %     % detObj.peakVal
                    %     %circshift the doppler fft bins
                    %     % detObj.dopplerIdx(detObj.dopplerIdx > ConfigParameters.numDopplerBins/2-1) = ...
                    %     % detObj.dopplerIdx(detObj.dopplerIdx > ConfigParameters.numDopplerBins/2-1) - 65535;
                    %     % detObj.doppler = detObj.dopplerIdx * ConfigParameters.dopplerResolutionMps; %convert doppler index to doppler (meters/sec)
                    %     % detObj.peakVal = bytes(5,:)+bytes(6,:)*256; %peak value (16-bit=> so constructed from 2 bytes)
                    %     % 
                    %     % 
                    %     % detObj.x = bytes(7,:)+bytes(8,:)*256;
                    %     % detObj.y = bytes(9,:)+bytes(10,:)*256;
                    %     % detObj.z = bytes(11,:)+bytes(12,:)*256;
                    % 
                    % 
                    %     detObj.x = bytes(5,:)+bytes(6,:)*256;
                    %     detObj.y = bytes(7,:)+bytes(8,:)*256;
                    %     detObj.z = bytes(9,:)+bytes(10,:)*256;
                    % 
                    %     detObj.x( detObj.x > 32767) =  detObj.x( detObj.x > 32767) - 65536;
                    %     detObj.y( detObj.y > 32767) =  detObj.y( detObj.y > 32767) - 65536;
                    %     detObj.z( detObj.z > 32767) =  detObj.z( detObj.z > 32767) - 65536;
                    % 
                    %     detObj.rangeIdx(detObj.rangeIdx > 32767) = detObj.rangeIdx(detObj.rangeIdx > 32767) -65536;
                    % 
                    % 
                    %     detObj.x =  detObj.x / xyzQFormat;
                    %     detObj.y =  detObj.y / xyzQFormat;
                    %     detObj.z =  detObj.z / xyzQFormat;
                    % 
                    %     detObj.rangeIdx = detObj.rangeIdx / xyzQFormat;
                    % 
                    %     dataOk = 1;
                    % end
                    detObj =[];
                    
                    if tlv.length > 0                       
                        % Extract the raw data for all the detected points
                        bytes = byteBuffer(idx+(1:Header.numDetectedObj*OBJ_STRUCT_SIZE_BYTES));
                        idx = idx + Header.numDetectedObj*OBJ_STRUCT_SIZE_BYTES;
                        
                        % Reshape the array to have the data for each point
                        % (X,Y,Z,doppler) in each column
                        bytes = reshape(bytes, OBJ_STRUCT_SIZE_BYTES, Header.numDetectedObj);
                        
                        % Convert the byte matrix to float data
                        floatData = reshape(typecast(reshape(uint8(bytes), 1, []), 'single'),4,Header.numDetectedObj);
                      
                        detObj.numObj = Header.numDetectedObj;
                        detObj.x = floatData(1,:);
                        detObj.y = floatData(2,:);
                        detObj.z = floatData(3,:);
                        detObj.doppler = floatData(4,:);
                        dataOk_1 = 1;
                        
                    end
                    
                case MMWDEMO_UART_MSG_RANGE_PROFILE
                    rp = [];
                    rp = byteBuffer(idx+(1:tlv.length));
                    idx = idx + tlv.length;
                    rp = rp(1:2:end)+rp(2:2:end)*256;
                    rp = 20 * log10(rp);
                    dataOk_2 = 1;


                case MMWDEMO_UART_MSG_NOISE_PROFILE

                    numNoiseBins = ConfigParameters.numRangeBins; % Noise profilindeki veri noktalarının sayısı
                    len = numNoiseBins * 2; % Her değer 2 bayt (16 bit)
                    
                    % Gürültü profilini byteBuffer'dan oku
                    noiseProfile = byteBuffer(idx+(1:len));
                    idx = idx + len;
                    
                    % Verileri dönüştür (16 bitlik integer olarak oku)
                    noiseProfile = noiseProfile(1:2:end) + noiseProfile(2:2:end) * 256;
                    
                    % Gürültü profilini grafik için normalize et
                    noiseProfile_dB = 20 * log10(noiseProfile); % Logaritmik skala (dB) kullan
                    dataOk_3 = 1;

                case MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO
                    
                    if tlv.length > 0   
                        bytes = byteBuffer(idx+(1:Header.numDetectedObj*4));
                        idx = idx + Header.numDetectedObj*4;
                        
                        % Reshape the array to have the data for each point
                        % (snr,noise) in each column
                        bytes = reshape(bytes, 4, Header.numDetectedObj);
                        
                        % Convert the byte matrix to float data
                        floatData = reshape(typecast(reshape(uint8(bytes), 1, []), 'int16'),2,Header.numDetectedObj);
                        detObj.snr = floatData(1,:);
                        detObj.noise = floatData(2,:);
                        
                        dataOk_3 = 1;
                    
                    end
                
                
                case MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP
                    theta=[];
                    range=[];
                    QQ=[];
                    numBytes = 2 * 4 * ConfigParameters.numRangeBins * 4;               
                    q = byteBuffer(idx+(1:numBytes));
                    idx = idx + numBytes;
                    q = q(1:2:end)+q(2:2:end)*2^8;
                    q(q>32767) = q(q>32767) - 65536;
                    q = q(1:2:end)+1j*q(2:2:end);
                    q = reshape(q, 2*4, ConfigParameters.numRangeBins);
                    Q = fft(q, NUM_ANGLE_BINS);  % column based NUM_ANGLE_BINS-point fft, padded with zeros
                    QQ=fftshift(abs(Q),1);
                    QQ=QQ.';
                    
                    QQ=abs(QQ(:,2:end));
                    QQ=fliplr(QQ);
                    theta = asind((-NUM_ANGLE_BINS/2+1 : NUM_ANGLE_BINS/2-1)'*(2/NUM_ANGLE_BINS));
                    range = (0:ConfigParameters.numRangeBins-1) * ConfigParameters.rangeIdxToMeters;
                    dataOk_4 = 1;



%                     imagesc(theta, range, QQ, [0,max(QQ(:))]);
%                     set(gca,'YDir','normal')
%                     xlabel('Azimuth Angle [degree]');
%                     ylabel('Range [m]');

                case MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP
                        len = ConfigParameters.numDopplerBins * ConfigParameters.numRangeBins * 2;
                        rangeDoppler = byteBuffer(idx+(1:len));
                        idx = idx + len;
                        rangeDoppler = rangeDoppler(1:2:end) + rangeDoppler(2:2:end)*256;
                        rangeDoppler = reshape(rangeDoppler, ConfigParameters.numDopplerBins, []);
                        rangeDoppler = fftshift(rangeDoppler,1);
                        dataOk_5 = 1;

            end
            
        end
        %Remove processed data
        if idx > 0
            shiftSize = Header.totalPacketLen;
            byteBuffer(1: byteBufferLength-shiftSize) = byteBuffer(shiftSize+1:byteBufferLength);
            byteBufferLength = byteBufferLength - shiftSize;
            if byteBufferLength < 0
                %             fprintf('Error: bytevec_cp_len < bytevecAccLen, %d %d \n', bytevec_cp_len, bytevecAccLen)
                byteBufferLength = 0;
            end
        end
%         if byteBufferLength > (byteBufferLength * 7/8)
%             byteBufferLength = 0;
%         end
        
    else
        magiNotOkCounter = magiNotOkCounter + 1;
    end
    
                   
                                    
                    
