classdef servo < handle
    %SERVO Creates a servo object
    %   sv = servo(ID) Initializes a servo object with default 
    %   parameters that is addressed using ID
    properties(Access = public)
        %% Serial properties
        comPort = 'com4';       %Com port to use
        ser                     %The serial object
        
        %% Properties related to physical changes on boards
        dacRef                  %DAC reference voltage corresponding to maximum DAC code
        
        %% Global settings
        spiPeriod               %SPI period to use when communicating with either DAC or ADC
        transmitType            %Type of data to save in memory and later transmit
        enableTrig              %Enable external trigger?
        offTime                 %Time at which the controller turns output off
        adcPeriod               %Sampling time of ADC - READ ONLY
        
        %% DAC settings
        minValueDAC             %Minimum value that DAC should output
        maxValueDAC             %Maximum value that DAC should output
        syncDelay               %Delay between last bit of data and when SYNC should be raised
        dacMode                 %Mode of the DAC - unipolar or bipolar?
        dacOffValue             %Value to write when offTime occurs
        useExtSwitch            %Enable the use of an external signal to control switch on/off logic
        
        %% General ramp settings
        sampleSource            %Source to use for samples - 'int' or 'mem'
        updateTime              %Delay between updating samples
        numSamples              %Number of samples to use
        
        %% Linear ramp settings
        rampValues              %Array of starting values for linear ramps
        rampBndyTimes           %Array of times at which the current linear ramp ends
        rampRates               %Array of increments to add at each updateTime
        rampCodes               %Array of 'codes' to use with ramp value
        
        %% PID settings
        polarity                %Polarity of the PID controller - positive or negative
        Kp                      %Proportional gain value - fixed gains only
        Ki                      %Integer gain value - fixed gains only
        Kd                      %Derivative gain value - fixed gains only
        divisorPID              %Overall divisor to use - fixed gains only
        useFixed                %Use fixed gains (1) or variable gains (0)
        
        %% Memory settings
        numMemSamples           %Number of samples to read from memory
        useSampleLimits         %Use limits on retrieved samples (1) or not (0)
        minSample               %Minimum sample value to retrieve
        maxSample               %Maximum sample value to retrieve
        sampleStep              %Increment when retrieving samples
        requestedSamples        %Number of samples that should have been received - READ ONLY
        
        %% Other properties
        auxCmd                  %Auxilliary command used for various purposes
        sampleData              %Sample data written to memory
        loopData1               %Kp and Ki data written to memory for variable gains
        loopData2               %Kd and n data written to memory for variable gains
        v                       %Voltage retrieved from memory
        t                       %Time value inferred from sample rate and number of voltage samples
        retryOnWarning          %If true, retries data retreival on warnings that sample lengths are wrong
        useCompression          %If true, applies static methods compressSamples and decompressSamples to data written to and read from memory
    end
    
    properties(Hidden = true)
        ID                      %Servo ID within FPGa, ranges from 0 to 4 depending on FPGA configuration
    end
    
    properties(Constant, Hidden=true)
        %% FPGA related constants
        CLK = 50e6;             %FPGA clock frequency [Hz]
        
        %% Serial constants
        BAUD_RATE = 1e6;        %Serial baud rate [Hz]
        BUFFER_SIZE = 2^24;     %Serial buffer size
        
        %% DAC constants
        DAC_REF = 10;               %Default DAC reference [V]
        DAC_NUM_BITS = 20;          %Number of bits for DAC data
        DAC_MODE_UNI = 'unipolar';  %Indicates DAC is in unipolar mode
        DAC_MODE_BI = 'bipolar';    %Indicates DAC is in bipolar mode
        
        %% ADC constants
        ADC_REF = 2.5;          %ADC reference voltage [V]
        ADC_NUM_BITS = 24;      %Number of bits for ADC data
        
        %% Sample generation constants
        SOURCE_INT = 'int';     %Use internal linear ramp generators for samples
        SOURCE_MEM = 'mem';     %Use data in memory for samples
        MEM_CODE_SHIFT = 24;    %Amount to shift sample 'codes' when written to memory
        MAX_NUM_RAMPS = 8;      %Maximum number of linear ramps
        
        %% PID constants
        PID_NEGATIVE = 'negative';  %PID polarity is negative
        PID_POSITIVE = 'positive';  %PID poalrity is positive
        
        %% Transmission constants
        TRANSMIT_ADC = 'adc';       %Save/transmit ADC data
        TRANSMIT_DAC = 'dac';       %Save/transmit DAC data
        TRANSMIT_CNTRL = 'cntrl';   %Save/transmit control signal data
        TRANSMIT_ERR = 'err';       %Save/transmit error signal
        
    end
    
    methods
        %% General property functions
        function sv=servo(ID)
            %SERVO Creates a servo object
            %   sv = servo(ID) Initializes a servo object with default 
            %   parameters that is addressed using ID
            
            sv.defineProperties;
            sv.setID(ID);
            
            sv.dacMode.set(servo.DAC_MODE_BI);
            sv.dacRef = servo.DAC_REF;
            sv.retryOnWarning = false;
            sv.useCompression = false;
        end
        
        function delete(sv)
            %DELETE Deletes the servo object
            %   DELETE calls the servo.close() function to close the serial
            %   connection
            sv.close;
        end
        
        function defineProperties(sv)
            %DEFINEPROPERTIES Defines the various servo properties
            %   Initializes the servo properties to their default values
            
            %% Global settings
            reg3 = '02';
            sv.spiPeriod =      servoCmd('uint','reg3',reg3,'reg1','00').setDevice(sv)...
                                .setFunctions('from',@(x)x/servo.CLK,'to',@(x)x*servo.CLK)...
                                .setLimits('lower',50e-9).set(100e-9);
            sv.transmitType =   servoCmd('uint','reg3',reg3,'reg1','01').setDevice(sv)...
                                .setFunctions('to',@(x) servo.getTransmitType(x,'to'),'from',@(x) servo.getTransmitType(x,'from'))...
                                .set('adc');
            sv.enableTrig =     servoCmd('uint','reg3',reg3,'reg1','02').setDevice(sv)...
                                .setFunctions('to',@(x) x~=0,'from',@(x) x)...
                                .set(1);
            sv.offTime =        servoCmd('uint','reg3',reg3,'reg1','03').setDevice(sv)...
                                .setFunctions('from',@(x)x/servo.CLK,'to',@(x)x*servo.CLK)...
                                .setLimits('lower',0,'upper',2^32/servo.CLK).set(1);
            sv.adcPeriod =      servoCmd('cmd','reg3',reg3,'reg1','05').setDevice(sv)...
                                .setFunctions('from',@(x)x/servo.CLK,'to',@(x)x*servo.CLK);
            %% DAC settings
            reg3 = '03';
            sv.dacMode =        servoCmd('uint','reg3',reg3,'reg1','03').setDevice(sv)...
                                .setFunctions('to',@(x) servo.getDacMode(x,'to'),'from',@(x) servo.getDacMode(x,'from'))...
                                .set(servo.DAC_MODE_UNI);
            sv.minValueDAC =    servoCmd('uint','reg3',reg3,'reg1','00').setDevice(sv)...
                                .setFunctions('to',@(x)servo.dacConvert(x,sv.dacMode.get,sv.dacRef,'code'),'from',@(x)servo.dacConvert(x,sv.dacMode.get,sv.dacRef,'volt'));
            sv.maxValueDAC =    servoCmd('uint','reg3',reg3,'reg1','01').setDevice(sv)...
                                .setFunctions('to',@(x)servo.dacConvert(x,sv.dacMode.get,sv.dacRef,'code'),'from',@(x)servo.dacConvert(x,sv.dacMode.get,sv.dacRef,'volt'));
            sv.syncDelay =      servoCmd('uint','reg3',reg3,'reg1','02').setDevice(sv)...
                                .setFunctions('from',@(x)x/servo.CLK,'to',@(x)x*servo.CLK)...
                                .setLimits('lower',0,'upper',255/servo.CLK).set(0);
            sv.dacOffValue =    servoCmd('uint','reg3',reg3,'reg1','04').setDevice(sv)...
                                .setFunctions('to',@(x)servo.dacConvert(x,sv.dacMode.get,sv.dacRef,'code'),'from',@(x)servo.dacConvert(x,sv.dacMode.get,sv.dacRef,'volt'))...
                                .set(0);
            sv.useExtSwitch =   servoCmd('uint','reg3',reg3,'reg1','05').setDevice(sv)...
                                .setFunctions('to',@(x) x~=0,'from',@(x) x)...
                                .set(1);
            
            %% General ramp parameters
            reg3 = '04';
            sv.sampleSource =   servoCmd('uint','reg3',reg3,'reg1','00').setDevice(sv)...
                                .setFunctions('to',@(x) servo.getSampleSource(x,'to'),'from',@(x) servo.getSampleSource(x,'from'))...
                                .set(servo.SOURCE_INT);
            sv.updateTime =     servoCmd('uint','reg3',reg3,'reg1','01').setDevice(sv)...
                                .setFunctions('from',@(x)x/servo.CLK,'to',@(x)x*servo.CLK)...
                                .setLimits('lower',5e-6).set(32e-6);
            sv.numSamples =     servoCmd('uint','reg3',reg3,'reg1','02').setDevice(sv)...
                                .setFunctions('to',@(x) x,'from',@(x) x)...
                                .setLimits('lower',0).set(100);
            
            %% Linear ramp parameters
            reg3 = '05';
            sv.rampValues =     servoCmd('array','reg3',reg3,'reg1','00').setDevice(sv);
            sv.rampBndyTimes =  servoCmd('array','reg3',reg3,'reg1','01').setDevice(sv);
            sv.rampRates =      servoCmd('array','reg3',reg3,'reg1','02').setDevice(sv);
            sv.rampCodes =      servoCmd('array','reg3',reg3,'reg1','03').setDevice(sv);
            
            %% PID parameters
            reg3 = '06';
            sv.polarity =       servoCmd('uint','reg3',reg3,'reg1','00').setDevice(sv)...
                                .setFunctions('to',@(x) servo.getPolarity(x,'to'),'from',@(x) servo.getPolarity(x,'from'))...
                                .set(servo.PID_NEGATIVE);
            sv.Kp =             servoCmd('uint','reg3',reg3,'reg1','01').setDevice(sv)...
                                .setLimits('lower',0,'upper',2^16-1).set(0);
            sv.Ki =             servoCmd('uint','reg3',reg3,'reg1','02').setDevice(sv)...
                                .setLimits('lower',0,'upper',2^16-1).set(0);
            sv.Kd =             servoCmd('uint','reg3',reg3,'reg1','03').setDevice(sv)...
                                .setLimits('lower',0,'upper',2^16-1).set(0);
            sv.divisorPID =     servoCmd('uint','reg3',reg3,'reg1','04').setDevice(sv)...
                                .setLimits('lower',0,'upper',2^16-1).set(0);
            sv.useFixed =       servoCmd('uint','reg3',reg3,'reg1','05').setDevice(sv)...
                                .setFunctions('to',@(x) x~=0,'from',@(x) x)...
                                .set(1);
            
            %% Memory parameters
            reg3 = '07';
            sv.numMemSamples =  servoCmd('uint','reg3',reg3,'reg1','00').setDevice(sv)...
                                .set(100);
            sv.useSampleLimits = servoCmd('uint','reg3',reg3,'reg1','01').setDevice(sv)...
                                .setFunctions('to',@(x) x~=0,'from',@(x) x)...
                                .set(0);
            sv.minSample =      servoCmd('uint','reg3',reg3,'reg1','02').setDevice(sv)...
                                .set(0);
            sv.maxSample =      servoCmd('uint','reg3',reg3,'reg1','03').setDevice(sv)...
                                .set(0);
            sv.sampleStep =     servoCmd('uint','reg3',reg3,'reg1','04').setDevice(sv)...
                                .set(1);
            sv.requestedSamples = servoCmd('uint','reg3',reg3,'reg1','05').setDevice(sv);
            
            %% Other properties
            sv.auxCmd =         servoCmd('cmd','reg3','00').setDevice(sv);
            sv.sampleData =     servoCmd('mem','reg3','10').setDevice(sv)...
                                .setBits([27,26],'00');
            sv.loopData1 =      servoCmd('mem','reg3','10').setDevice(sv)...
                                .setBits([27,26],'01');
            sv.loopData2 =      servoCmd('mem','reg3','10').setDevice(sv)...
                                .setBits([27,26],'10');
            
        end
        
        function setID(sv,ID)
            %SETID Sets the ID of this servo instance
            %   servo.setID(ID) tells the serial controller in the FPGA
            %   which servo controller to send information to
            if nargin < 2
                ID = 0;
            end
            sv.ID = ID;
            bits = [31,30];
            
            %% Global parameters
            sv.spiPeriod.setBits(bits,sv.ID);
            sv.transmitType.setBits(bits,sv.ID);
            sv.enableTrig.setBits(bits,sv.ID);
            sv.offTime.setBits(bits,sv.ID);
            sv.adcPeriod.setBits(bits,sv.ID);
            
            %% DAC settings
            sv.minValueDAC.setBits(bits,sv.ID);
            sv.maxValueDAC.setBits(bits,sv.ID);
            sv.syncDelay.setBits(bits,sv.ID);
            sv.dacMode.setBits(bits,sv.ID);
            sv.dacOffValue.setBits(bits,sv.ID);
            sv.useExtSwitch.setBits(bits,sv.ID);
            
            %% General ramp parameters
            sv.sampleSource.setBits(bits,sv.ID);
            sv.updateTime.setBits(bits,sv.ID);
            sv.numSamples.setBits(bits,sv.ID);
            
            %% Linear ramp parameters
            sv.rampValues.setBits(bits,sv.ID);
            sv.rampBndyTimes.setBits(bits,sv.ID);
            sv.rampRates.setBits(bits,sv.ID);
            sv.rampCodes.setBits(bits,sv.ID);
            
            %% PID parameters
            sv.polarity.setBits(bits,sv.ID);
            sv.Kp.setBits(bits,sv.ID);
            sv.Ki.setBits(bits,sv.ID);
            sv.Kd.setBits(bits,sv.ID);
            sv.divisorPID.setBits(bits,sv.ID);
            sv.useFixed.setBits(bits,sv.ID);
            
            %% Memory parameters
            sv.numMemSamples.setBits(bits,sv.ID);
            sv.useSampleLimits.setBits(bits,sv.ID);
            sv.minSample.setBits(bits,sv.ID);
            sv.maxSample.setBits(bits,sv.ID);
            sv.sampleStep.setBits(bits,sv.ID);
            sv.requestedSamples.setBits(bits,sv.ID);
            
            %% Other properties
            sv.auxCmd.setBits(bits,sv.ID);
            sv.sampleData.setBits(bits,sv.ID);
            sv.loopData1.setBits(bits,sv.ID);
            sv.loopData2.setBits(bits,sv.ID);
        end
        
        function coerceValues(sv)    %#ok
            %COERCEVALUES Coerces property values to be in acceptable
            %ranges
            %   For the servo class, no coercion is needed
        end
        
        %% Serial port functions
        function open(sv)
            %OPEN Opens a serial port
            %   Uses the servo.comPort and servo.BAUD_RATE properties to
            %   open a serial port.  Checks for already open or existing
            %   interfaces and uses those if they exist.
            if isa(sv.ser,'serial') && isvalid(sv.ser) && strcmpi(sv.ser.port,sv.comPort)
                if strcmpi(sv.ser.status,'closed')
                    fopen(sv.ser);
                end
                return
            else
                r = instrfindall('type','serial','port',upper(sv.comPort));
                if isempty(r)
                    sv.ser=serial(sv.comPort,'baudrate',sv.BAUD_RATE);
                    sv.ser.OutputBufferSize = sv.BUFFER_SIZE;
                    sv.ser.InputBufferSize = sv.BUFFER_SIZE;
                    fopen(sv.ser);
                elseif strcmpi(r.status,'open')
                    sv.ser = r;
                else
                    sv.ser = r;
                    sv.ser.OutputBufferSize = sv.BUFFER_SIZE;
                    sv.ser.InputBufferSize = sv.BUFFER_SIZE;
                    fopen(sv.ser);
                end   
            end
        end
        
        function close(sv)
            %CLOSE Closes the serial port
            %   Closes and deletes the serial port associated with the
            %   servo controller.
            if isa(sv.ser,'serial') && isvalid(sv.ser) && strcmpi(sv.ser.port,sv.comPort)
                if strcmpi(sv.ser.status,'open')
                    fclose(sv.ser);
                end
                delete(sv.ser);
            end
        end
        
        %% DAC and manual functions
        function set.dacMode(sv,v)
            %SET.DACMODE Sets the mode (bi/unipolar) of the DAC
            if isa(v,'servoCmd')
                sv.dacMode = v;
            else
                error('Value must be a servoCmd');
            end
        end
        
        function dacSetup(sv)
            %DACSETUP Sets up the AD5791 to use offset-binary encoding
            c = bin2dec('001000000000000000010010');
            
            sv.auxCmd.setBits([27,24],0).setReg('reg1','00');
            sv.auxCmd.dataType = servoCmd.TYPE_UINT;
            sv.auxCmd.set(c);
            sv.auxCmd.write(sv.ser);
            
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setReg('reg1','01');
            sv.auxCmd.write(sv.ser);
            
            %Finish set-up by writing a zero value
            sv.dacWrite(0);
        end
        
        function dacWrite(sv,V)
            %DACWRITE Writes a voltage to the DAC
            %   Uses SERVO.DACREF and SERVO.DACMODE to convert a voltage to
            %   a DAC code that is then written to the device
            sv.open;
            c = sv.dacConvert(V,sv.dacMode.get,sv.dacRef);
            c = uint32(c)+bitshift(1,20);
            
            sv.auxCmd.setBits([27,24],0).setReg('reg1','00');
            sv.auxCmd.dataType = servoCmd.TYPE_UINT;
            sv.auxCmd.set(c);
            sv.auxCmd.write(sv.ser);
            
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setReg('reg1','01');
            sv.auxCmd.write(sv.ser);
        end
        
        function dacSwitch(sv,v)
            %DACSWITCH Turns the switch on/off according to the given value
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setBits([27,24],0).setReg('reg1','02');
            sv.auxCmd.onOff(v);
            sv.open;
            sv.auxCmd.write(sv.ser);
        end
        
        function adcTrig(sv)
            %ADCTRIG Triggers collection of data from the ADC
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setBits([27,24],0).setReg('reg1','03');
            sv.open;
            sv.auxCmd.write(sv.ser);
        end
        
        %% Triggers     
        function reset(sv)
            %RESET Resets the FPGA memory pointer
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setBits([27,24],1).setReg('reg1','00');
            sv.open;
            sv.auxCmd.write(sv.ser);
        end

        function start(sv)
            %START Manually triggers the start of the sequence
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setBits([27,24],1).setReg('reg1','01');
            sv.open;
            sv.auxCmd.write(sv.ser);
        end
        
        %% Data retrieval
        function setSampleLimits(sv,minSample,maxSample,sampleStep)
            %SETSAMPLELIMITS Sets the sample retrieval limits
            %   servo.setSampleLimits disables retrieval limits
            %   servo.setSampleLimits(minSample,maxSample) uses sample
            %   limits
            %   servo.setSampleLimits(minSample,maxSample,ss) uses sample
            %   step ss
            if nargin==1 || isempty(minSample)
                sv.useSampleLimits.set(0);
            elseif nargin < 4
                sv.useSampleLimits.set(1);
                sv.sampleStep.set(1);
                sv.minSample.set(minSample);
                sv.maxSample.set(maxSample);
            else
                sv.useSampleLimits.set(1);
                sv.sampleStep.set(sampleStep);
                sv.minSample.set(minSample);
                sv.maxSample.set(maxSample);
            end
            sv.open;
            sv.useSampleLimits.write(sv.ser);
            sv.minSample.write(sv.ser);
            sv.maxSample.write(sv.ser);
            sv.sampleStep.write(sv.ser);
        end
        
        function setTimeLimits(sv,dt,ti,tf,sampleStep)
            %SETTIMELIMITS Computes and sets the retrieval limits using
            %time limits
            minSample = round(ti/dt); %#ok
            maxSample = round(tf/dt); %#ok
            if nargin == 4
                sv.setSampleLimits(minSample,maxSample);    %#ok
            elseif nargin == 5
                sv.setSampleLimits(minSample,maxSample,sampleStep);    %#ok
            end
        end
        
        function read(sv,cmd)
            %READ Reads data from a command
            sv.open;
            cmd.write(sv.ser);
            pause(0.5);
            b = sv.ser.BytesAvailable;
            pause(0.1);
            while (b~=sv.ser.BytesAvailable)
                b = sv.ser.BytesAvailable;
                pause(0.5);
            end
            if mod(sv.ser.BytesAvailable,4) ~= 0
                fprintf('Error reading memory...retrying...\n');
                sv.close;
                sv.read(cmd);
            end
        end
        
        function data = readMemory(sv,readType)
            %READMEMORY Reads the memory using the given read type
            %   READMEMORY(readType) reads memory with readTypes 0,1,2,3 or
            %   strings 'sample','loop1','loop2','data'.
            %   Returns retrived data as a uint32 integer
            
            readType = lower(readType);
            if ischar(readType)
                switch readType
                    case 'sample'
                        readType = 0;
                    case 'loop1'
                        readType = 1;
                    case 'loop2'
                        readType = 2;
                    case 'data'
                        readType = 3;
                    otherwise
                        error('Read type not supported');
                end
            elseif isnumeric(readType)
                if ~(readType >= 0 && readType <= 3)
                    error('readType must be an appropriate string or a number in the range [0,3]');
                end
            else
                error('readType must be an appropriate string or a number in the range [0,3]');
            end
            
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setBits([27,24],1).setReg('reg1','02');
            sv.auxCmd.setReg('reg0',dec2hex(readType,2));
            
            sv.read(sv.auxCmd);
            data = fread(sv.ser,sv.ser.BytesAvailable/4,'uint32');
            data = uint32(data);

            sv.requestedSamples.read(sv.ser);
            if numel(data) ~= ceil(sv.requestedSamples.get/sv.sampleStep.get)
                if sv.retryOnWarning
                    disp('Number of data points retrieved not the same as number requested!  Retrying...');
                    data = sv.readMemory(readType);
                else
                    warning('Number of data points retrieved not the same as number requested!');
                end
            end

        end
        
        function [data,t] = getMemSamples(sv,cnv)
            %GETMEMSAMPLES Retrieves data in memory under SAMPLE_LOCATION
            %   GETMEMSAMPLES(cnv) applies conversion 'adc' or 'dac' to
            %   data
            data = sv.readMemory('sample');
            if sv.useCompression
                data = sv.decompressSamples(data);
            end
%             data = typecast(data,'uint32');
            codes = bitshift(data,-24,'uint32');
            data = data - bitshift(codes,24,'uint32');
            if nargin==1 || strcmpi(cnv,sv.TRANSMIT_ADC)
                data = sv.adcConvert(data);
            elseif strcmpi(cnv,sv.TRANSMIT_DAC)
                data = sv.dacConvert(data,sv.dacMode.get,sv.dacRef,'volt');
            end
            
            dt = sv.updateTime.get*sv.sampleStep.get;
            if sv.useSampleLimits.get == 0
                t = dt*(0:numel(data)-1);
            else
                t0 = sv.minSample.get*dt;
                t = t0+dt*(0:numel(data)-1);
            end
        end
        
        function [Kp,Ki,Kd,n,t] = getMemLoop(sv)
            %GETMEMLOOP Retrieves data in memory under LOOP1 and
            %LOOP2_LOCATION
            
            %LOOP1_LOCATION
            data = uint32(sv.readMemory('loop1'));
            data2 = typecast(data,'uint16');
            Kp = data2(1:2:end);
            Ki = data2(2:2:end);
            
            
            data = uint32(sv.readMemory('loop2'));
            data2 = typecast(data,'uint16');
            Kd = data2(1:2:end);
            n = data2(2:2:end);
            
            dt = sv.updateTime.get*sv.sampleStep.get;
            if sv.useSampleLimits.intValue == 0
                t = dt*(0:numel(data)-1);
            else
                t0 = sv.minSample.intValue*dt;
                t = t0+dt*(0:numel(data)-1);
            end
        end
        
        function [data,t] = getData(sv,cnv)
            %GETDATA Retrieves data in memory under SAMPLE_LOCATION
            %   GETDATA(cnv) applies conversion 'adc' or 'dac' to
            %   data
            data = sv.readMemory('data');
            
            if nargin==1 || strcmpi(cnv,sv.TRANSMIT_ADC)
                data = sv.adcConvert(data);
            elseif strcmpi(cnv,sv.TRANSMIT_DAC)
                data = data-2^20;
                data = sv.dacConvert(data,sv.dacMode.get,sv.dacRef,'volt');
            end
            if sv.transmitType.get == 2
                dt = sv.updateTime.get*sv.sampleStep.get;
            else
                sv.adcPeriod.read(sv.ser);
                dt = sv.adcPeriod.get*sv.sampleStep.get;
            end
            
            if sv.useSampleLimits.get == 0
                t=dt*(0:numel(data)-1);
            else
                t0=sv.minSample.get*dt;
                t=t0+dt*(0:numel(data)-1);
            end
            data = data(:);t = t(:);
            sv.v = data;
            sv.t=t;
        end
        
        %% Global parameters
        function set.spiPeriod(sv,val)
            if isa(val,'servoCmd')
                sv.spiPeriod = val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.transmitType(sv,val)
            if isa(val,'servoCmd')
                sv.transmitType = val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.enableTrig(sv,val)
            if isa(val,'servoCmd')
                sv.enableTrig = val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.offTime(sv,val)
            if isa(val,'servoCmd')
                sv.offTime=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function adcStart(sv,v)
            %ADCSTART Sets the value of the ADC_START signal, which tells
            %the ADC whether or not to start conversions
            sv.auxCmd.dataType = servoCmd.TYPE_CMD;
            sv.auxCmd.setBits([27,24],2).setReg('reg1','04');
            sv.auxCmd.onOff(v);
            sv.open;
            sv.auxCmd.write(sv.ser);
        end
        
        %% DAC settings
        function set.minValueDAC(sv,val)
            if isa(val,'servoCmd')
                sv.minValueDAC=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.maxValueDAC(sv,val)
            if isa(val,'servoCmd')
                sv.maxValueDAC=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.syncDelay(sv,val)
            if isa(val,'servoCmd')
                sv.syncDelay=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.dacOffValue(sv,val)
            if isa(val,'servoCmd')
                sv.dacOffValue=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.useExtSwitch(sv,val)
            if isa(val,'servoCmd')
                sv.useExtSwitch=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        %% General ramp settings
        function set.sampleSource(sv,val)
            if isa(val,'servoCmd')
                sv.sampleSource=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.updateTime(sv,val)
            if isa(val,'servoCmd')
                sv.updateTime=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.numSamples(sv,val)
            if isa(val,'servoCmd')
                sv.numSamples=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        
        %% Linear ramp settings
        function set.rampValues(sv,val)
            if isa(val,'servoCmd')
                sv.rampValues=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.rampBndyTimes(sv,val)
            if isa(val,'servoCmd')
                sv.rampBndyTimes=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.rampRates(sv,val)
            if isa(val,'servoCmd')
                sv.rampRates=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.rampCodes(sv,val)
            if isa(val,'servoCmd')
                sv.rampCodes=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        
        %% PID parameters
        function set.polarity(sv,val)
            if isa(val,'servoCmd')
                sv.polarity=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.Kp(sv,val)
            if isa(val,'servoCmd')
                sv.Kp=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.Ki(sv,val)
            if isa(val,'servoCmd')
                sv.Ki=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.Kd(sv,val)
            if isa(val,'servoCmd')
                sv.Kd=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.divisorPID(sv,val)
            if isa(val,'servoCmd')
                sv.divisorPID=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.useFixed(sv,val)
            if isa(val,'servoCmd')
                sv.useFixed=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        %% Memory parameters
        function set.numMemSamples(sv,val)
            if isa(val,'servoCmd')
                sv.numMemSamples=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.useSampleLimits(sv,val)
            if isa(val,'servoCmd')
                sv.useSampleLimits=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.minSample(sv,val)
            if isa(val,'servoCmd')
                sv.minSample=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        function set.maxSample(sv,val)
            if isa(val,'servoCmd')
                sv.maxSample=val;
            else
                error('Must be a servoCmd');
            end
        end

        function set.sampleStep(sv,val)
            if isa(val,'servoCmd')
                sv.sampleStep=val;
            else
                error('Must be a servoCmd');
            end
        end
        
        %% Memory uploading
        function sampleUpload(sv,V,codes)
            %SAMPLEUPLOAD Uploads a series of integer values corresponding
            %to samples and associate sample codes.
            V = V(:);
            N = numel(V);
            if numel(codes)==1
                codes = codes*ones(N,1);
            elseif numel(codes)~=N
                error('Number of codes must be either 1 or the same as the number of voltages!');
            end
            codes = max(min(round(codes(:)),255),0);
            
            data = typecast(int32(V),'uint32');
            data = bitshift(bitshift(data,8),-8);
            data = data+bitshift(uint32(codes),sv.MEM_CODE_SHIFT);
            if sv.useCompression
                data = servo.compressSamples(data,true);
            end
            N = numel(data);
            sv.sampleData.set(data);
            sv.open;
            sv.numMemSamples.set(N-1).write(sv.ser);
            sv.numSamples.set(N).write(sv.ser);
            sv.sampleData.write(sv.ser);
        end
        
        function loopUpload(sv,Kp,Ki,Kd,N)
            %LOOPUPLOAD Uploads loop parameters for set-point dependent
            %loop parameters.
            %   servo.loopUpload(Kp,Ki,Kd,N) uploads proportional gain Kp,
            %   integral gain Ki, derivative gain Kd, and overall divisor N
            if any(Kp < 0) || any(Kp > (2^16-1))
                error('Values for Kp must be in the range of [0,2^16-1]');
            elseif any(Ki < 0) || any(Ki > (2^16-1))
                error('Values for Ki must be in the range of [0,2^16-1]');
            elseif any(Kd < 0) || any(Kd > (2^16-1))
                error('Values for Kd must be in the range of [0,2^16-1]');
            elseif any(N < 0) || any(N > (2^16-1))
                error('Values for n must be in the range of [0,2^16-1]');
            end
            
            if numel(Kp)~=numel(Ki) || numel(Kp)~=numel(Ki) || numel(Kp)~=numel(N)
                error('Sizes of gain parameters must be the same');
            else
                M = numel(Kp);
            end
            
            loop1 = uint32(bitshift(Ki(:),16)+Kp(:));
            loop2 = uint32(bitshift(N(:),16)+Kd(:));
            if sv.useCompression
                loop1 = sv.compressSamples(loop1,false);
                loop2 = sv.compressSamples(loop2,false);
            end
            M = numel(loop1);
            sv.loopData1.set(loop1);
            sv.loopData2.set(loop2);
            
            sv.open;
            sv.numMemSamples.set(M-1).write(sv.ser);
            sv.numSamples.set(M).write(sv.ser);
            sv.loopData1.write(sv.ser);
            sv.loopData2.write(sv.ser);
        end
        
        %% Device writing
        function upload(sv)
            %UPLOAD Uploads all parameters to the device
            sv.open;
            
            %Global parameters
            sv.spiPeriod.write(sv.ser);
            sv.transmitType.write(sv.ser);
            sv.enableTrig.write(sv.ser);
            sv.offTime.write(sv.ser);
            
            %DAC settings
            sv.minValueDAC.write(sv.ser);
            sv.maxValueDAC.write(sv.ser);
            sv.syncDelay.write(sv.ser);
            sv.dacMode.write(sv.ser);
            sv.dacOffValue.write(sv.ser);
            sv.useExtSwitch.write(sv.ser);

            %General ramp settings
            sv.rampValues.write(sv.ser);
            sv.rampBndyTimes.write(sv.ser);
            sv.rampRates.write(sv.ser);
            sv.rampCodes.write(sv.ser);
            
            %Linear ramp parameters
            sv.sampleSource.write(sv.ser);
            sv.updateTime.write(sv.ser);
            sv.numSamples.write(sv.ser);
            
            %PID parameters
            sv.polarity.write(sv.ser);
            sv.Kp.write(sv.ser);
            sv.Ki.write(sv.ser);
            sv.Kd.write(sv.ser);
            sv.divisorPID.write(sv.ser);
            sv.useFixed.write(sv.ser);
            
            %Memory parameters
            sv.useSampleLimits.write(sv.ser);
            sv.minSample.write(sv.ser);
            sv.maxSample.write(sv.ser);
            sv.sampleStep.write(sv.ser);

        end
        
        %% Plot generation
        function [v,t]=makePlotVec(sv)
            %MAKEPLOTVEC Makes voltage and time vectors from the parameters
            %comprising the linear ramps.  Useful for comparing measurement
            %with expected values.
            dt = sv.updateTime.get;
            v = [];
            t = [];
            
            rval = sv.rampValues.get;
            btime = sv.rampBndyTimes.get;
            rcode = sv.rampCodes.get;
            
            for nn=1:(numel(rval)-1)
                t2 = btime(nn):btime(nn+1);
                if rcode(nn)==0
                    volt1 = servo.adcConvert(round(rval(nn+1)),'volt');
                    volt0 = servo.adcConvert(round(rval(nn)),'volt');
                elseif rcode(nn)==1
                    volt1 = servo.dacConvert(round(rval(nn+1)),sv.dacMode.get,sv.dacRef,'volt');
                    volt0 = servo.dacConvert(round(rval(nn)),sv.dacMode.get,sv.dacRef,'volt');
                end
                v2 = (volt1-volt0)/(btime(nn+1)-btime(nn))*(t2-t2(1))+volt0;
                v = [v,v2];   %#ok
                t = [t,t2];   %#ok
            end
            t = t*dt;
        end
        
        function [v,t,c] = simLinRamps(sv,cnv)
            %SIMLINRAMPS Simulates the linear ramp generation process to
            %compare with what is measured.
            
            rval = sv.rampValues.get;
            btime = sv.rampBndyTimes.get;
            rrate = sv.rampRates.get;
            rcode = sv.rampCodes.get;
            
            v = zeros(btime(end),1);
            c = zeros(btime(end),1);
            rampIdx = 1;
            for nn=0:(btime(end)-1)
                if nn == btime(rampIdx)
                    v(nn+1) = rval(rampIdx);
                    c(nn+1) = rcode(rampIdx);
                elseif nn > btime(rampIdx) && nn < (btime(rampIdx+1)-1)
                    v(nn+1) = v(nn) + rrate(rampIdx);
                    c(nn+1) = rcode(rampIdx);
                elseif nn == (btime(rampIdx+1)-1)
                    v(nn+1) = v(nn) + rrate(rampIdx);
                    rampIdx = rampIdx + 1;
                else
                    v(nn+1) = rval(1);
                    c(nn+1) = rcode(1);
                    rampIdx = 1;
                end 
            end
            
            t = sv.updateTime.get*(0:(numel(v)-1));
            if strcmpi(cnv,'adc')
                v = servo.adcConvert(v,'volt');
            elseif strcmpi(cnv,'dac')
                v = servo.dacConvert(v,sv.dacMode.get,sv.dacRef,'volt');
            end
        end
        
    end
    
    methods(Static)
        function val=dacConvert(in,mode,refVoltage,direction)
            %DACCONVERT Converts either a voltage to a DAC code or vice
            %versa
            %   val=servo.dacConvert(in,mode) converts a voltage to a code
            %   using the given polarity mode, either servo.DAC_MODE_UNI or
            %   servo.DAC_MODE_BI.  Default DAC ref of servo.DAC_REF is
            %   used
            %   
            %   val=servo.dacConvert(in,mode,refVoltage) converts a voltage
            %   to a code using the given mode and the given reference
            %   voltage corresponding to the maximum DAC code
            %
            %   val=servo.dacConvert(in,mode,refVoltage,'volt') converts a 
            %   DAC code to a voltage.
            %
            %   val=servo.dacConvert(in,mode,refVoltage,'code') converts a
            %   voltage to a DAC code
            if nargin<3
                refVoltage=servo.DAC_REF;
            end
            if nargin<4
                direction='code';
            end
            
            if isa(mode,'servoCmd')
                mode = mode.get;
            end
            
            if strcmpi(direction,'code')
                if nargin==1 || strcmpi(mode,servo.DAC_MODE_BI)
                    val=in./(2*refVoltage)+0.5;
                elseif strcmpi(mode,servo.DAC_MODE_UNI)
                    val=in./(refVoltage);
                else
                    error('Mode must be either %s or %s',servo.DAC_MODE_BI,servo.DAC_MODE_UNI);
                end
                val=round(val.*2^servo.DAC_NUM_BITS);
                val=min(max(val,0),2^servo.DAC_NUM_BITS-1);
            elseif strcmpi(direction,'volt')
                if strcmpi(mode,servo.DAC_MODE_BI)
                    val=in/2^servo.DAC_NUM_BITS*2*refVoltage-refVoltage;
                elseif strcmpi(mode,servo.DAC_MODE_UNI)
                    val=in/2^servo.DAC_NUM_BITS*refVoltage;
                else
                    error('Mode must be either %s or %s',servo.DAC_MODE_BI,servo.DAC_MODE_UNI);
                end
            else
                error('Direction must be either code or volt');
            end
        end
        
        function val=adcConvert(valIn,direction)
            %ADCCONVERT Converts a value from an ADC code to a voltage or
            %vice versa
            %
            %   val=servo.adcConvert(valIn) converts an ADC code to a
            %   voltage
            %
            %   val=servo.adcConvert(valIn,'volt') converts an ADC code to
            %   a voltage
            %
            %   val=servo.adcConvert(valIn,'code') converts a voltage to an
            %   ADC code
            if nargin==1
                direction = 'volt';
            end
            if strcmpi(direction,'volt')
                val = (double(valIn)-double(2^servo.ADC_NUM_BITS*bitget(valIn,servo.ADC_NUM_BITS)))/2^servo.ADC_NUM_BITS*(2*servo.ADC_REF);
            elseif strcmpi(direction,'code')
                val = valIn*2^servo.ADC_NUM_BITS/(2*servo.ADC_REF);
            else
                error('Direction must be either code or volt');
            end
        end
        
        function plotTime(data,dt,sm)
            %PLOTIME Plots data as a function of time with a smoothed value
            %removed from the data
            %
            %   servo.plotTime(data) plots data
            %
            %   servo.plotTime(data,dt) plots the data with time step dt
            %
            %   servo.plotTime(data,dt,sm) plots the data with time step dt
            %   and with the smoothed signal smooth(data,sm) removed
            if nargin==1
                t=(0:size(data,1)-1)';
            else
                t=dt*(0:size(data,1)-1)';
            end
            if nargin==3
                y=servo.smooth(data,sm);
            else
                y=data;
            end
            plot(t,y);
        end
        
        function plotFreq(data,dt,sm,plotType)
            %PLOTFREQ Plots FFT of the data
            %
            %   servo.plotFreq(data,dt) plots the FFT of the data with time
            %   step dt.  RMS power is plotted
            %   servo.plotFreq(data,dt,sm) plots the FFT of the data with
            %   time step dt with the smoothed signal removed.  Useful for
            %   removing DC offsets and other low frequency components.
            %   RMS power is plotted
            %   servo.plotFreq(data,dt,sm,plotType) plots the FFT of the
            %   data with different y axes.  Allowed values are 'pow' for
            %   RMS power, 'amp' for RMS amplitude, and 'nsd' for noise
            %   spectral density.
            if nargin>=3
                [Y,f]=servo.calcFFT(data,dt,sm);
            else
                [Y,f]=servo.calcFFT(data,dt);
            end
            Y=Y/sqrt(2);    %Appropriate for noise calculations as RMS/average values
            if nargin<4 || strcmpi(plotType,'pow')
                plot(f,abs(Y).^2);
                xlabel('Frequency [Hz]');ylabel('Average Power [arb. units]');
            elseif strcmpi(plotType,'amp')
                plot(f,abs(Y));
                xlabel('Frequency [Hz]');ylabel('RMS Amplitude [V]');
            elseif strcmpi(plotType,'nsd')
                plot(f,abs(Y)/sqrt(diff(f(1:2))));
                xlabel('Frequency [Hz]');ylabel('Noise spectral density [VHz^{-1/2}]');
            end
            xlim([0,max(f)]);
        end
        
        function [Y,f]=calcFFT(data,dt,sm)
            %CALCFFT Calculates the FFT of the data
            %
            %   servo.calcFFT(data,dt) calculates the FFT of the data with 
            %   time step dt.
            %
            %   servo.calcFFT(data,dt,sm) calculates the FFT of the data 
            %   with time step dt with the smoothed signal removed.  
            N=size(data,1);
            f=1/(dt)*(0:N-1)/(N);
            f=f(1:floor(N/2));
            f=f(:);
            if nargin==3
                data=servo.smooth(data,sm);
            end
            Y=fft(data,[],1);
            Y=Y(1:floor(N/2),:);
            Y(1,:) = Y(1,:)/N;
            Y(2:end,:) = 2*Y(2:end,:)/N;
        end
        
        function plotCumNoise(data,dt,sm)
            %PLOTCUMNOISE Plots the cumulative RMS noise of the data
            %
            %   servo.plotCumNoise(data,dt) plots the cumulative noise of
            %   the data
            %
            %   servo.plotCumNoise(data,dt,sm) removes the smoothed signal
            %   with smoothing sm.
            if nargin>=3
                [Y,f]=servo.calcFFT(data,dt,sm);
            else
                [Y,f]=servo.calcFFT(data,dt);
            end
            Y=Y/sqrt(2);    %Appropriate for noise calculations as RMS/average values
            plot(f,sqrt(cumsum(abs(Y).^2,1)));
        end
        
        function N=getCumNoise(data,dt,sm,r)
            %GETCUMNOISE Returns the cumulative noise of data
            if nargin>=3
                [Y,f]=servo.calcFFT(data,dt,sm);
            else
                [Y,f]=servo.calcFFT(data,dt);
            end
            Y=Y/sqrt(2);    %Appropriate for noise calculations as RMS/average values
            Y=Y(f>=min(r) & f<max(r),:);
            N=sqrt(mean(sum(abs(Y).^2,1),2));
        end
        
        function y=filter(data,dt,fHigh,fLow)
            %FILTER Filters the data with a 5th order Butterworth filter.
            N=size(data,1);
            f=1/(dt)*(0:N-1)/N;
            f=f-f(round(N/2));
            f=f(:);
            Y=fftshift(fft(data,[],1),1);
            if nargin==3 || isempty(fLow)
                Filt=1./(1+(f/fHigh).^10);
            elseif nargin==4 && isempty(fHigh)
                Filt=1-1./(1+(f/fLow).^10);
            else
                Filt=1./(1+(f/fHigh).^10).*(1-1./(1+(f./fLow).^10));
            end
            Y=bsxfun(@times,Y,Filt);
            y=real(ifft(ifftshift(Y,1),[],1));
        end
        
        function y=smooth(data,sm)
            %SMOOTH Removes the smoothed component of the data
            %
            %   y=servo.smooth(data,sm) calculates the smoothed signal
            %   smooth(data,sm) for each column and removes it from data
            if nargin==1 || isempty(sm)
                y=data;
            elseif sm==0
                y=bsxfun(@minus,data,mean(data,1));
            else
                y=zeros(size(data));
                for nn=1:size(data,2)
                    y(:,nn)=data(:,nn)-smooth(data(:,nn),sm);
                end
            end
        end
        
        function [y,t]=minJerk(dt,T,s0,sf)
            %MINJERK Calculates a minimum jerk trajectory
            %
            %   [y,t]=servo.minJerk(dt,T,s0,sf) calculates a minimum jerk
            %   trajectory with time step dt, total time T, initial
            %   position s0, and final position sf
            t=0:dt:T;
            y=s0+(sf-s0).*(10*(t/T).^3-15*(t/T).^4+6*(t/T).^5);
        end
        
        function makeFigure(data,dt,sm)
            %MAKEFIGURE Makes a figure showing various analyses
            if nargin == 1
                dt = 32e-6;
            end
%             sm2 = round(size(data,1)/4);
            if nargin < 3 && dt < 1e-3
                sm = round(100e-3/dt);
            elseif nargin < 3
                sm = round(100e-3/dt);
            end
            t = dt*(0:size(data,1)-1);


            subplot(3,2,1);
            servo.plotTime(data*1e6,dt,sm);
            y = 1e6*servo.smooth(data,sm);
            hold on;
            xlabel('Time [s]');ylabel('Voltage deviation [\muV]');

            subplot(3,2,3);
            servo.plotTime(data,dt,[]);
            hold on;
            xlabel('Time [s]');ylabel('Voltage [V]');

            subplot(3,2,5);
            hold on;
            y = y(:);
            x = linspace(-200,200,200);
            cc = histcounts(y,x);
            x = (x(2:end)+x(1:end-1))/2;
%             bar(x,cc/sum(cc),'hist');
            plot(x,cc/sum(cc),'.-');
            hold on;
            xlabel('Voltage deviation [\muV]');ylabel('Probability');title(sprintf('V_{rms} = %.2f \\muV',std(y)));

            subplot(3,2,2);
            hold on;
            servo.plotCumNoise(data*1e6,dt*1e3,sm);
            xlabel('Frequency [kHz]');ylabel('Cumulative noise [\muV]');
            xlim([0,Inf]);
            set(gca,'xtick',0:(1/(2*dt*1e3)));
            title(sprintf('%.2f \\muV_{rms} (0 - 1 kHz), %.2f \\muV_{rms} (1 - 15 kHz)',servo.getCumNoise(data*1e6,dt,sm,[0,1e3]),servo.getCumNoise(data*1e6,dt,sm,[1e3,15e3])));
            grid on;

            subplot(3,2,[4,6]);
            hold on;
            servo.plotFreq(data*1e6/sqrt(1e3),dt*1e3,sm,'nsd');
            xlabel('Frequency [kHz]');ylabel('Noise spectral density [\muVkHz^{-1/2}]');
            xlim([0,Inf]);
            ylim([0,10]);
        end
        
        %% Conversion functions
        function r = getTransmitType(x,opt)
            %GETTRANSMITTYPE Returns either the string label for the given
            %transmission type or the integer value
            %
            %   r = servo.getTransmitType(x,'to') converts a string x
            %   indicator to an integer
            %
            %   r = servo.getTransmitType(x,'from') converts an integer x
            %   value to a string indicator
            if strcmpi(opt,'to')
                x = lower(x);
                switch x
                    case servo.TRANSMIT_ADC
                        r = 0;
                    case servo.TRANSMIT_DAC
                        r = 1;
                    case servo.TRANSMIT_CNTRL
                        r = 2;
                    case servo.TRANSMIT_ERR
                        r = 3;
                    otherwise
                        error('Only types of transmission are: %s, %s, %s, and %s',servo.TRANSMIT_ADC,servo.TRANSMIT_DAC,servo.TRANSMIT_CNTRL,servo.TRANSMIT_ERR);
                end
            else
                switch x
                    case 0
                        r = servo.TRANSMIT_ADC;
                    case 1
                        r = servo.TRANSMIT_DAC;
                    case 2
                        r = servo.TRANSMIT_CNTRL;
                    case 3
                        r = servo.TRANSMIT_ERR;
                    otherwise
                        warning('Transmission value not recognized');
                end
            end
        end
        
        function r = getSampleSource(x,opt)
            %GETSAMPLESOURCE Returns either the string label for the given
            %sample source or the integer value
            %
            %   r = servo.getSampleSource(x,'to') converts a string x
            %   indicator to an integer
            %
            %   r = servo.getSampleSource(x,'from') converts an integer x
            %   value to a string indicator
            if strcmpi(opt,'to')
                x = lower(x);
                switch x
                    case servo.SOURCE_INT
                        r = 0;
                    case servo.SOURCE_MEM
                        r = 1;
                    otherwise
                        error('Only two options are allowed: %s and %s',servo.SOURCE_INT,servo.SOURCE_MEM);
                end
            else
                switch x
                    case 0
                        r = servo.SOURCE_INT;
                    case 1
                        r = servo.SOURCE_MEM;
                    otherwise
                        warning('Sample source not recognized');
                end
            end
        end
        
        function r = getDacMode(x,opt)
            %GETDACMODE Returns either the string label for the given
            %DAC mode or the integer value
            %
            %   r = servo.getDacMode(x,'to') converts a string x
            %   indicator to an integer
            %
            %   r = servo.getDacMode(x,'from') converts an integer x
            %   value to a string indicator
            if strcmpi(opt,'to')
                x = lower(x);
                switch x
                    case servo.DAC_MODE_UNI
                        r = 0;
                    case servo.DAC_MODE_BI
                        r = 1;
                    otherwise
                        error('Only two options are allowed: %s and %s',servo.DAC_MODE_UNI,servo.DAC_MODE_BI);
                end
            else
                switch x
                    case 0
                        r = servo.DAC_MODE_UNI;
                    case 1
                        r = servo.DAC_MODE_BI;
                    otherwise
                        warning('DAC mode not recognized');
                end
            end
        end
        
        function r = getPolarity(x,opt)
            %GETPOLARITY Returns either the string label for the given
            %PID polarity or the integer value
            %
            %   r = servo.getPolarity(x,'to') converts a string
            %   indicator to an integer
            %
            %   r = servo.getPolarity(x,'from') converts an integer
            %   value to a string indicator
            if strcmpi(opt,'to')
                x = lower(x);
                switch x
                    case servo.PID_NEGATIVE
                        r = 0;
                    case servo.PID_POSITIVE
                        r = 1;
                    otherwise
                        error('Only two options are allowed: %s and %s',servo.PID_NEGATIVE,servo.PID_POSITIVE);
                end
            else
                switch x
                    case 0
                        r = servo.PID_NEGATIVE;
                    case 1
                        r = servo.PID_POSITIVE;
                    otherwise
                        warning('PID polarity not recognized');
                end
            end
        end
        
        function y = compressSamples(data,includeCode)
            %COMPRESSSAMPLES Compresses values in DATA and returns
            %compressed samples.
            %
            % For any consecutive length of data > 1000 samples,
            % compression works by replacing those samples with 1 sample at
            % the beginning, followed by a code with a '1' in the MSB and
            % the next 31 bits being the number of samples that were
            % replaced.
            %
            % Optional parameter INCLUDECODE is by default true, but can be
            % set to false.  This prevents the program from inserting the
            % number of samples with an MSB = 1.  This is used when writing
            % loop parameters
            % 
            if nargin == 1
                includeCode = true;
            end
            N = numel(data);
            if isa(data,'uint32')
                y = zeros(numel(data),1,'uint32');
                buf = zeros(numel(data),1,'uint32');
%                 bufCount = uint32(0);
%                 yCount = uint32(1);
            else
                data = int32(data);
                y = zeros(numel(data),1,'int32');
                buf = zeros(numel(data),1,'int32');
%                 bufCount = int32(0);
%                 yCount = int32(1);
            end
            bufCount = 0;
            yCount = 1;
            for nn=1:N
                if (nn < N) && (data(nn) == data(nn+1))
                    bufCount = bufCount + 1;
                    buf(bufCount) = data(nn);
                elseif bufCount == 0
                    y(yCount) = data(nn);
                    yCount = yCount + 1;
                elseif bufCount < 1000
                    y(yCount + (0:(bufCount-1))) = buf(1:(bufCount));
                    yCount = yCount + bufCount;
                    y(yCount) = data(nn);
                    yCount = yCount + 1;
                    bufCount = 0;
                else
                    y(yCount) = buf(1);
                    if includeCode
                        y(yCount+1) = bufCount;
                        y(yCount+1) = bitset(y(yCount+1),32,1);
                        yCount = yCount+2;
                    else
                        yCount = yCount+1;
                    end
                    bufCount = 0;
                end
            end
            y = y(1:(yCount-1));
        end
        
        function y=decompressSamples(data)
            %DECOMPRESSSAMPLES Decompresses samples that were compressed by
            %SERVO/COMPRESSSAMPLES.
            %
            % Assumes that the MSB is an indicator for how many times to
            % repeat the previous value.
            if isa(data,'uint32')
                data = uint32(data);
                N = numel(data);
                y = zeros(10*numel(data),1,'uint32');
            elseif isa(data,'int32')
                data = int32(data);
                N = numel(data);
                y = zeros(10*numel(data),1,'int32');
            else
                error('Problem decompressing: inputs should be int32 or uint32');
            end
            yCount = 1;
            for nn=1:N
                if bitget(data(nn),32) == 1
                    n = bitand(2^24-1,data(nn));
                    y(yCount + (0:(n-1))) = y(yCount-1);
                    yCount = yCount + n;
                else
                    y(yCount) = data(nn);
                    yCount = yCount + 1;
                end
            end
            y = y(1:(yCount-1));
        end
        
        
    end %end static methods
    
    
end %end classdef


