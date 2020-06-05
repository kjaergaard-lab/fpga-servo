%measureClosedLoopLinear Example file that shows how to measure the
%closed loop response of the servo when connected to a simple buffered RC
%filter, using different target frequencies for the closed-loop low-pass
%filter
%
%   To test this file out, refer to the documentation.

addpath('..');

%% Initialize
sv=servo(0);                            %Create the servo object
sv.comPort = 'com4';                    %Set the com port to the appropriate value for your PC
%
% dacRef and dacMode have to be set at the start since othre properties
% depend on them correctly being set
%
sv.dacRef = 10;                         %Set the DAC ref voltage to 10 V.
sv.dacMode.set(sv.DAC_MODE_UNI);        %Set the DAC mode to unipolar.  This assumes that VREFN is set to AGND

sv.open;                                %Create and open the serial object

%% Set global parameters
sv.spiPeriod.set(100e-9);               %Set the SPI period to 100 ns
sv.transmitType.set('adc');             %Set the transmit type to ADC, so data stored and retrieved will be voltages recorded by ADC
sv.enableTrig.set(true);                %Enable triggers
sv.offTime.set(1);                      %Set the offTime to 1.5 s

%% Set DAC settings
sv.minValueDAC.set(0);                  %Set the minimum DAC value to 0 V
sv.maxValueDAC.set(2.5);                %Set the maximum DAC value to 2.5 V
sv.syncDelay.set(0);                    %Set the SYNC delay to 0 s - it's not needed with the AD5791
sv.dacOffValue.set(0);                  %Set the DAC off value to 0 V
sv.useExtSwitch.set(false);             %Disable the use of external switch inputs

%% Sample generator settings
sv.sampleSource.set('mem');             %Set the source of new samples to 'memory', i.e. samples are drawn from RAM  
sv.updateTime.set(32e-6);               %Set the sample update time to 32 us

%sv.numSamples is calculated after the arbitrary ramps are calculated

%% PID settings
%
% These settings should regulate the measured ADC voltage if the DAC output
% is connected directly to the ADC evaluation board's inputs.
%
sv.polarity.set(servo.PID_NEGATIVE);        %Set the servo polarity to negative - this is the most likely configuration
sv.useFixed.set(1);                         %Use fixed gains only.  

%% Memory settings
sv.useSampleLimits.set(false);              %Disables sample retrieval limits - when data is read back, will retrieve ALL data
sv.sampleStep.set(1);                       %Set the sample step to 1 - unnecessary with useSampleLimits set to false

%% Upload parameters
sv.dacSetup;                                %Enable the DAC output and set encoding to offset-binary
sv.upload;                                  %Upload all parameters
sv.reset;                                   %Reset the sample generator memory pointer

%% Record open-loop response
%
% The following code creates arbitrary voltages with a sin-wave modulation,
% and then records and stores the ADC measurement.

targetFreq = [250,500,1000,2000];
P = load('Open loop system parameters');

modFreq = [10:10:100,200:100:1000,2e3:1e3:10e3];
Vdc = 1;
Vmod = 0.1;
tRamp = 100e-3;
tMod = 500e-3;
totalTime = tRamp + tMod;
dt = sv.updateTime.get;
totalSamples = floor(totalTime/dt);
savedSamples = totalSamples-1e3;

data(numel(modFreq),numel(targetFreq)) = struct('v',[],'t',[]);

%
% Get the ADC sample period - needed for calculating PID gains
%
Ts = sv.adcPeriod.read.get;


for mm = 1:numel(targetFreq)
    figure(1);clf(1);figure(2);clf(2);
    %
    % Set the PID gains using knowledge of the open loop dynamics
    %
    N = 10;
    gain(mm).Kp = round(targetFreq(mm)/(P.G0*P.w1)*2^N);
    gain(mm).Ki = round(2*pi*targetFreq(mm)/P.G0*Ts*2^N);
    gain(mm).Kd = round(targetFreq(mm)/(2*pi*P.G0*P.w2^2*Ts)*2^N);
    gain(mm).N = N+5;                   %This additional 5 is due to DAC/ADC voltage scaling.  See documentation
    
    sv.Kp.set(gain(mm).Kp).write;
    sv.Ki.set(gain(mm).Ki).write;
    sv.Kd.set(gain(mm).Kd).write;
    sv.divisorPID.set(gain(mm).N).write; 
    
    for nn = 1:numel(modFreq)
        fprintf(1,'Run %d/%d, Mod Freq: %d Hz, Target Freq: %d Hz\n',nn,numel(modFreq),modFreq(nn),targetFreq(mm));
        %
        % Creates a modulation signal
        %
        [V0,t0] = sv.minJerk(dt,tRamp-dt,0,Vdc);
        t1 = (t0(end)+dt) + (0:dt:tMod);
        V1 = V0(end) + 0.1*sin(2*pi*modFreq(nn)*(t1-t1(1)));
        t = [t0 t1 t1(end)+dt];
        V = [V0 V1 0];
        %
        % Plot the uploaded signal
        %
        figure(1);clf;
        plot(t,V);
        xlabel('Time [s]');
        ylabel('DAC voltage [V]');
        drawnow;
        %
        % Upload the signal and start
        %
        sv.sampleUpload(sv.adcConvert(V,'code'),0);
        sv.numSamples.set(numel(V)).write;
        sv.start;
        %
        % Wait for ramp to finish and get data
        %
        pause(totalTime+0.25);
        sv.getData('adc');
        data(nn,mm).v = sv.v(1:savedSamples);
        data(nn,mm).t = sv.t(1:savedSamples);
        %
        % Plot the acquired data
        %
        figure(2);
        plot(sv.t,sv.v);
        hold on;
        xlabel('Time [s]');
        ylabel('Measured voltage [V]');
    end
end

save('Closed loop response','data','modFreq','Vdc','Vmod','tRamp','tMod','savedSamples','targetFreq','gain');
rmpath('..');