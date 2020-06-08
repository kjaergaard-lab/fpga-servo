%programServo_ArbSamples_FixedGain Example file that shows how to use the SERVO and SERVOCMD
%methods to program a particular sequence into the servo controller.
%
%   This file demonstrates how to use arbitrary voltage signals with fixed
%   gain
%
%   To test this file out, you should connect the output of the DAC
%   evaluation board (VOUT_BUF) to the ADC input on the ADC evaluation
%   board.  Connect the signal line of VOUT_BUF to AINN and the ground
%   shield to AINP, as the differential amplifier on the evaluation board
%   inverts the voltages.

%% Initialize
sv=servo(0);                            %Create the servo object
sv.comPort = 'com4';                    %Set the com port to the appropriate value for your PC
%
% dacRef and dacMode have to be set at the start since other properties
% depend on them correctly being set
%
sv.dacRef = 10;                         %Set the DAC ref voltage to 10 V.
sv.dacMode.set(sv.DAC_MODE_UNI);        %Set the DAC mode to unipolar.  This assumes that VREFN is set to AGND
sv.retryOnWarning = true;               %Retries retrieving data when number of retrieved samples does not match expected number
sv.useCompression = false;              %Disables sample compression.
sv.open;                                %Create and open the serial object

%% Set global parameters
sv.spiPeriod.set(100e-9);               %Set the SPI period to 100 ns
sv.transmitType.set('adc');             %Set the transmit type to ADC, so data stored and retrieved will be voltages recorded by ADC
sv.enableTrig.set(true);                %Enable triggers
sv.offTime.set(1.5);                    %Set the offTime to 1.5 s

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
sv.Kp.set(250);                             %Set the proportional gain to 250
sv.Ki.set(120);                             %Set the integral gain to 120
sv.Kd.set(100);                             %Set the derivative gain to 100
sv.divisorPID.set(15);                      %Set the overall divisor to 10 + 5 = 15 (divide by 2^15).  The 2^5 comes from the difference in voltage scaling between the ADC and DAC
sv.useFixed.set(1);                         %Use fixed gains only.  


%% Set arbitrary ramps
%
% The following code creates an arbitrary voltage profile that has a
% voltage start at 0 V, increase using a minimum jerk trajectory in 100 ms to 1 V,
% apply a sin^3 oscillation for 400 ms, then go back down to 0 V using the
% same minimum jerk trajectory
dt = sv.updateTime.get;

[V0,t0] = sv.minJerk(dt,100e-3-dt,0,1);
t1 = (t0(end)+dt):dt:500e-3;
V1 = V0(end) + 0.1*sin(2*pi*(t1-100e-3)/10e-3).^3;
[V2,t2] = sv.minJerk(dt,100e-3-dt,V1(end),0);
t2 = t1(end)+dt+t2;
t = [t0 t1 t2];
V = [V0 V1 V2];

figure(1);clf;
plot(t,V);

%
% Comment out one of the following lines to have the DAC output be directly
% read from memory, or be calculated using the PID process.
%
sv.sampleUpload(sv.dacConvert(V,sv.dacMode,sv.dacRef),1);       %Set all codes to 1, which means that the PID process is DISABLED and the DAC output is controlled directly by samples read from memory
% sv.sampleUpload(sv.adcConvert(V,'code'),0);                     %Set all codes to 0, which means that the PID process is ENABLED and the DAC output is calculated via the PID process

%% Memory settings
sv.useSampleLimits.set(false);              %Disables sample retrieval limits - when data is read back, will retrieve ALL data
sv.sampleStep.set(1);                       %Set the sample step to 1 - unnecessary with useSampleLimits set to false

%% Upload parameters
sv.dacSetup;                                %Enable the DAC output and set encoding to offset-binary
sv.upload;                                  %Upload all parameters
sv.reset;                                   %Reset the sample generator memory pointer
