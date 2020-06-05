%programServo_LinSamples_FixedGain Example file that shows how to use the SERVO and SERVOCMD
%methods to program a particular sequence into the servo controller.
%
%   This file demonstrates how to use linear ramps with fixed gains.
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
% dacRef and dacMode have to be set at the start since othre properties
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
sv.sampleSource.set('int');             %Set the source of new samples to 'internal', i.e. the linear ramp generator    
sv.updateTime.set(32e-6);               %Set the sample update time to 32 us

%sv.numSamples is set after the linear ramps are calculated

%% Linear ramp settings
%
% These ramp settings are for a voltage that starts out at 0 V for 50 ms,
% ramps up to 1 V in 100 ms, holds at 1 V for 500 ms, ramps down to 0 V in
% 50 ms, and then stays at 0 V for 50 ms.
%
times = [50,100,500,100,50];            %Set the durations of each piece-wise section of the linear ramps in ms.                           
voltages = [0,0,1,1,0,0];               %Set the voltages of the linear ramps -
%
% Comment out one of the following lines to see how sample codes work.  If
% all the codes are set to 1, then the PID process is DISABLED and the DAC
% output is directly controlled by the linear ramps.  If all the codes are
% set to 0, then the PID process is ENABLED and the DAC output is
% calculated by the PID process based on the linear ramps and the current
% ADC voltage.
%
codes = ones(1,6);                      %Set all codes to 1, which means there the PID process is DISABLED and the DAC output is controlled directly by linear ramps
% codes = zeros(1,6);                     %Set all codes to 0, which means that the PID process is ENABLED and the DAC output is calculated via the PID process

%
% The following block of code calculates the necessary integer values for
% the linear ramp parameters based on the voltage code.
%
bndyTimes = [0,cumsum(times)]*1e-3/sv.updateTime.get;                       %Get the boundary times
voltageCode = zeros(size(voltages));
for nn=1:numel(voltages)
    if codes(nn)==0
        voltageCode(nn) = sv.adcConvert(voltages(nn),'code');               %If the code is 0, calculate the code using adcConvert
    elseif codes(nn)==1
        voltageCode(nn) = sv.dacConvert(voltages(nn),sv.dacMode,sv.dacRef); %If the code is 1, calculate the code using dacConvert
    end
end
voltageRates = round(diff(voltageCode)./diff(bndyTimes));                   %Calculate the appropriate increment for the linear ramps

sv.rampValues.set(voltageCode);             %Set the starting ramp values
sv.rampBndyTimes.set(bndyTimes);            %Set the ramp boundary times
sv.rampRates.set(voltageRates);             %Set the ramp increments
sv.rampCodes.set(codes);                    %Set the ramp codes
sv.numSamples.set(bndyTimes(end));          %Set the total number of samples to output


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
sv.useFixed.set(1);                         %Use fixed gains only.  This is the only allowed setting if linear ramps are used

%% Memory settings
sv.useSampleLimits.set(false);              %Disables sample retrieval limits - when data is read back, will retrieve ALL data
sv.sampleStep.set(1);                       %Set the sample step to 1 - unnecessary with useSampleLimits set to false

%% Upload parameters
sv.dacSetup;                                %Enable the DAC output and set encoding to offset-binary
sv.upload;                                  %Upload all parameters
sv.reset;                                   %Reset the sample generator memory pointer


