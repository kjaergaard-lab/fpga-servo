function T = PIDsim(f,Ts,openLoop,gain,Mfunc)
%PIDSIM Calculates the closed loop response function for the servo
%controller given the sample period, open loop response, set gain
%parameters, and measurement function
%% Constants
OSR = 512/32;
CLK = 16e6;

%% Calculations
w = 2*pi*f;
zinv = exp(-1i*w*Ts);

%
% Calculate measurement response
%
if nargin>=5
    M = Mfunc(f);
else
    M = exp(-1i*w*Ts).*abs((sin(32*pi*f/CLK)./(32*sin(pi*f/CLK))).^5.*(sin(32*OSR*pi*f/CLK)./(OSR*sin(32*pi*f/CLK))));
end

%
% Include SPI lag (ADC + DAC), PID lag, and DAC slew time
%
Td = 6.0e-6;
H = exp(-1i*w*Td);

%
% Calculate the open loop response
%
G = openLoop.G0./(1+1i*w/(2*pi*openLoop.w1)-(w./(2*pi*openLoop.w2)).^2);

%
% Calculate the ideal servo response using the z-transform
%
scale = 2^4*10/5;   %Valid for unipolar DAC.  Change 10 to 20 for bipolar DAC settings
B = [gain.Kp+gain.Ki/2+gain.Kd,-gain.Kp+gain.Ki/2-2*gain.Kd,gain.Kd]/2^gain.N*scale;
K = (B(1)+B(2)*zinv+B(3)*zinv.^2)./(1-zinv);

%
% Calculate response
%
L = K.*G.*M.*H;
T = L./(1+L);



end