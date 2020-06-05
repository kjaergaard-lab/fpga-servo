function varargout=calcClosedLoop
%CALCCLOSEDLOOP Calculates the closed loop response of a linear system given
%data with the modFreq, targetFreq, measured voltages, and times.

addpath('..');

%
% Load the closed loop data recorded using measureClosedLoopLinear
%
M = load('Closed loop response');
%
% Load open loop parameters
%
P = load('Open loop system parameters');

%
% Set up the curve fitting function
%
options = optimset('display','off','TolFun',1e-10);
lb = [0,-3*pi,0];
ub = [100,3*pi,3];
guess = [1,0,0];

pMod.amp = zeros(numel(M.modFreq),numel(M.targetFreq));
pMod.phase = zeros(numel(M.modFreq),numel(M.targetFreq));
pMod.offset = zeros(numel(M.modFreq),numel(M.targetFreq));

%
% Start and end points for the curve fit.
%
tStart = 0.35;
tEnd = 0.45;

%
% Run over all target frequencies
%
for mm = 1:numel(M.targetFreq)
    %
    % Run over all recorded frequencies and fit to a sin curve
    %
    for nn=1:numel(M.modFreq)
        func = @(c,x) M.Vmod*c(1)*sin(2*pi*(M.modFreq(nn))*(x-M.tRamp)+c(2))+c(3);
        idx = M.data(nn,mm).t>=tStart & M.data(nn,mm).t<=tEnd;
        tmp = lsqcurvefit(func,guess,M.data(nn,mm).t(idx),M.data(nn,mm).v(idx),lb,ub,options);
        pMod.amp(nn,mm) = tmp(1);
        pMod.phase(nn,mm) = tmp(2);
        pMod.offset(nn,mm) = tmp(3);

        figure(1);clf;
        plot(M.data(nn).t(idx),M.data(nn).v(idx),'b-');
        hold on;
        plot(M.data(nn,mm).t(idx),func(tmp,M.data(nn,mm).t(idx)),'r--');
        drawnow;
    end
end

%
% Model the known Low-Latency filter with 512x oversampling ratio.
%
OSR = 512/32;
CLK = 16e6;
Ts = 32*OSR/CLK;
Gadc = @(x) exp(-1i*2*pi*x*Ts).*(sin(32*pi*x/CLK)./(32*sin(pi*x/CLK))).^5.*(sin(32*OSR*pi*x/CLK)./(OSR*sin(32*pi*x/CLK)));

figure(2);clf;
for mm = 1:numel(M.targetFreq)
    %
    % Model the servo controller
    %
    freqInterp = logspace(log10(min(M.modFreq)),log10(max(M.modFreq)),1e3)';
    T = PIDsim(freqInterp,Ts,P,M.gain(mm),Gadc);

    %
    % Plot the results
    %
    subplot(2,1,1);
    h = loglog(M.modFreq,pMod.amp(:,mm),'o');
    hold on   
%     loglog(freqInterp,abs(G(pFilter(mm,:),freqInterp)),'--','color',h.Color);
    loglog(freqInterp,abs(T),'-','color',h.Color);
    xlabel('Frequency [Hz]');
    ylabel('AC Gain');

    subplot(2,1,2);
    h = semilogx(M.modFreq,unwrap(pMod.phase(:,mm))*180/pi,'o');
    hold on
%     semilogx(freqInterp,unwrap(angle(G(pFilter(mm,:),freqInterp)))*180/pi,'--','color',h.Color);
    semilogy(freqInterp,unwrap(angle(T))*180/pi,'-','color',h.Color);
    xlabel('Frequency [Hz]');
    ylabel('Phase shift [\circ]');
end

if nargout > 0
    varargout{1} = pMod;
end

rmpath('..');