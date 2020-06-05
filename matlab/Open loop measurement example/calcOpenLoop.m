function varargout=calcOpenLoop
%CALCOPENLOOP Calculates the open loop response of a linear system given
%data with the modFreq, measured voltages, and times.

addpath('..');

%
% Load the open loop data recorded using measureOpenLoopLinear
%
M = load('Open loop response');

%
% Set up the curve fitting function
%
options = optimset('display','off','TolFun',1e-10);
lb = [0,-3*pi,0];
ub = [100,3*pi,3];
guess = [1,0,0];

pMod = zeros(numel(guess),numel(M.modFreq));

%
% Start and end points for the curve fit.
%
tStart = 0.25;
tEnd = 0.45;

%
% Run over all recorded frequencies and fit to a sin curve
%
for nn=1:numel(M.modFreq)
    func = @(c,x) M.Vmod*c(1)*sin(2*pi*(M.modFreq(nn))*(x-M.tRamp)+c(2))+c(3);
    idx = M.data(nn).t>=tStart & M.data(nn).t<=tEnd;
    tmp = lsqcurvefit(func,guess,M.data(nn).t(idx),M.data(nn).v(idx),lb,ub,options);
    pMod(:,nn) = tmp(:);
    figure(1);clf;
    plot(M.data(nn).t(idx),M.data(nn).v(idx),'b-');
    hold on;
    plot(M.data(nn).t(idx),func(pMod(:,nn),M.data(nn).t(idx)),'r--');
    drawnow;
end

%
% Model the known Low-Latency filter with 512x oversampling ratio.
%
OSR = 512/32;
CLK = 16e6;
tLL = 32*OSR/CLK;
Gadc = @(x) exp(-1i*2*pi*x*tLL).*abs((sin(32*pi*x/CLK)./(32*sin(pi*x/CLK))).^5.*(sin(32*OSR*pi*x/CLK)./(OSR*sin(32*pi*x/CLK))));

%
% Model the RC filter + buffer as a second order system.  Include the
% measurement filter to get the actual RC filter dynamics.
%
G = @(c,x) c(1)./(1+1i*x./c(2)-(x/c(3)).^2).*Gadc(x);
lb = [0,0,0];
ub = [10,2e4,1e6];
guess = [1,400,1e3];

%
% Get the filter parameters
%
pFilter = lsqcurvefit(@(c,x) abs(G(c,x)),guess,M.modFreq(:),pMod(1,:)',lb,ub,options);

%
% Plot the results
%
figure(2);clf;


subplot(2,1,1);
loglog(M.modFreq,pMod(1,:),'o');
hold on
freqInterp = logspace(log10(min(M.modFreq)),log10(max(M.modFreq)),1e3)';
loglog(freqInterp,abs(G(pFilter,freqInterp)),'r--');
xlabel('Frequency [Hz]');
ylabel('AC Gain');

subplot(2,1,2);
semilogx(M.modFreq,unwrap(pMod(2,:))*180/pi,'o');
hold on
semilogx(freqInterp,unwrap(angle(G(pFilter,freqInterp)))*180/pi,'r--');
xlabel('Frequency [Hz]');
ylabel('Phase shift [\circ]');

G0 = pFilter(1);
w1 = pFilter(2);
w2 = pFilter(3);
save('Open loop system parameters','G0','w1','w2');


if nargout > 0
    varargout{1} = pMod;
    varargout{2} = pFilter;
end

rmpath('..');