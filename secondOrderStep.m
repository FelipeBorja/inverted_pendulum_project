function [ p1, p2, respInfo ] = secondOrderStep( wn, z, plotStep )
%secondOrderStep returns the two poles and the results of stepinfo() given
%the natural frequency (rad/s) and damping ratio (zeta) of a canonical
%second order system
%   plotStep = 1 also plots the response of this system (else = no plot)

    %define the system
    s = tf('s');
    num = wn^2;
    denom = s^2 + 2*z*wn*s + wn^2;
    tcl = num/denom;

    if plotStep == 1
        figure
        clf
        [y, t] = step(tcl);
        plot(t,y)
        xlabel('Time (s)')
        ylabel('Unit Step Response')
        title(sprintf('Step Response with wn = %.2f, zeta = %.2f]',... 
            wn, z))
        grid on
    end
    respInfo = stepinfo(tcl, 'SettlingTimeThreshold', 0.001);

    %find the pole values, using quadratic formula
    a = 1;
    b = 2*z*wn;
    c = wn^2;
    p1 = (-b + sqrt(b^2 - 4*a*c)) / (2*a);
    p2 = (-b - sqrt(b^2 - 4*a*c)) / (2*a);

end