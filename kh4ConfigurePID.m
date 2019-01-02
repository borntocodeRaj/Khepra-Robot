function success = kh4ConfigurePID(P,I,D)
%--------------------------------------------------------------------------
% Set the Proportional (P), Integral (I), and Derivative (D) of the PID
% controller
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,sprintf('H,%d,%d,%d',round(P),round(I),round(D)));
value = fscanf(serialPort);
if strcmp(value(1,1),'h') == 1
    success = 1;
else
    success = 0;
end
fclose(serialPort);
end