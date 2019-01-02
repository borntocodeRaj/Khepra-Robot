function success = kh4SetSpeedOpenloopMode(left_motor,right_motor)
%--------------------------------------------------------------------------
% Set the speed of the motor in the open loop control mode. Maximum is
% +/-2940 which corresponds to 100% of PWM.
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,sprintf('L,%d,%d',round(left_motor), round(right_motor)));
value = fscanf(serialPort);
if strcmp(value(1,1),'l') == 1
    success = 1;
else
    success = 0;
end
fclose(serialPort);
end