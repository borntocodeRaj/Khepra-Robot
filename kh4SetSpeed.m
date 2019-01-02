function success = kh4SetSpeed(speed_motor_left,speed_motor_right)
%--------------------------------------------------------------------------
% Set the speed of the both motors with PID(without profile). 0 will stop
% the engine. Max forward speed is 1200 and max backward speed -1200.
% and profile to reach the goal.
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fprintf(serialPort,sprintf('D,%d,%d',round(speed_motor_left),round(speed_motor_right)));
value = fscanf(serialPort);
if strcmp(value(1,1),'d') == 1
    success = 1;
else
    success = 0;
end
end