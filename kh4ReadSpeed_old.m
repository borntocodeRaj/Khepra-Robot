function [speed_motor_left, speed_motor_right] = kh4ReadSpeed_old
%--------------------------------------------------------------------------
% Read the speed of the motors encoder position values. 
% Format of the response :
% [speed_motor_left, speed_motor_right]
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'E');
data=fscanf(serialPort);
Speed=str2num(data(3:end));
speed_motor_left=Speed(1); speed_motor_right=Speed(2);
fclose(serialPort);
end

