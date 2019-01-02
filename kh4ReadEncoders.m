function [left_motor, right_motor] = kh4ReadEncoders
%--------------------------------------------------------------------------
% Read motor encoder position values. 
% Format of the response :
% Pos=[left_motor, right_motor]
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fprintf(serialPort,'R');
data=fscanf(serialPort)
Pos=str2num(data(3:end));
left_motor=Pos(1); right_motor=Pos(2);
end