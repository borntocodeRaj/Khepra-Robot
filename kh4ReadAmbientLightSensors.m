function  [Values_sensors]= kh4ReadAmbientLightSensors
%--------------------------------------------------------------------------
% Read the 10 bit (0 to 1024) brightness value of each infra-red sensors.
% Note: A value of 0 means that the sensors is saturated with IR light, and a big
% value means that there�s no IR light source in front the sensor. IR light comes from
% an incandescent light, like the Sun, a fire�
% Values_sensors=[ Back_Left, Left, Front_Left, Front, Front_Right, Right,
% Back_right, Back, Ground_Left, Ground_Front_Left,Ground_Front_Right,Ground_Right]
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'O');
data=fscanf(serialPort);
Values_sensors=str2num(data(3:end));
fclose(serialPort);
end