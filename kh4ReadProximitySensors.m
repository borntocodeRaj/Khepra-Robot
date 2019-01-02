function  [Values_sensors]= kh4ReadProximitySensors
%--------------------------------------------------------------------------
% Read the 10 bit (0 to 1024) proximity value of each infra-red sensors.
% Note: The smaller the value, the further the object is from it. A value of
% 900 (i.e.) means that an obstacle is very close from the sensor.
% Incandescent IR sources (the Sun,…) can perturb the sensors.
% Values_sensors=[ Back_Left, Left, Front_Left, Front, Front_Right, Right,
% Back_right, Back, Ground_Left, Ground_Front_Left,Ground_Front_Right,Ground_Right]
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'N');
data=fscanf(serialPort);
Values_sensors=str2num(data(3:end));
fclose(serialPort);
end