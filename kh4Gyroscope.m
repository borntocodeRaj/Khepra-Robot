function Gyr = kh4Gyroscope
%--------------------------------------------------------------------------
% Read gyrosope values, new data coming first.
% Format of the response :
% Gyr=[X1,X2,...,X10,Y1,Y2,...,Y10,Z1,Z2,....,Z10]
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'S');
data=fscanf(serialPort);
Gyr=str2num(data(3:end));
fclose(serialPort);
end