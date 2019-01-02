function Acc = kh4Accelerometer
% Read accelerometer values, new data coming first.
% The accelerometer returns 12-bit data (two's complement) with a range of
% +/-2g. This means a value of 1g will return a value of 16384. The data rate
% is configured to 100Hz, as the dsPIC of the Khepera refreshes 10 values at
% a time, the user needs to read every 100ms (10Hz) to obtain fresh data.
% Format of the response :
% acc=[X1,X2,...,X10,Y1,Y2,...,Y10,Z1,Z2,....,Z10]

load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'T');
data=fscanf(serialPort);
Acc=str2num(data(3:end));
fclose(serialPort);
end