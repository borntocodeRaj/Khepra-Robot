function US_values = kh4GetUSValues

% Get the values of the Ultrasonic (US) values. 
% 1000, means nothing detected in range, 0 means something under 25 cm, between 25 and 250
% means the distance of an object in [cm]
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'G');
data=fscanf(serialPort);
US_values=str2num(data(3:end));
fclose(serialPort);
end