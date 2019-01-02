function success = kh4SetPosition(motor_left,motor_right)

% Set the position goal in encoder values. 
% The controller will use the PID and profile to reach the goal.
% Wheel diameter: 42mm
% Revolution resolution: 19456 [pulses]
% Wheel in pulses (19456 pulse = 147.453 mm).

load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,sprintf('F,%d,%d',round(motor_left),round(motor_right)));
value = fscanf(serialPort);
if strcmp(value(1,1),'f') == 1
    success = 1;
else
    fclose(serialPort);
    error('erreur de COM')
end
fclose(serialPort);
end