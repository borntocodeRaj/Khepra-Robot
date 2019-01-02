function success = kh4ResetEncoders
%--------------------------------------------------------------------------
%Reset the values of the motors encoders
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'I');
value = fscanf(serialPort);
if strcmp(value(1,1),'i') == 1
    success = 1;
else
    success = 0;
end
fclose(serialPort);
end