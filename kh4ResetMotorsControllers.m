function success = kh4ResetMotorsControllers
%--------------------------------------------------------------------------
%Reset the motors controllers
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,'M');
value = fscanf(serialPort);
if strcmp(value(1,1),'m') == 1
    success = 1;
else
    success = 0;
end
fclose(serialPort);
end