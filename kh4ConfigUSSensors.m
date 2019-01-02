function success = kh4ConfigUSSensors (config)
%--------------------------------------------------------------------------
% Configure which US sensor is activated. Left=1, front left=2,
% front=4, front right=8, right=16, all=31, none=0. They can
% be combined by addition: for example, left+right => 17
%--------------------------------------------------------------------------

if config<=31
    load('SerialKhepera.mat','serialPort')
    fopen(serialPort);
    fprintf(serialPort,sprintf('U,%d',config));
    value = fscanf(serialPort);
    if strcmp(value(1,1),'u') == 1
        success = 1;
    else
        success = 0;
    end
    fclose(serialPort);
else
    disp('The value is incorrect');
end
end