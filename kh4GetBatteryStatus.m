function Status = kh4GetBatteryStatus (argument)
%--------------------------------------------------------------------------
% Get the battery status where argument is:
% 0 : voltage in [mV]
% 1 : current in [mA]
% 2 : average current in [mA]
% 3 : absolute remaining capacity in [mAh]
% 4 : battery temperature in [C]
% 5 : relative remaining capacity in [%]
% 6 : whether the charger is plugged (1), or not (0)
% 7 : the status number of the battery controller DS2781
%--------------------------------------------------------------------------

if argument<=7
    load('SerialKhepera.mat','serialPort')
    fopen(serialPort);
    fprintf(serialPort,sprintf('V,%d',argument));
    value = fscanf(serialPort);
    Status=str2num(value(3:end));
    if strcmp(value(1,1),'v') == 1
        
        switch argument
            case 0
                sprintf('Voltage : %d [mV]' ,Status)
            case 1
                sprintf('Current : %d [mA]' ,Status)
            case 2
                sprintf('Average current : %d [mA]' ,Status)
            case 3
                sprintf('Absolute remaining capacity : %d [mAh]' ,Status)
            case 4
                sprintf('Battery temperature : %d [C]' ,Status)
            case 5
                sprintf('Relative remaining capacity : %d [%%]' ,Status)
            case 6
                sprintf('Charger plugged (1) or not (0) : %d' ,Status)
            case 7
                sprintf('The status number of the battery controller DS2781 : %d' ,Status)
        end
    else
        disp('error')
    end
    fclose(serialPort);
else
    warning('The argument is incorrect');
end
end