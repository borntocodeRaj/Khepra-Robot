function success = kh4ConfigureSpeedProfile(acc_inc, acc_div, min_speed_acc, min_speed_dec,max_speed)
%--------------------------------------------------------------------------
%Configure the speed profile.
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,sprintf('J,%d,%d,%d,%d,%d',round(acc_inc),round(acc_div)...
      , round(min_speed_acc), round(min_speed_dec),round(max_speed)));
value = fscanf(serialPort);
if strcmp(value(1,1),'j') == 1
    success = 1;
else
    success = 0;
end
fclose(serialPort);
end