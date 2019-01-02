function success = kh4SetRGBLeds(lr,lg,lb,rr,rg,rb,br,bg,bb)
%--------------------------------------------------------------------------
%Set the 3 color leds. Values are in range[0-63]. First letter is the
%position : l= left, r= right and b=back. Second letter is the color:
%r=red, g=green and b= blue.
%--------------------------------------------------------------------------
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
fprintf(serialPort,sprintf('K,%d,%d,%d,%d,%d,%d,%d,%d,%d',...
                           round(lr),round(lg),round(lb),...
                           round(rr),round(rg),round(rb),...
                           round(br),round(bg),round(bb)));
value = fscanf(serialPort);
if strcmp(value(1,1),'k') == 1
    success = 1;
else
    success = 0;
end
fclose(serialPort);
end