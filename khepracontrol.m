%---------- Exemple------------%
%Définissez le port du khepera à travers laquelle le ComId communique
clear all
clc
%kh4Start('COM6'); %à faire une seule fois au démarrage du robot

posg=160;
posd=160;

vitg=500;
vitd=500;

P = 10;
I = 5;
D = 10;

kh4ResetEncoders;

kh4ConfigurePID(P,I,D)
success=kh4SetPosition(posg,posd); %POSITION

%prise de mesure
SamplingTime = 1.085; % temps de mesure en s
k=0;
tStart = tic;
T = toc(tStart); % demarrage chrono
load('SerialKhepera.mat','serialPort')
fopen(serialPort);
success=kh4SetSpeed(vitg,vitd); %VITESSE

while T< SamplingTime
    % t actuel
    k=k+1;
    
[lm(k), rm(k)] = kh4ReadEncoders; %POSITION
%[lspeed(k), rspeed(k)] = kh4ReadSpeed; %VITESSE
 T = toc(tStart); %mesure chrono actuel
 time(k)=T
end

kx=1.52; ky=6;c=0;        %gains vitesse
xG=-200;yG=0; da=5;dm=5; cn=1; %centimeter
xo=1;phi=1;yo=1 ;th=40; outputs(1,1)=0; outputs(2,1)=0;
        
while (abs(xo)>0.5||abs(yo)>0.5||abs(phi)>0.5)&& (Values_sensors ~= 1)
success=US_values;
[xo,yo] = kh4ReadEncoders;
end
xr=0; yr=0; angle_v=0; delta=0.3;and_method=1; xo=0;phi=0;yo=0; 
j=1; 
while (kh4ReadProximitySensors ~= 1)

success=kh4SetSpeed(0,0);
fclose(serialPort);

end
%graphes
 xr=0; yr=0; angle_v=0; delta=0.3;and_method=1; xo=0;yo=0; 
        j=1; 
  d0=kh4SetPosition(0,0);
  d1=kh4SetPosition(0,1);
  d2=kh4SetPosition(1,0);
  d3=kh4SetPosition(1,1);
            
  xg=(xG-xr)*cosd(0)+(yG-yr)*sin(0);
            yg=(yG-yr)*cosd(0)+(xG-xr)*sin(0);             
            % APF
             %attratif
             xp=sqrt(xg^2+yg^2);
            fax=-1*(-xg);
            fay=-0.2*(-yg); % 0.2
            if xg<70
                z=4;
                fax=-1*z*(-xg);
                fay=-0.2*z*(-yg);
            end
            % target circle
            if xp<=25
                d0=89;d1=89;d2=89;d3=89;d4=89;
                d5=89;d6=89;d7=89;d8=89;
            end    
            
            d0x=-(1/d0-1/70)*(1/d0^2)*(cosd(0*th));
            d0y=-(1/d0-1/70)*(1/d0^2)*(sind(0*th));
            d1x=-(1/d1-1/70)*(1/d1^2)*(cosd(1*th));
            d1y=-(1/d1-1/70)*(1/d1^2)*(sind(1*th));
            d2x=-(1/d2-1/70)*(1/d2^2)*(cosd(2*th));
            d2y=-(1/d2-1/70)*(1/d2^2)*(sind(2*th));
            d3x=-(1/d3-1/70)*(1/d3^2)*(cosd(3*th));
            d3y=-(1/d3-1/70)*(1/d3^2)*(sind(3*th));
          
          
            % resultante
            fx=fax+13640*(d0x+d1x+d2x+d3x);
            fy=fay+0.4*13640*(d0y+d1y+d2y+d3y); %0.4 bon pour 1.
            
           
           inputs(j,:)=[dl,fl,df,fr dr,xg,fx,fy];
           cn=1;
            if (outputs((j-1),1)<-5) && (0<df<40 || 0<fl<40 || 0<fr<40)
                df=5; fl=5; fr=5; kx=2; cn=cn+1; 
            end
            s=1;
            if cn>=3 && cdc<=50
              df=10; fl=10; fr=10;
               s=s+1;
               if dl>60 && s==2
                   c=1;
               else
                   c=-1;
               end
               if c==1
                  fy=25; 
               else
                   fy=-25;                  
               end
            end
            if cdc>51
                cn=1;
                kx=1;
                c=0;
            end          
           
                      % APF controller
                      
        outputs(j,1)=kx*vx; outputs(j,2)=ky*vy; 
                
close all
plot(time,posg,'g')
plot(time,lm); %POSITION
hold on;
plot(time,rm,'r'); %POSITION

%plot(time,lspeed,'g'); %VITESSE
%plot(time,rspeed,'c'); %VITESSE

legend('Position motor left','Position motor right')
xlabel('Time (unit)')
ylabel('Encoders (pulses)')
title('title')

    