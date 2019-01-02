function kh4Start(port)
%Définissez le port de communication avec le Khepera
 
clc;
try
    fclose(instrfind);
end
delete(instrfind);
disp('Démarrage en cours')
serialPort = serial(port, 'Baudrate', 115200);
save('SerialKhepera.mat','serialPort')
tStart = tic;
T=0;
prevP = 0;
disp('Progression:  0 %')
while (T<50)
    T = toc(tStart);
    P=floor(T/50*100);
    if P~=prevP
        a = sprintf('\b\b\b\b\b\b% 3d %%',P);
        disp(a)
    end
prevP = P;
end
fopen(serialPort);

%Envoi des commandes de demarrage
fprintf(serialPort,'root')
fprintf(serialPort,'./kh4server /dev/ttyS2');
pause(2);

%Desactive ultrasons (trop bruyants)
flushinput(serialPort);
fprintf(serialPort,'U,0');
value = fscanf(serialPort);
if strcmp(value(1,1),'u') == 1
    disp('Démarrage terminé - Khepera prêt')
else
    error('Problème lors du démarrage')
end

fclose(serialPort);
end