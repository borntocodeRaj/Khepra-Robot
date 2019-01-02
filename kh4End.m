function kh4End
fclose(instrfind)
delete(instrfind)
disp('Khepera déconnecté')
end