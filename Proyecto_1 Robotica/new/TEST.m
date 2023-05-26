
for coeff = 0:0.01:1
    q = slerp(qs,qf,coeff);
    position = ps + (pf - ps)*coeff;
    set(patch,Orientation=q,Position=position); 
    drawnow
end

robotat= robotat_connect('192.168.50.200');
robotat_disconnect(robotat)
