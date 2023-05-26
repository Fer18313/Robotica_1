function [q] = restricciones(q)
q = wrapTo180(q);
q(6) = q(6)-90; %añadir un offset al efector 
q = max(-160,q);
q = min(160,q);
end

