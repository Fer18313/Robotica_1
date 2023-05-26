function [q] = res2(q)
q = wrapTo180(q);
q = max(-160,q);
q = min(160,q);
end

