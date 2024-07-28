function [D] = RndBtw(min,max,n)
D = min+rand(n,n)*(max-min);
D = D+D.';
D = D/2;
end

