function [ steps ] = GetMoveSteps( startp,endp )
%GETMOVESTEPS Summary of this function goes here
%   Detailed explanation goes here

m=(endp(2)-startp(2))/(endp(1)-startp(1)); %gradient

if m==Inf %vertical line
    pts = abs(startp(2)-endp(2));
    xx(1:pts)=startp(1);
    yy(1:pts)=linspace(startp(2),endp(2),pts);
elseif m==0 %horizontal line
    pts = abs(startp(1)-endp(1));
    xx(1:pts)=linspace(startp(1),endp(1),pts);
    yy(1:pts)=startp(2);
else %if (endp(1)-startp(1))~=0
    pts = abs(startp(1)-endp(1));
    xx=linspace(startp(1),endp(1),pts);
    yy=m*(xx-startp(1))+startp(2);
end

steps = [xx;yy];
end

