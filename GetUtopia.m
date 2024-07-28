function UtopiaIndex = GetUtopia(Fit)

Utopia = [min(Fit(:,1)) min(Fit(:,2)) min(Fit(:,3))];
d = pdist2(Fit, Utopia);
[~,UtopiaIndex] = min(d);
end
