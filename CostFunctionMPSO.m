function Costs=CostFunctionMPSO(route,D, C)

    n=numel(route);
  
    L=0;
    for i=1:n-1
        L=L+D(route(i),route(i+1));
    end
    
    CC = 0;
    for i=1:n
        for j=i+1:n
            if(C(route(i)) ~= C(route(j)))
                CC = CC + 1;
            end
        end
    end
    Costs = [n L CC];
end