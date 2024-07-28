function [PCs, PDRs, Ds, Dvs, Cost] = CostFunctionGWO(Route)
    global Options;
    
    n = numel(Route) - 1;
    
    PCs = zeros(1, n);
    PDRs = zeros(1, n);
    Ds = zeros(1, n);
    Dvs = zeros(1, n);
    
    PC = 1;
    PDR = 1;
    D = 0;
    Dv = 0;
    
    for i = 1:n
        current = Route(i);
        next = Route(i+1);
        
        % Ensure indices are valid and within bounds
        if current <= size(Options.Connectivity, 1) && next <= size(Options.Connectivity, 2)
            PCs(i) = Options.Connectivity(current, next);
            PDRs(i) = Options.Reliability(current, next);
            Ds(i) = Options.Delay(current, next);
        else
            % Handle invalid indices if necessary
            PCs(i) = 0;
            PDRs(i) = 0;
            Ds(i) = 0;
        end
        
        PC = PC * PCs(i);
        PDR = PDR * PDRs(i);
        D = D + Ds(i);
        Dvs(i) = Ds(i) / numel(Route);
        Dv = Dv + Dvs(i);
    end
    
    % Normalizing Dv to avoid division by zero
    if Dv == 0
        Dv = 1e-6;
    end
    
    % Calculate Cost
    Cost = (0.2 * PC) + (0.2 * PDR) + (0.6 * ((80 - D) / 80) * (1 / (1 + Dv)));
end
