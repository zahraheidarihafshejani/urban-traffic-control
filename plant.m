function [xNext, info] = plant(x, g, d, p)

    x = x(:);
    g = g(:);
    d = d(:);

    S = p.S(:);
    gMin = p.gMin(:);
    gMax = p.gMax(:);

    tol = 1e-8;

    assert(abs(p.T - p.C) <= tol, ...
        'plant_step requires T = C.');

    assert(all(g >= gMin - tol) && all(g <= gMax + tol), ...
        'Green times violate their bounds.');

    assert(abs(sum(g) + p.lostTime - p.C) <= tol, ...
        'Green times must satisfy sum(g) + lostTime = C.');

    % Arrivals
    arrivals = d * (p.T / 3600);

    % Maximum departures
    departureCapacity = ...
        S .* (g / p.C) * (p.T / 3600);

    % Actual departures 
    availableVehicles = x + arrivals;

    departures = min(departureCapacity, availableVehicles);   

    % Vehicle conservation
    xMax = [150; 150]; 
    
    % If vehicles exceed xMax, excess vehicles cannot enter (or are blocked)
    xNext_unconstrained = availableVehicles - departures;
    xNext = min(xNext_unconstrained, xMax);

    %% Diagnostics
    info.arrivals = arrivals;
    info.departureCapacity = departureCapacity;
    info.departures = departures;

    % Average actual departure rate 
    info.outflowRate = departures * (3600 / p.T);

    % Unused service capacity 
    info.unusedCapacity = departureCapacity - departures;

    % Should be zero up to numerical precision
    info.massBalanceResidual = ...
        xNext - x - arrivals + departures;

end
