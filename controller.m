function [g, info] = controller(mode, x, cfg, lq)

    p = cfg.plant;
    effectiveCycle = p.C - p.lostTime;

    switch lower(mode)
        case 'fixed'
            g = cfg.compare.fixedGreen(:);
            info.unconstrained_g = g;

        case 'lq'

            gNom = lq.gNom(:);
            xNom = lq.xNom(:);

            % State 
            dx = x(:) - xNom;

            % LQ control law: g = gNom - K * dx
            

            if size(lq.K, 1) == 1
                u_adj = lq.K * dx; 
                g_calc = gNom + [-u_adj; u_adj];
            else
                % If K is a matrix
                g_calc = gNom - lq.K * dx;
            end

            % --- Constraint enforcement 
            g = enforce_constraints(g_calc, p.gMin, p.gMax, effectiveCycle);
            
            info.unconstrained_g = g_calc;
            info.gNom = gNom;
            info.dx = dx;

        otherwise
            error('Unknown control mode: %s', mode);
    end
end

function g_out = enforce_constraints(G, g_min, g_max, C_eff)

    n = length(G);
    g = G(:);
    g_min = g_min(:);
    g_max = g_max(:);
    
    H = (1:n)';
    
    for iter = 1:20
        sum_g_H = sum(g(H));
        fixed_idx = setdiff((1:n)', H);
        rem_C = C_eff - sum(g(fixed_idx));
        
        delta = (rem_C - sum_g_H) / length(H);
        g(H) = g(H) + delta;
        
        % Check for violations
        too_high = H(g(H) > g_max(H) + 1e-9);
        too_low  = H(g(H) < g_min(H) - 1e-9);
        
        if isempty(too_high) && isempty(too_low)
            break;
        end
        
        % Fix violating phases 
        if ~isempty(too_high)
            g(too_high) = g_max(too_high);
            H = setdiff(H, too_high);
        end
        if ~isempty(too_low)
            g(too_low) = g_min(too_low);
            H = setdiff(H, too_low);
        end
        
        if isempty(H)
            break;
        end
    end
    % Safety saturation
    g_out = max(g_min, min(g_max, g));
    % Final normalization to ensure exact cycle sum 
    g_out = g_out * (C_eff / sum(g_out));
end
