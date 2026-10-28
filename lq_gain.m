function [K, info] = lq_gain(cfg)


    A = cfg.lq.A;
    B = cfg.lq.B;
    Q = cfg.lq.Q;
    R = cfg.lq.R;

    n = size(A, 1);
    K_horizon = cfg.lq.K_horizon;
    tol = cfg.lq.tol;
    maxOuterIters = cfg.lq.maxOuterIters;

    K_prev = nan(size(B, 2), n);
    converged = false;
    Pk = eye(n);

    for outer = 1:maxOuterIters
        Pnext = eye(n);
        K_hist = zeros(size(B, 2), n, K_horizon);

        for k = K_horizon:-1:1
            G = R + (B' * Pnext * B);
            G = (G + G') / 2;

            Kk = G \ (B' * Pnext * A);
            Pk = Q + (A' * Pnext * A) - (A' * Pnext * B * Kk);
            Pk = (Pk + Pk') / 2;

            K_hist(:,:,k) = Kk;
            Pnext = Pk;
        end

        Kcand = K_hist(:,:,1);

        if ~any(isnan(K_prev(:)))
            if norm(Kcand - K_prev, 'fro') <= tol * max(1, norm(K_prev, 'fro'))
                converged = true;
                K = Kcand;
                break;
            end
        end

        K_prev = Kcand;
        K_horizon = round(1.5 * K_horizon);
    end

    if ~converged
        K = Kcand;
    end

    p = cfg.plant;
    effectiveCycle = p.C - p.lostTime;
    d = cfg.demand.rate(:);
    S = p.S(:);
    
    % Nominal green splits
    gNom = effectiveCycle * (d ./ S) / sum(d ./ S);
    xNom = [0; 0]; % Target zero 

    % --- Output info structure ---
    info = struct();
    info.method = "backward_riccati_diakaki";
    info.converged = converged;
    info.final_horizon = K_horizon;
    info.tol = tol;
    info.outer_iters = outer;
    info.P = Pk;
    info.Q = Q;
    info.R = R;
    info.gNom = gNom;   
    info.xNom = xNom;  
end
