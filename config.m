function cfg = config()
    %% Plant
    cfg.plant.C = 90;            
    cfg.plant.T = 90;          
    cfg.plant.lostTime = 10;      

    cfg.plant.S = [1800; 2000];   
    cfg.plant.gMin = [10; 10];    
    cfg.plant.gMax = [70; 70];   

    %% Initial state and demands
    cfg.initial.x = [30; 45];     
    cfg.demand.rate = [700; 800]; % [veh/h]

    %% Fixed-time comparison case
    cfg.compare.fixedGreen = [40; 40];

    %% LQ parameters & matrices setup (Diakaki)
    cfg.nz = 2;                   % Number of links
    cfg.xMax = [150; 150];        % Maximum link storage capacity 
    cfg.r = 0.05;                 % Control effort weight in R = r*I 

    % Input matrix B
    T_hr = cfg.plant.T / 3600;
    cfg.Bv = T_hr * [-cfg.plant.S(1)/cfg.plant.C; ...
                      cfg.plant.S(2)/cfg.plant.C];

    % --- LQ design matrices ---
    cfg.lq.A = eye(cfg.nz);                 % A = I 
    cfg.lq.B = cfg.Bv;                      % B
    cfg.lq.Q = diag(1 ./ cfg.xMax);         % Q
    cfg.lq.R = cfg.r * eye(size(cfg.lq.B,2)); % R
    
    % Riccati iteration settings (backward iteration method - Appendix A.2)
    cfg.lq.K_horizon = 200;
    cfg.lq.tol = 1e-10;
    cfg.lq.maxOuterIters = 20;

    %% Simulation
    cfg.sim.duration = 3600;      

    %% Validation
    p = cfg.plant;
    assert(abs(p.T-p.C) < 1e-9, 'This model requires T = C.');
    assert(abs(sum(cfg.compare.fixedGreen)+p.lostTime-p.C) < 1e-9, ...
        'Fixed-time greens must sum to C - lostTime.');
    assert(all(cfg.xMax > 0), 'xMax must be positive.');
    assert(cfg.r > 0, 'LQ weight r must be positive.');
end
