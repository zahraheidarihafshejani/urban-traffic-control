clear;
clc;
close all;

cfg = config();
p = cfg.plant;

%% Simulation duration
% 90 minutes 
cfg.sim.duration = 90 * 60;  % [s]

assert(abs(cfg.sim.duration / p.T - round(cfg.sim.duration / p.T)) < 1e-10, ...
    'Simulation duration must be an integer multiple of p.T.');

nSteps = round(cfg.sim.duration / p.T);

timeState = (0:nSteps) * p.T / 60;       
timeInput = (0:nSteps-1) * p.T / 60;      

%% Time-varying demand profile
dBase = cfg.demand.rate(:);

%  more demand is directed to link 1.
surgeStart = 10;  
surgeEnd   = 30;   
dSurge     = [1000; 600]; 

dProfile = repmat(dBase, 1, nSteps);
surgeIdx = (timeInput >= surgeStart) & (timeInput < surgeEnd);
dProfile(:, surgeIdx) = repmat(dSurge, 1, sum(surgeIdx));

%% Compute the LQ gain once
[lq.K, lqInfo] = lq_gain(cfg);

lq.P = lqInfo.P;
lq.Q = lqInfo.Q;
lq.R = lqInfo.R;
lq.gNom = lqInfo.gNom;
lq.xNom = lqInfo.xNom;

fprintf('LQ nominal green: [%.3f, %.3f] s\n', ...
    lq.gNom(1), lq.gNom(2));
fprintf('LQ gain K: [%.6f, %.6f]\n', lq.K(1), lq.K(2));
fprintf('Demand surge: %.1f–%.1f min, d = [%.1f, %.1f] veh/h\n', ...
    surgeStart, surgeEnd, dSurge(1), dSurge(2));

%% Simulate both controllers
modes = {'fixed', 'lq'};
names = {'Fixed-time', 'LQ'};

for m = 1:numel(modes)

    x = cfg.initial.x(:);

    out(m).x = zeros(2, nSteps + 1);
    out(m).g = zeros(2, nSteps);
    out(m).demand = dProfile;
    out(m).arrivals = zeros(2, nSteps);
    out(m).departures = zeros(2, nSteps);
    out(m).outflowRate = zeros(2, nSteps);
    out(m).massBalanceResidual = zeros(2, nSteps);

    out(m).x(:,1) = x;

    for k = 1:nSteps

        [g, ~] = controller(modes{m}, x, cfg, lq);

        dNow = dProfile(:,k);

        [xNext, plantInfo] = plant(x, g, dNow, p);

        out(m).g(:,k) = g(:);
        out(m).arrivals(:,k) = plantInfo.arrivals;
        out(m).departures(:,k) = plantInfo.departures;
        out(m).outflowRate(:,k) = plantInfo.outflowRate;
        out(m).massBalanceResidual(:,k) = ...
            plantInfo.massBalanceResidual;
        out(m).x(:,k+1) = xNext;

        x = xNext;
    end

    %% Performance measures
    dtHours = p.T / 3600;
    
    % Integral of link accumulation over time [veh*h].
    accumulationArea = ...
        0.5 * (out(m).x(:,1:end-1) + out(m).x(:,2:end)) * dtHours;

    out(m).accumulationVehHoursByLink = sum(accumulationArea, 2);
    out(m).accumulationVehHours = ...
        sum(out(m).accumulationVehHoursByLink);

    out(m).maxQueueByLink = max(out(m).x, [], 2);
    out(m).maxTotalVehicles = max(sum(out(m).x, 1));

    out(m).totalArrivals = sum(out(m).arrivals, 2);
    out(m).totalDepartures = sum(out(m).departures, 2);

    out(m).greenTotalVariation = ...
        sum(sum(abs(diff(out(m).g, 1, 2))));

    fprintf('\n%s results:\n', names{m});
    fprintf('  Final vehicles:             [%.3f, %.3f]\n', ...
        out(m).x(1,end), out(m).x(2,end));
    fprintf('  Maximum vehicles by link:   [%.3f, %.3f]\n', ...
        out(m).maxQueueByLink(1), out(m).maxQueueByLink(2));
    fprintf('  Maximum total vehicles:     %.3f\n', ...
        out(m).maxTotalVehicles);
    fprintf('  Accumulation area:          %.3f veh*h\n', ...
        out(m).accumulationVehHours);
    fprintf('  Total departures:           [%.3f, %.3f]\n', ...
        out(m).totalDepartures(1), out(m).totalDepartures(2));
    fprintf('  Mean green:                 [%.3f, %.3f] s\n', ...
        mean(out(m).g(1,:)), mean(out(m).g(2,:)));
    fprintf('  Green total variation:      %.3f s\n', ...
        out(m).greenTotalVariation);
end

%% Plot: states, green commands, and demand/outflow
figure('Color','w','Name','Fixed-time vs LQ: demand surge');
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

for m = 1:numel(modes)

    % Link accumulations / queues
    nexttile(m);
    plot(timeState, out(m).x(1,:), 'LineWidth', 1.5);
    hold on;
    plot(timeState, out(m).x(2,:), 'LineWidth', 1.5);
    xline(surgeStart, '--k', 'Surge start');
    xline(surgeEnd, '--k', 'Surge end');
    grid on;
    xlabel('Time [min]');
    ylabel('Vehicles');
    title([names{m}, ' - states']);
    legend('Link 1','Link 2','Location','best');

    % Green commands
    nexttile(m + 2);
    stairs(timeInput, out(m).g(1,:), 'LineWidth', 1.5);
    hold on;
    stairs(timeInput, out(m).g(2,:), 'LineWidth', 1.5);
    xline(surgeStart, '--k');
    xline(surgeEnd, '--k');
    grid on;
    xlabel('Time [min]');
    ylabel('Green [s]');
    title([names{m}, ' - green commands']);
    legend('Phase 1','Phase 2','Location','best');

    % Demand and actual outflow
    nexttile(m + 4);
    stairs(timeInput, out(m).demand(1,:), '--', 'LineWidth', 1.2);
    hold on;
    plot(timeInput, out(m).outflowRate(1,:), 'LineWidth', 1.5);
    stairs(timeInput, out(m).demand(2,:), '--', 'LineWidth', 1.2);
    plot(timeInput, out(m).outflowRate(2,:), 'LineWidth', 1.5);
    xline(surgeStart, '--k');
    xline(surgeEnd, '--k');
    grid on;
    xlabel('Time [min]');
    ylabel('Flow [veh/h]');
    title([names{m}, ' - demand and outflow']);
    legend('Demand 1','Outflow 1','Demand 2','Outflow 2', ...
        'Location','best');
end
