
%% FLOOD RESCUE ROBOT - ALGORITHM COMPARISON
% A*, ARA*, RRT*
%
% Figure 1: Six performance comparison bar charts
% Figure 2: Dynamic simulation comparison
%
% Reads existing JSON and MAT files from web_dashboard.
% Does not save CSV, PNG, FIG, or other output files.

clc;
close all;
clearvars;

%% =========================================================
% 1. LOAD EXISTING FILES
% ==========================================================

projectDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(projectDir,'web_dashboard');

names = {'A*','ARA*','RRT*'};

colors = [
    0.20 0.60 0.95
    0.30 0.78 0.30
    1.00 0.60 0.18
];

metricFiles = {
    'astar_metrics.json'
    'ara_metrics.json'
    'rrt_metrics.json'
};

dynamicFiles = {
    'astar_dynamics.mat'
    'ara_dynamics.mat'
    'rrt_dynamics.mat'
};

M = cell(1,3);
D = cell(1,3);

if ~isfolder(dataDir)
    error('Folder not found: %s',dataDir);
end

for k = 1:3

    metricPath = fullfile(dataDir,metricFiles{k});
    dynamicPath = fullfile(dataDir,dynamicFiles{k});

    if ~isfile(metricPath)
        error('Missing metrics file: %s',metricPath);
    end

    if ~isfile(dynamicPath)
        error('Missing dynamic file: %s',dynamicPath);
    end

    M{k} = jsondecode(fileread(metricPath));

    loaded = load(dynamicPath);

    if ~isfield(loaded,'history')
        error('Variable history not found in %s',dynamicFiles{k});
    end

    D{k} = loaded.history;

    fprintf('Loaded %s successfully.\n',names{k});

end

%% =========================================================
% 2. EXTRACT PERFORMANCE METRICS
% ==========================================================

executionTime = nan(1,3);
victimsRescued = nan(1,3);
victimsDead = nan(1,3);
timePerVictim = nan(1,3);
successRate = nan(1,3);
averagePathLength = nan(1,3);

for k = 1:3

    % Values from the actual JSON files.
    executionTime(k) = readNumber(M{k},'time');
    victimsRescued(k) = readNumber(M{k},'rescued');
    victimsDead(k) = readNumber(M{k},'dead');
    averagePathLength(k) = readNumber(M{k},'avgPathLength');

    % This is a proxy, not an individually measured rescue time.
    if isfinite(executionTime(k)) && victimsRescued(k) > 0
        timePerVictim(k) = ...
            executionTime(k)/victimsRescued(k);
    end

    % Rescue success among victims classified as rescued or dead.
    totalVictims = victimsRescued(k) + victimsDead(k);

    if totalVictims > 0
        successRate(k) = ...
            100*victimsRescued(k)/totalVictims;
    end

end

%% =========================================================
% 3. FIGURE 1 - PERFORMANCE COMPARISON
% ==========================================================

metricNames = {
    'Execution Time'
    'Victims Rescued'
    'Victims Dead'
    'Time per Victim Rescued (Proxy)'
    'Rescue Success Rate (%)'
    'Average Path Length'
};

metricValues = {
    executionTime
    victimsRescued
    victimsDead
    timePerVictim
    successRate
    averagePathLength
};

yLabels = {
    'Simulation Steps'
    'Victim Count'
    'Victim Count'
    'Steps per Rescued Victim'
    'Success Rate (%)'
    'Distance Units'
};

fig1 = figure( ...
    'Name','Performance Comparison', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position',[70 40 1150 820]);

layout1 = tiledlayout(fig1,3,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

title(layout1, ...
    'Flood Rescue Algorithm Performance Comparison', ...
    'FontSize',16, ...
    'FontWeight','bold');

for j = 1:6

    ax = nexttile(layout1);
    values = metricValues{j};

    b = bar(ax,values,0.65,'FaceColor','flat');
    b.CData = colors;

    set(ax, ...
        'XTick',1:3, ...
        'XTickLabel',names, ...
        'FontSize',10, ...
        'LineWidth',1);

    title(ax,metricNames{j},'FontWeight','bold');
    ylabel(ax,yLabels{j});

    grid(ax,'on');
    box(ax,'on');

    finiteValues = values(isfinite(values));

    if ~isempty(finiteValues)

        maxValue = max(finiteValues);

        if maxValue > 0
            ylim(ax,[0 maxValue*1.18]);
        else
            ylim(ax,[0 1]);
        end

        hold(ax,'on');

        for k = 1:3

            if isfinite(values(k))

                text(ax,k,values(k), ...
                    sprintf('%.2f',values(k)), ...
                    'HorizontalAlignment','center', ...
                    'VerticalAlignment','bottom', ...
                    'FontSize',9);

            end

        end

        hold(ax,'off');

    end

end

%% =========================================================
% 4. FIGURE 2 - DYNAMIC COMPARISON
% ==========================================================

fig2 = figure( ...
    'Name','Dynamic Comparison', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position',[100 60 1150 820]);

layout2 = tiledlayout(fig2,2,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

title(layout2, ...
    'Flood Rescue Dynamic Performance Comparison', ...
    'FontSize',16, ...
    'FontWeight','bold');

%% PLOT 1: BOAT TRAJECTORIES

ax1 = nexttile(layout2);
hold(ax1,'on');

for k = 1:3

    x = D{k}.boatX(:);
    y = D{k}.boatY(:);

    n = min(numel(x),numel(y));

    valid = isfinite(x(1:n)) & isfinite(y(1:n));

    plot(ax1,x(1:n),y(1:n), ...
        'Color',colors(k,:), ...
        'LineWidth',2, ...
        'DisplayName',names{k});

    % Start marker
    if n >= 1 && any(valid)
        firstValid = find(valid,1,'first');
        plot(ax1,x(firstValid),y(firstValid),'o', ...
            'Color',colors(k,:), ...
            'MarkerFaceColor',colors(k,:), ...
            'HandleVisibility','off');
    end

end

title(ax1,'Boat Trajectories','FontWeight','bold');
xlabel(ax1,'X Position');
ylabel(ax1,'Y Position');
grid(ax1,'on');
box(ax1,'on');
axis(ax1,'equal');
legend(ax1,'Location','best');
hold(ax1,'off');

%% PLOT 2: TRACKING ERROR

ax2 = nexttile(layout2);
hold(ax2,'on');

for k = 1:3

    t = D{k}.t(:);
    err = D{k}.trackError(:);

    n = min(numel(t),numel(err));

    valid = isfinite(t(1:n)) & isfinite(err(1:n));

    plot(ax2,t(1:n),err(1:n), ...
        'Color',colors(k,:), ...
        'LineWidth',1.6, ...
        'DisplayName',names{k});

end

title(ax2,'Dynamic Tracking Error','FontWeight','bold');
xlabel(ax2,'Simulation Time');
ylabel(ax2,'Tracking Error');
grid(ax2,'on');
box(ax2,'on');
legend(ax2,'Location','best');
hold(ax2,'off');

%% PLOT 3: BOAT SPEED

ax3 = nexttile(layout2);
hold(ax3,'on');

for k = 1:3

    t = D{k}.t(:);
    speed = D{k}.velocity(:);

    n = min(numel(t),numel(speed));

    valid = isfinite(t(1:n)) & isfinite(speed(1:n));

    plot(ax3,t(1:n),speed(1:n), ...
        'Color',colors(k,:), ...
        'LineWidth',1.6, ...
        'DisplayName',names{k});

end

title(ax3,'Boat Speed During Rescue','FontWeight','bold');
xlabel(ax3,'Simulation Time');
ylabel(ax3,'Velocity');
grid(ax3,'on');
box(ax3,'on');
legend(ax3,'Location','best');
hold(ax3,'off');

%% PLOT 4: PID FORCE COMMAND

ax4 = nexttile(layout2);
hold(ax4,'on');

for k = 1:3

    t = D{k}.t(:);
    force = D{k}.Fcmd(:);

    n = min(numel(t),numel(force));

    valid = isfinite(t(1:n)) & isfinite(force(1:n));

    plot(ax4,t(1:n),force(1:n), ...
        'Color',colors(k,:), ...
        'LineWidth',1.6, ...
        'DisplayName',names{k});

end

title(ax4,'PID Force Command (Fcmd)','FontWeight','bold');
xlabel(ax4,'Simulation Time');
ylabel(ax4,'Force Command');
grid(ax4,'on');
box(ax4,'on');
legend(ax4,'Location','best');
hold(ax4,'off');

%% =========================================================
% 5. FINISH
% ==========================================================

drawnow;

fprintf('\n========================================\n');
fprintf('COMPARISON COMPLETED\n');
fprintf('Figure 1: Performance comparison\n');
fprintf('Figure 2: Dynamic comparison\n');
fprintf('No CSV or PNG files were saved.\n');
fprintf('========================================\n');

%% =========================================================
% LOCAL FUNCTION: READ A NUMERIC JSON FIELD
% ==========================================================

function value = readNumber(S,fieldName)

    value = NaN;

    if isstruct(S) && isfield(S,fieldName)

        candidate = S.(fieldName);

        if isnumeric(candidate) && ...
                isscalar(candidate) && isfinite(candidate)

            value = double(candidate);

        end
    end

end
