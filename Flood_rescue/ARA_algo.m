clc; clear; close all;

% Boat stabilization parameters. Tune with matched disturbance tests.
STAB_YAW_DAMPING = 8.0;
STAB_TORQUE_MAX = 60.0;

cd(fileparts(mfilename('fullpath')));

rng(1);

%% =========================================================

%  DYNAMIC FLOOD RESCUE

%  OPTIMIZED ARA* + PID + DIFFERENTIAL THRUST DYNAMICS

%

%  Optimizations:

%   1. No ARA* replanning every simulation tick

%   2. Controlled refinement interval

%   3. Efficient array-based parent tracking

%   4. No containers.Map

%   5. Path smoothing

%   6. Event-based dynamic replanning

%   7. Planning metrics

% =========================================================

%% ===================== CONFIG ============================

GRID = 70;

DT   = 0.25;

N_VICTIMS = 8;

N_DRONES  = 3;

N_BOATS   = 4;

N_DEBRIS  = 130;

waterFlow = [0.035 0.022];

%% ================= DYNAMIC WATER CURRENT ====================

waterBase          = waterFlow;
waterWaveAmp       = 0.035;
waterWaveFreq      = [0.018 0.024];
waterGustAmp       = 0.018;
waterShockChance   = 0.025;
waterShockAmp      = 0.045;
waterMaxMagnitude  = 0.120;
waterChangeGain    = 0.45;
waterDisturbRef    = 0.105;

waterFlowPrev      = waterFlow;
waterFlowRate      = [0 0];
waterDisturbance   = norm(waterFlow);
waterSeverity      = 0;

%% ================= ARA* PARAMETERS =======================

ARA_W_INIT  = 2.5;

ARA_W_DELTA = 0.5;

ARA_W_MIN   = 1.0;

% IMPORTANT:

% Do not refine every simulation tick.

ARA_REFINE_INTERVAL = 12;

% Check dynamic path only periodically.

PATH_CHECK_INTERVAL = 4;

VICTIM_MOVE_THRESH = 2.5;

%% ================= MOTHER SHIP ============================

MOTHERSHIP_POS    = [round(GRID/2) GRID-3];
MOTHERSHIP_RADIUS = 2.5;

%% ================= DYNAMIC WIND ==========================

wind.angle      = 0.0;

wind.speed      = 0.045;

wind.turnRate   = 0.008;

wind.turnAccel  = 0.0006;

wind.turnVel    = 0.0;

wind.maxSpeed   = 0.085;

wind.minSpeed   = 0.010;

wind.speedDrift = 0.0012;

wind.vec = wind.speed *  [cos(wind.angle) sin(wind.angle)];

%% ================= TURBULENCE ============================

turb.debris = 0.075;

turb.victim = 0.038;

turb.drone  = 0.010;

%% ================= BOAT SPEED ============================

SPEED_CRITICAL = 8.5;

SPEED_HIGH     = 6.5;

SPEED_MEDIUM   = 4.5;

SPEED_LOW      = 3.0;

MAX_SPEED = SPEED_CRITICAL;

BOAT_CURRENT   = 0.18;

BOAT_WIND_GAIN = 0.30;

%% ================= PID CONTROLLER ========================

PID_KP_HEADING = 55;

PID_KI_HEADING = 1.5;

PID_KD_HEADING = 18;

PID_KP_SPEED = 45;

PID_KI_SPEED = 4;

PID_KD_SPEED = 8;

HEADING_I_MAX = 2;

SPEED_I_MAX   = 10;

%% ================= ADAPTIVE PID SETTINGS ====================

PID_HEADING_SCALE_MIN = 0.85;
PID_HEADING_SCALE_MAX = 1.65;
PID_SPEED_SCALE_MIN   = 0.85;
PID_SPEED_SCALE_MAX   = 1.50;
PID_KI_REDUCTION      = 0.35;

WAYPOINT_TOL = 0.8;

%% ================= BOAT DYNAMICS =========================

BOAT_MASS = 100;

BOAT_IZ   = 45;

DRAG_V = 12;

DRAG_R = 8;

THRUSTER_SEP = 1.2;

TMAX         = 220;

%% ================= COLLISION AVOIDANCE ===================

BOAT_SEP_RADIUS = 3.5;

BOAT_SEP_FORCE  = 2.2;

%% ================= DANGER SCORING ========================

DANGER_DEBRIS_RADIUS = 5.0;

CRIT_THRESH = 12;

HIGH_THRESH = 7;

MED_THRESH  = 3;

%% ================= PREEMPTION ============================

PREEMPT_ENABLE = true;

%% ================= DRONE SETTINGS ========================

DRONE_SCAN_BASE   = 30;

DRONE_SCAN_GROWTH = 4;

%% ================= OUTPUT ================================

outDir = fullfile(  fileparts(mfilename('fullpath')),  'web_dashboard');

if ~exist(outDir,'dir')

    mkdir(outDir);

end

%% =========================================================

% CREATE DEBRIS

% ==========================================================

debrisPos = double([  randi([3 GRID-2],N_DEBRIS,1),  randi([3 GRID-2],N_DEBRIS,1)]);

%% =========================================================

% CREATE VICTIMS

% ==========================================================

victims = struct();

for v = 1:N_VICTIMS

    while true

        p = [  randi([6 GRID-5]),  randi([6 GRID-5])];

        if ~tooClose(p,debrisPos,5)

            break;

        end

    end

    victims(v).pos       = double(p);

    victims(v).rescued   = false;

    victims(v).dead      = false;

    victims(v).detected  = false;

    victims(v).danger    = 0;

    victims(v).dangerLvl = 'LOW';

    victims(v).prevLvl   = 'LOW';

    victims(v).assignedB = 0;

    debrisPos = clearAround(  debrisPos,p,5);

end

%% =========================================================

% CREATE DRONES

% ==========================================================

bandH = GRID/N_DRONES;

drones = struct();

for d = 1:N_DRONES

    cy = round(  bandH*(d-1) + bandH/2);

    drones(d).pos =  double([4 cy]);

    drones(d).dir = 1;

    drones(d).scanR =  DRONE_SCAN_BASE;

    drones(d).yLo =  max(1,round(bandH*(d-1)+1));

    drones(d).yHi =  min(GRID,round(bandH*d));

end

%% =========================================================

% CREATE BOATS

% ==========================================================

BSTARTS = [  2 2;

    2 GRID-1;

    GRID-1 2;

    GRID-1 GRID-1];

boats = struct();

for b = 1:N_BOATS

    boats(b).pos =  double(BSTARTS(b,:));

    boats(b).assigned = false;

    boats(b).victimIdx = 0;

    boats(b).path = [];

    boats(b).pathIdx = 1;

    boats(b).speed = SPEED_LOW;

    boats(b).preempted = false;

    boats(b).returning = false;
    boats(b).returnedToMothership = false;

    % Adaptive PID telemetry
    boats(b).KpHeading = PID_KP_HEADING;
    boats(b).KiHeading = PID_KI_HEADING;
    boats(b).KdHeading = PID_KD_HEADING;
    boats(b).KpSpeed = PID_KP_SPEED;
    boats(b).KiSpeed = PID_KI_SPEED;
    boats(b).KdSpeed = PID_KD_SPEED;
    boats(b).pidGainFactor = 1.0;
    boats(b).waterDisturbance = 0;
    boats(b).waterSeverity = 0;

    % ARA* state

    boats(b).araW = ARA_W_INIT;

    boats(b).lastPlanTick = 0;

    boats(b).lastCheckTick = 0;

    boats(b).planCount = 0;

    boats(b).planningTime = 0;

    % Boat dynamic state

    boats(b).psi = 0;

    boats(b).vel = 0;

    boats(b).r = 0;

    % PID states

    boats(b).headingI = 0;

    boats(b).prevHeadE = 0;

    boats(b).speedI = 0;

    boats(b).prevSpeedE = 0;

    % Controller outputs

    boats(b).Fcmd = 0;

    boats(b).tauCmd = 0;
    boats(b).tauStabilization = 0;
    boats(b).stabilizationEffortPct = 0;
    boats(b).headingErrorDeg = 0;

    boats(b).TL = 0;

    boats(b).TR = 0;

    boats(b).trackError = 0;

    debrisPos = clearAround(  debrisPos,boats(b).pos,4);

end

%% =========================================================

% METRICS

% ==========================================================

boatRescueCounts =  zeros(1,N_BOATS);

totalPathLength = 0;

totalPlanningTime = 0;

totalPlans = 0;

%% ================= DYNAMIC DATA LOGGING =================

history.t = [];
history.tauStabilization = [];
history.stabilizationEffortPct = [];
history.headingErrorDeg = [];

history.velocity = [];
history.heading = [];
history.yawRate = [];
history.trackError = [];

history.Fcmd = [];
history.tauCmd = [];
history.TL = [];
history.TR = [];

history.KpHeading = [];
history.KiHeading = [];
history.KdHeading = [];

history.KpSpeed = [];
history.KiSpeed = [];
history.KdSpeed = [];

history.pidGainFactor = [];

history.waterDisturbance = [];
history.waterSeverity = [];

history.araW = [];

history.boatX = [];
history.boatY = [];
%% =========================================================

% MAIN LOOP

% ==========================================================

t = 0;

nRescued = 0;

nDead = 0;

fprintf('\n');

fprintf('=====================================\n');

fprintf(' ARA* + ADAPTIVE PID FLOOD RESCUE\n');

fprintf('=====================================\n');

fprintf('Initial Weight : %.2f\n',ARA_W_INIT);

fprintf('Weight Step    : %.2f\n',ARA_W_DELTA);

fprintf('Refine Every   : %d ticks\n',  ARA_REFINE_INTERVAL);

fprintf('Path Check     : %d ticks\n',  PATH_CHECK_INTERVAL);

fprintf('Open http://localhost:8000\n\n');

while ((nRescued+nDead) < N_VICTIMS ||  any(~[boats.returnedToMothership])) && t < 6000

    t = t + 1;

    %% =====================================================

    % 1. WIND PHYSICS

    % ======================================================

    wind.turnVel =  wind.turnVel  + wind.turnAccel*(rand()-0.5)*2;

    wind.turnVel =  max(-wind.turnRate,  min(wind.turnRate,wind.turnVel));

    wind.angle =  wind.angle + wind.turnVel;

    wind.speed =  wind.speed  + wind.speedDrift*(rand()-0.5)*2;

    wind.speed =  max(wind.minSpeed,  min(wind.maxSpeed,wind.speed));

    wind.vec =  wind.speed *  [cos(wind.angle)  sin(wind.angle)];

    %% =====================================================

    % 2. DEBRIS MOTION

    % ======================================================

    nD = size(debrisPos,1);

    tj = turb.debris *  (rand(nD,2)-0.5);

    debrisPos =  debrisPos  + waterFlow  + wind.vec*0.65  + tj;

    debrisPos(:,1) =  max(1,min(GRID,debrisPos(:,1)));

    debrisPos(:,2) =  max(1,min(GRID,debrisPos(:,2)));

    %% =====================================================

    % 3. VICTIM MOTION

    % ======================================================

    for v = 1:N_VICTIMS

        if victims(v).rescued ||  victims(v).dead

            continue;

        end

        tj =  turb.victim*(rand(1,2)-0.5);

        newPos =  victims(v).pos  + waterFlow  + wind.vec*0.5  + tj;

        if newPos(1)<1 ||  newPos(1)>GRID ||  newPos(2)<1 ||  newPos(2)>GRID

            victims(v).dead = true;

            if victims(v).assignedB ~= 0

                b = victims(v).assignedB;

                boats(b).assigned = false;

                boats(b).victimIdx = 0;

                boats(b).path = [];

                boats(b).pathIdx = 1;

                boats(b).speed = SPEED_LOW;

                boats(b).araW = ARA_W_INIT;

                victims(v).assignedB = 0;

                boats(b) =  resetBoatControl(boats(b));

            end

            nDead = nDead + 1;

            for d = 1:N_DRONES

                drones(d).scanR =  DRONE_SCAN_BASE  + nDead*DRONE_SCAN_GROWTH;

            end

            continue;

        end

        victims(v).pos = newPos;

    end

    %% =====================================================

    % 4. DRONE MOTION

    % ======================================================

    for d = 1:N_DRONES

        tj =  turb.drone*(rand(1,2)-0.5);

        drones(d).pos(1) =  drones(d).pos(1)  + drones(d).dir*0.5  + wind.vec(1)*0.10  + tj(1);

        drones(d).pos(2) =  drones(d).pos(2)  + wind.vec(2)*0.10  + tj(2);

        drones(d).pos(2) =  max(drones(d).yLo,  min(drones(d).yHi,  drones(d).pos(2)));

        if drones(d).pos(1) > GRID-2

            drones(d).dir = -1;

        end

        if drones(d).pos(1) < 2

            drones(d).dir = 1;

        end

        drones(d).pos =  clampPos(drones(d).pos,GRID);

    end

    %% =====================================================

    % 5. DANGER SCORING

    % ======================================================

    for v = 1:N_VICTIMS

        if victims(v).rescued ||  victims(v).dead

            continue;

        end

        victims(v).prevLvl =  victims(v).dangerLvl;

        vp = victims(v).pos;

        dists = sqrt(  (debrisPos(:,1)-vp(1)).^2  +  (debrisPos(:,2)-vp(2)).^2);

        nNear =  sum(dists < DANGER_DEBRIS_RADIUS);

        margin = min([  vp(1)-1,  GRID-vp(1),  vp(2)-1,  GRID-vp(2)]);

        boundaryPenalty =  max(0,10-margin);

        score =  nNear + boundaryPenalty;

        victims(v).danger = score;

        if score >= CRIT_THRESH

            victims(v).dangerLvl = 'CRITICAL';

        elseif score >= HIGH_THRESH

            victims(v).dangerLvl = 'HIGH';

        elseif score >= MED_THRESH

            victims(v).dangerLvl = 'MEDIUM';

        else

            victims(v).dangerLvl = 'LOW';

        end

    end

    %% =====================================================

    % 6. BUILD OBSTACLE MAP

    % ======================================================

    oMap =  buildObstacleMap(debrisPos,GRID);

    for b = 1:N_BOATS

        bp =  safeCell(boats(b).pos,GRID);

        oMap(bp(2),bp(1)) = 0;

    end

    for v = 1:N_VICTIMS

        if ~victims(v).rescued &&  ~victims(v).dead

            vp =  safeCell(  victims(v).pos,GRID);

            oMap(vp(2),vp(1)) = 0;

        end

    end

    %% =====================================================

    % 7. DRONE DETECTION

    % ======================================================

    for d = 1:N_DRONES

        for v = 1:N_VICTIMS

            if victims(v).rescued ||  victims(v).dead ||  victims(v).detected

                continue;

            end

            if norm(  drones(d).pos  - victims(v).pos)  <= drones(d).scanR

                victims(v).detected = true;

            end

        end

    end

    %% =====================================================

    % 8. UPDATE BOAT SPEED

    % ======================================================

    for b = 1:N_BOATS

        if ~boats(b).assigned

            continue;

        end

        v = boats(b).victimIdx;

        boats(b).speed =  dangerSpeed(  victims(v).dangerLvl,  SPEED_CRITICAL,  SPEED_HIGH,  SPEED_MEDIUM,  SPEED_LOW);

    end

    %% =====================================================

    % 9. OPTIMIZED ARA* REFINEMENT

    %

    % Instead of replanning every tick,

    % refine periodically.

    % ======================================================

    if mod(t,ARA_REFINE_INTERVAL) == 0

        for b = 1:N_BOATS

            if ~boats(b).assigned

                continue;

            end

            if boats(b).araW <= ARA_W_MIN

                continue;

            end

            v = boats(b).victimIdx;

            if victims(v).dead

                continue;

            end

            boats(b).araW =  max(ARA_W_MIN,  boats(b).araW - ARA_W_DELTA);

            tic;

            [newPath,newIdx] =  planPathARA(  oMap,  boats(b).pos,  victims(v).pos,  boats(b).araW);

            planTime = toc;

            % Only replace current path

            % if a valid path was found.

            if ~isempty(newPath)

                boats(b).path = newPath;

                boats(b).pathIdx = newIdx;

            end

            boats(b).lastPlanTick = t;

            boats(b).planCount =  boats(b).planCount + 1;

            boats(b).planningTime =  boats(b).planningTime  + planTime;

            totalPlanningTime =  totalPlanningTime  + planTime;

            totalPlans =  totalPlans + 1;

        end

    end

    %% =====================================================

    % 10. PREEMPTIVE REASSIGNMENT

    % ======================================================

    if PREEMPT_ENABLE

        bestV = 0;

        bestScore = -1;

        for v = 1:N_VICTIMS

            if victims(v).rescued ||  victims(v).dead ||  ~victims(v).detected

                continue;

            end

            if victims(v).assignedB ~= 0

                continue;

            end

            if victims(v).danger > bestScore

                bestScore =  victims(v).danger;

                bestV = v;

            end

        end

        if bestV > 0 &&  bestScore >= HIGH_THRESH

            for b = 1:N_BOATS

                if ~boats(b).assigned

                    continue;

                end

                cv = boats(b).victimIdx;

                if dangerRank(  victims(bestV).dangerLvl)  >  dangerRank(  victims(cv).dangerLvl)+1

                    victims(cv).assignedB = 0;

                    boats(b).victimIdx = bestV;

                    victims(bestV).assignedB = b;

                    boats(b).preempted = true;

                    boats(b).araW = ARA_W_INIT;

                    tic;

                    [boats(b).path,  boats(b).pathIdx] =  planPathARA(  oMap,  boats(b).pos,  victims(bestV).pos,  boats(b).araW);

                    planTime = toc;

                    boats(b).planningTime =  boats(b).planningTime  + planTime;

                    boats(b).planCount =  boats(b).planCount + 1;

                    totalPlanningTime =  totalPlanningTime  + planTime;

                    totalPlans =  totalPlans + 1;

                    boats(b) =  resetBoatControl(boats(b));

                    break;

                end

            end

        end

    end

    %% =====================================================

    % 11. PRIORITY DISPATCH

    % ======================================================

    freeBoats = [];

    for b = 1:N_BOATS

        if ~boats(b).assigned

            freeBoats(end+1) = b;

        end

    end

    if ~isempty(freeBoats)

        unassignedV = [];

        unassignedS = [];

        for v = 1:N_VICTIMS

            if victims(v).rescued ||  victims(v).dead ||  ~victims(v).detected

                continue;

            end

            if victims(v).assignedB ~= 0

                continue;

            end

            unassignedV(end+1) = v;

            unassignedS(end+1) =  victims(v).danger;

        end

        [~,order] =  sort(unassignedS,'descend');

        unassignedV =  unassignedV(order);

        for vi = 1:min(  length(unassignedV),  length(freeBoats))

            v = unassignedV(vi);

            bestB = freeBoats(1);

            bestD = inf;

            for fb = freeBoats

                d = norm(  boats(fb).pos  - victims(v).pos);

                if d < bestD

                    bestD = d;

                    bestB = fb;

                end

            end

            boats(bestB).assigned = true;

            boats(bestB).victimIdx = v;

            boats(bestB).preempted = false;

            boats(bestB).araW = ARA_W_INIT;

            boats(bestB).speed =  dangerSpeed(  victims(v).dangerLvl,  SPEED_CRITICAL,  SPEED_HIGH,  SPEED_MEDIUM,  SPEED_LOW);

            victims(v).assignedB = bestB;

            tic;

            [boats(bestB).path,  boats(bestB).pathIdx] =  planPathARA(  oMap,  boats(bestB).pos,  victims(v).pos,  boats(bestB).araW);

            planTime = toc;

            boats(bestB).planningTime =  boats(bestB).planningTime  + planTime;

            boats(bestB).planCount =  boats(bestB).planCount + 1;

            totalPlanningTime =  totalPlanningTime  + planTime;

            totalPlans =  totalPlans + 1;

            boats(bestB) =  resetBoatControl(boats(bestB));

            freeBoats(  freeBoats == bestB) = [];

        end

    end

    %% =====================================================

    % 12. BOAT MOVEMENT

    % ======================================================

    for b = 1:N_BOATS

        % Send boats to the Mother Ship after all victims are resolved.
        if (nRescued+nDead) >= N_VICTIMS &&  ~boats(b).assigned && ~boats(b).returning &&  ~boats(b).returnedToMothership

            boats(b).returning = true;
            boats(b).path = [];
            boats(b).pathIdx = 1;

            tic;
            [boats(b).path,boats(b).pathIdx] = planPathARA(  oMap,boats(b).pos,MOTHERSHIP_POS,boats(b).araW);
            planTime = toc;

            boats(b).planningTime = boats(b).planningTime + planTime;
            boats(b).planCount = boats(b).planCount + 1;
            totalPlanningTime = totalPlanningTime + planTime;
            totalPlans = totalPlans + 1;
            boats(b) = resetBoatControl(boats(b));
        end

        if boats(b).returning

            if norm(boats(b).pos-MOTHERSHIP_POS) <= MOTHERSHIP_RADIUS
                boats(b).returning = false;
                boats(b).returnedToMothership = true;
                boats(b).path = [];
                boats(b).pathIdx = 1;
                boats(b).speed = SPEED_LOW;
                boats(b) = resetBoatControl(boats(b));
                continue;
            end

            v = 0;
            vpos = MOTHERSHIP_POS;

        else

            if ~boats(b).assigned
                continue;
            end

            v = boats(b).victimIdx;

            if victims(v).dead
                boats(b).assigned = false;
                boats(b).victimIdx = 0;
                boats(b).path = [];
                boats(b).pathIdx = 1;
                boats(b).speed = SPEED_LOW;
                boats(b).araW = ARA_W_INIT;
                boats(b) = resetBoatControl(boats(b));
                continue;
            end

            vpos = victims(v).pos;
        end

        %% ===============================================

        % PERIODIC PATH VALIDATION

        % ================================================

        if mod(t,PATH_CHECK_INTERVAL) == 0

            pathBlocked =  isPathBlocked(  boats(b).path,  boats(b).pathIdx,  oMap,GRID);

            victimMoved =  victimPathChanged(  boats(b).path,  vpos,  VICTIM_MOVE_THRESH);

            if pathBlocked || victimMoved

                boats(b).araW =  ARA_W_INIT;

                tic;

                [boats(b).path,  boats(b).pathIdx] =  planPathARA(  oMap,  boats(b).pos,  vpos,  boats(b).araW);

                planTime = toc;

                boats(b).planningTime =  boats(b).planningTime  + planTime;

                boats(b).planCount =  boats(b).planCount + 1;

                totalPlanningTime =  totalPlanningTime  + planTime;

                totalPlans =  totalPlans + 1;

            end

        end

        %% ===============================================

        % PID WAYPOINT TRACKING

        % ================================================

        if isempty(boats(b).path)

            continue;

        end

        if boats(b).pathIdx >  size(boats(b).path,1)

            continue;

        end

        target =  double(  boats(b).path(  boats(b).pathIdx,:));

        direction =  target - boats(b).pos;

        dist = norm(direction);

        boats(b).trackError = dist;

        if dist < WAYPOINT_TOL

            boats(b).pathIdx =  boats(b).pathIdx + 1;

        else

            oldPos = boats(b).pos;

            %% GUIDANCE

            psiDes =  atan2(  direction(2),  direction(1));

            speedDes =  min(  boats(b).speed,  max(0.7,1.6*dist));

            %% ADAPTIVE PID GAIN SCHEDULING

        disturbance = norm(waterFlow*BOAT_CURRENT +  wind.vec*BOAT_WIND_GAIN) +  waterChangeGain*norm(waterFlowRate);

        pidGainFactor = PID_HEADING_SCALE_MIN +  waterSeverity*(PID_HEADING_SCALE_MAX-PID_HEADING_SCALE_MIN);

        KpHeading = PID_KP_HEADING*pidGainFactor;
        KdHeading = PID_KD_HEADING*pidGainFactor;
        KiHeading = PID_KI_HEADING*(1-PID_KI_REDUCTION*waterSeverity);

        speedGainFactor = PID_SPEED_SCALE_MIN +  waterSeverity*(PID_SPEED_SCALE_MAX-PID_SPEED_SCALE_MIN);

        KpSpeedAdaptive = PID_KP_SPEED*speedGainFactor;
        KdSpeedAdaptive = PID_KD_SPEED*speedGainFactor;
        KiSpeedAdaptive = PID_KI_SPEED*(1-PID_KI_REDUCTION*waterSeverity);

        boats(b).KpHeading = KpHeading;
        boats(b).KiHeading = KiHeading;
        boats(b).KdHeading = KdHeading;
        boats(b).KpSpeed = KpSpeedAdaptive;
        boats(b).KiSpeed = KiSpeedAdaptive;
        boats(b).KdSpeed = KdSpeedAdaptive;
        boats(b).pidGainFactor = pidGainFactor;
        boats(b).waterDisturbance = disturbance;
        boats(b).waterSeverity = waterSeverity;

        %% HEADING PID

            headE =  wrapAngle(  psiDes - boats(b).psi);

            boats(b).headingI =  max(  -HEADING_I_MAX,  min(  HEADING_I_MAX,  boats(b).headingI  + headE*DT));

            headD =  wrapAngle(  headE  - boats(b).prevHeadE)/DT;

            tauCmd =  KpHeading*headE  + KiHeading*boats(b).headingI  + KdHeading*headD;

            % Additional yaw-rate damping stabilizes unwanted boat rotation.
            tauPID = tauCmd;
            tauStab = -STAB_YAW_DAMPING * boats(b).r;
            tauStab = max(-STAB_TORQUE_MAX, min(STAB_TORQUE_MAX, tauStab));
            tauCmd = tauPID + tauStab;
            boats(b).tauStabilization = tauStab;
            boats(b).stabilizationEffortPct = 100 * abs(tauStab) / max(STAB_TORQUE_MAX, eps);
            boats(b).headingErrorDeg = rad2deg(headE);


            boats(b).prevHeadE = headE;

            %% SPEED PID

            speedE =  speedDes - boats(b).vel;

            boats(b).speedI =  max(  -SPEED_I_MAX,  min(  SPEED_I_MAX,  boats(b).speedI  + speedE*DT));

            speedD =  (speedE  - boats(b).prevSpeedE)/DT;

            Fcmd =  max(0,  KpSpeedAdaptive*speedE  + KiSpeedAdaptive*boats(b).speedI  + KdSpeedAdaptive*speedD);

            boats(b).prevSpeedE = speedE;

            %% DIFFERENTIAL THRUST

            TL =  0.5*Fcmd  - tauCmd/THRUSTER_SEP;

            TR =  0.5*Fcmd  + tauCmd/THRUSTER_SEP;

            TL =  max(-TMAX,min(TMAX,TL));

            TR =  max(-TMAX,min(TMAX,TR));

            F = TL + TR;

            tau =  (TR-TL)  * THRUSTER_SEP/2;

            boats(b).Fcmd = Fcmd;

            boats(b).tauCmd = tauCmd;

            boats(b).TL = TL;

            boats(b).TR = TR;

            %% BOAT DYNAMICS

            vDot =  (F - DRAG_V*boats(b).vel)  / BOAT_MASS;

            rDot =  (tau - DRAG_R*boats(b).r)  / BOAT_IZ;

            boats(b).vel =  max(0,  min(MAX_SPEED,  boats(b).vel + vDot*DT));

            boats(b).r =  boats(b).r + rDot*DT;

            boats(b).psi =  wrapAngle(  boats(b).psi  + boats(b).r*DT);

            %% WORLD MOTION

            bodyVel =  boats(b).vel  *  [cos(boats(b).psi),  sin(boats(b).psi)];

            currentVel =  waterFlow*BOAT_CURRENT;

            windVel =  wind.vec*BOAT_WIND_GAIN;

            boats(b).pos =  boats(b).pos  +  (bodyVel  + currentVel  + windVel)*DT;

            totalPathLength =  totalPathLength  + norm(  boats(b).pos-oldPos);

        end

        %% ===============================================

        % BOAT COLLISION AVOIDANCE

        % ================================================

        sepForce = [0 0];

        for ob = 1:N_BOATS

            if ob == b

                continue;

            end

            diff =  boats(b).pos  - boats(ob).pos;

            d = norm(diff);

            if d < BOAT_SEP_RADIUS  && d > 1e-4

                pushMag =  BOAT_SEP_FORCE  *  (1-d/BOAT_SEP_RADIUS);

                sepForce =  sepForce  + (diff/d)*pushMag;

            end

        end

        boats(b).pos =  boats(b).pos  + sepForce*DT;

        boats(b).pos =  clampPos(boats(b).pos,GRID);

        %% ===============================================

        % RESCUE CHECK

        % ================================================

        if norm(  boats(b).pos  - vpos) < 1.8

            victims(v).rescued = true;

            victims(v).assignedB = 0;

            boats(b).assigned = false;

            boats(b).victimIdx = 0;

            boats(b).path = [];

            boats(b).pathIdx = 1;

            boats(b).speed = SPEED_LOW;

            boats(b).preempted = false;

    boats(b).returning = false;
    boats(b).returnedToMothership = false;

    % Adaptive PID telemetry
    boats(b).KpHeading = PID_KP_HEADING;
    boats(b).KiHeading = PID_KI_HEADING;
    boats(b).KdHeading = PID_KD_HEADING;
    boats(b).KpSpeed = PID_KP_SPEED;
    boats(b).KiSpeed = PID_KI_SPEED;
    boats(b).KdSpeed = PID_KD_SPEED;
    boats(b).pidGainFactor = 1.0;
    boats(b).waterDisturbance = 0;
    boats(b).waterSeverity = 0;

            boats(b).araW = ARA_W_INIT;

            boats(b) =  resetBoatControl(boats(b));

            nRescued = nRescued + 1;

            boatRescueCounts(b) =  boatRescueCounts(b) + 1;

        end

    end

    %% =====================================================

    % 13. EXPORT JSON

    % ======================================================

    vDat = struct();

    for v = 1:N_VICTIMS

        vDat(v).id = v;

        vDat(v).x = victims(v).pos(1);

        vDat(v).y = victims(v).pos(2);

        vDat(v).lvl =  victims(v).dangerLvl;

        vDat(v).danger =  victims(v).danger;

        vDat(v).dead =  victims(v).dead;

        vDat(v).rescued =  victims(v).rescued;

        vDat(v).detected =  victims(v).detected;

    end

    %% BOAT DATA

    bDat = struct();

    for b = 1:N_BOATS

        bDat(b).id = b;

        bDat(b).x = boats(b).pos(1);

        bDat(b).y = boats(b).pos(2);

        bDat(b).target =  boats(b).victimIdx;

        bDat(b).speed =  boats(b).speed;

        bDat(b).preempted =  boats(b).preempted;

        bDat(b).rescues =  boatRescueCounts(b);

        bDat(b).araW =  boats(b).araW;

        bDat(b).heading =  rad2deg(boats(b).psi);

        bDat(b).velocity =  boats(b).vel;

        bDat(b).yawRate =  boats(b).r;

        bDat(b).Fcmd =  boats(b).Fcmd;

        bDat(b).tauCmd =  boats(b).tauCmd;
    bDat(b).tauStabilization = boats(b).tauStabilization;
    bDat(b).stabilizationEffortPct = boats(b).stabilizationEffortPct;
    bDat(b).headingErrorDeg = boats(b).headingErrorDeg;

        bDat(b).TL =  boats(b).TL;

        bDat(b).TR =  boats(b).TR;

        bDat(b).trackErr =  boats(b).trackError;

        bDat(b).plans =  boats(b).planCount;
        bDat(b).speedTarget = boats(b).speed;
        bDat(b).KpHeading = boats(b).KpHeading;
        bDat(b).KiHeading = boats(b).KiHeading;
        bDat(b).KdHeading = boats(b).KdHeading;
        bDat(b).KpSpeed = boats(b).KpSpeed;
        bDat(b).KiSpeed = boats(b).KiSpeed;
        bDat(b).KdSpeed = boats(b).KdSpeed;
        bDat(b).pidGainFactor = boats(b).pidGainFactor;
        bDat(b).waterDisturbance = boats(b).waterDisturbance;
        bDat(b).waterSeverity = boats(b).waterSeverity;
        bDat(b).returning = boats(b).returning;
        bDat(b).returnedToMothership = boats(b).returnedToMothership;

        if ~isempty(boats(b).path)

            bDat(b).path =  boats(b).path;

        else

            bDat(b).path = [];

        end

    end

    %% DRONE DATA

    dDat = struct();

    for d = 1:N_DRONES

        dDat(d).id = d;

        dDat(d).x = drones(d).pos(1);

        dDat(d).y = drones(d).pos(2);

        dDat(d).r = drones(d).scanR;

    end

    %% GLOBAL STATE

    S.t = t;

    S.wind = wind.speed;

    S.windAng = rad2deg(wind.angle);

    S.rescued = nRescued;

    S.dead = nDead;

    S.victims = vDat;

    S.boats = bDat;

    S.drones = dDat;

    S.debris = debrisPos;

    S.waterFlowX = waterFlow(1);
    S.waterFlowY = waterFlow(2);
    S.waterMagnitude = norm(waterFlow);
    S.waterRate = norm(waterFlowRate);
    S.waterDisturbance = waterDisturbance;
    S.waterSeverity = waterSeverity;

    S.motherShip = struct();
    S.motherShip.x = MOTHERSHIP_POS(1);
    S.motherShip.y = MOTHERSHIP_POS(2);
    S.motherShip.radius = MOTHERSHIP_RADIUS;
    S.returnedBoats = sum([boats.returnedToMothership]);

    %% WRITE JSON

S.stabilization = struct('enabled',true,'controller','Adaptive PID + yaw-rate damping','yawDampingGain',STAB_YAW_DAMPING,'stabilizerTorqueLimit',STAB_TORQUE_MAX);

jsonStr = jsonencode(S);

fid = fopen( fullfile(outDir,'state.json'), 'w');

fprintf(fid,'%s',jsonStr);

fclose(fid);

%% ================= DYNAMIC HISTORY LOGGING =================

history.t(end+1) = t * DT;

history.velocity(end+1) = mean([boats.vel]);

history.heading(end+1) = mean(rad2deg([boats.psi]));

history.yawRate(end+1) = mean([boats.r]);
history.tauStabilization(end+1) = mean([boats.tauStabilization]);
history.stabilizationEffortPct(end+1) = mean([boats.stabilizationEffortPct]);
history.headingErrorDeg(end+1) = mean([boats.headingErrorDeg]);

history.trackError(end+1) = mean(abs([boats.trackError]));

history.Fcmd(end+1) = mean([boats.Fcmd]);

history.tauCmd(end+1) = mean([boats.tauCmd]);

history.TL(end+1) = mean([boats.TL]);

history.TR(end+1) = mean([boats.TR]);

history.KpHeading(end+1) = mean([boats.KpHeading]);

history.KiHeading(end+1) = mean([boats.KiHeading]);

history.KdHeading(end+1) = mean([boats.KdHeading]);

history.KpSpeed(end+1) = mean([boats.KpSpeed]);

history.KiSpeed(end+1) = mean([boats.KiSpeed]);

history.KdSpeed(end+1) = mean([boats.KdSpeed]);

history.pidGainFactor(end+1) = mean([boats.pidGainFactor]);

history.waterDisturbance(end+1) = waterDisturbance;

history.waterSeverity(end+1) = waterSeverity;

history.araW(end+1) = mean([boats.araW]);

history.boatX(end+1) = boats(1).pos(1);

history.boatY(end+1) = boats(1).pos(2);

pause(0.1);

end
%% ================= SAVE DYNAMIC HISTORY ==================

save(fullfile(outDir,'ara_dynamics.mat'),'history');

%% =========================================================

% FINAL METRICS

% ==========================================================

fprintf('\n');

fprintf('=====================================\n');

fprintf('SIMULATION COMPLETE\n');

fprintf('=====================================\n');

fprintf('Rescued        : %d\n',nRescued);

fprintf('Lost           : %d\n',nDead);

fprintf('Simulation Step: %d\n',t);

fprintf('Total Plans    : %d\n',totalPlans);

fprintf('Planning Time  : %.4f seconds\n',  totalPlanningTime);

fprintf('Path Length    : %.2f\n',  totalPathLength);

metrics.algorithm =  "Optimized ARA* + PID + Dynamic Boat";

metrics.time = t;

metrics.rescued = nRescued;

metrics.dead = nDead;

metrics.pathLength = totalPathLength;

metrics.avgPathLength =  totalPathLength / max(nRescued,1);

metrics.planningTime =  totalPlanningTime;

metrics.totalPlans =  totalPlans;

if totalPlans > 0

    metrics.avgPlanningTime =  totalPlanningTime/totalPlans;

else

    metrics.avgPlanningTime = 0;

end

metrics.stabilization = struct();
metrics.stabilization.enabled = true;
metrics.stabilization.controller = 'Adaptive PID + yaw-rate damping';
metrics.stabilization.yawDampingGain = STAB_YAW_DAMPING;
metrics.stabilization.torqueLimit = STAB_TORQUE_MAX;
metrics.stabilization.meanEffortPct = mean(history.stabilizationEffortPct);
metrics.stabilization.maxEffortPct = max(history.stabilizationEffortPct);
metrics.stabilization.meanAbsHeadingErrorDeg = mean(abs(history.headingErrorDeg));
metrics.stabilization.maxAbsHeadingErrorDeg = max(abs(history.headingErrorDeg));
metrics.stabilization.meanAbsYawRate = mean(abs(history.yawRate));

jsonStr = jsonencode(metrics);

fid = fopen(  fullfile(  outDir,'ara_metrics.json'),  'w');

fprintf(fid,'%s',jsonStr);

fclose(fid);

%% =========================================================

% HELPER FUNCTIONS

% ==========================================================

function map = buildObstacleMap(pos,G)

    map = false(G,G);

    xi =  max(1,min(G,round(pos(:,1))));

    yi =  max(1,min(G,round(pos(:,2))));

    idx =  sub2ind([G G],yi,xi);

    map(idx) = true;

end

function p = clampPos(p,G)

    p(1) =  max(1,min(double(G),p(1)));

    p(2) =  max(1,min(double(G),p(2)));

end

function c = safeCell(p,G)

    c =  max(1,min(G,round(p)));

end

function dpos = clearAround(dpos,point,r)

    d = sqrt(  (dpos(:,1)-point(1)).^2  +  (dpos(:,2)-point(2)).^2);

    dpos(d<r,:) = [];

end

function result = tooClose(p,positions,r)

    if isempty(positions)

        result = false;

        return;

    end

    d = sqrt(  (positions(:,1)-p(1)).^2  +  (positions(:,2)-p(2)).^2);

    result = any(d<r);

end

function spd = dangerSpeed(  lvl,sc,sh,sm,sl)

    switch lvl

        case 'CRITICAL'

            spd = sc;

        case 'HIGH'

            spd = sh;

        case 'MEDIUM'

            spd = sm;

        otherwise

            spd = sl;

    end

end

function r = dangerRank(lvl)

    switch lvl

        case 'CRITICAL'

            r = 4;

        case 'HIGH'

            r = 3;

        case 'MEDIUM'

            r = 2;

        otherwise

            r = 1;

    end

end

function a = wrapAngle(a)

    a = atan2(sin(a),cos(a));

end

%% =========================================================

% RESET BOAT CONTROL

% ==========================================================

function boat = resetBoatControl(boat)

    boat.vel = 0;

    boat.r = 0;

    boat.headingI = 0;

    boat.prevHeadE = 0;

    boat.speedI = 0;

    boat.prevSpeedE = 0;

    boat.Fcmd = 0;

    boat.tauCmd = 0;

    boat.TL = 0;

    boat.TR = 0;

    boat.trackError = 0;

end

%% =========================================================

% PATH BLOCK CHECK

% ==========================================================

function blocked =  isPathBlocked(path,pathIdx,map,G)

    blocked = false;

    if isempty(path)

        blocked = true;

        return;

    end

    if pathIdx > size(path,1)

        return;

    end

    startIdx =  max(1,pathIdx);

    for k = startIdx:size(path,1)

        p = safeCell(path(k,:),G);

        if map(p(2),p(1))

            blocked = true;

            return;

        end

    end

end

%% =========================================================

% VICTIM PATH CHANGE

% ==========================================================

function moved =  victimPathChanged(path,victimPos,threshold)

    if isempty(path)

        moved = true;

        return;

    end

    finalPoint =  double(path(end,:));

    moved =  norm(finalPoint-victimPos)  > threshold;

end

%% =========================================================

% ARA* PLANNER WRAPPER

% ==========================================================

function [path,idx] =  planPathARA(oMap,boatPos,victimPos,W)

    G = size(oMap,1);

    start =  safeCell(boatPos,G);

    goal =  safeCell(victimPos,G);

    % Make sure start and goal are free.

    oMap(start(2),start(1)) = false;

    oMap(goal(2),goal(1)) = false;

    path =  araStarGrid(  oMap,start,goal,W);

    if isempty(path)

        path = [start;goal];

    end

    % Reduce unnecessary waypoints.

    path =  smoothPath(path,oMap);

    idx = 1;

end

%% =========================================================

% OPTIMIZED WEIGHTED A* CORE

%

% Uses array-based parent storage instead of containers.Map.

%

% 8-direction movement.

% Octile heuristic.

% ==========================================================

function path =  araStarGrid(map,start,goal,W)

    G = size(map,1);

    if isequal(start,goal)

        path = start;

        return;

    end

    %% 8-DIRECTIONAL MOVEMENT

    DIRS = [  1  0  1;

        -1  0  1;

         0  1  1;

         0 -1  1;

         1  1  sqrt(2);

        -1  1  sqrt(2);

         1 -1  sqrt(2);

        -1 -1  sqrt(2)];

    %% SEARCH ARRAYS

    closed = false(G,G);

    inOpen = false(G,G);

    gScore = inf(G,G);

    fScore = inf(G,G);

    parentX = zeros(G,G,'int16');

    parentY = zeros(G,G,'int16');

    %% START

    sx = start(1);

    sy = start(2);

    gx = goal(1);

    gy = goal(2);

    gScore(sy,sx) = 0;

    h0 = octileHeuristic(  sx,sy,gx,gy);

    fScore(sy,sx) =  W*h0;

    inOpen(sy,sx) = true;

    %% MAIN SEARCH

    while any(inOpen(:))

        tempF = fScore;

        tempF(~inOpen) = inf;

        [minF,linearIdx] =  min(tempF(:));

        if isinf(minF)

            break;

        end

        [cy,cx] =  ind2sub([G G],linearIdx);

        inOpen(cy,cx) = false;

        if closed(cy,cx)

            continue;

        end

        %% GOAL FOUND

        if cx == gx && cy == gy

            path =  reconstructPathArray(  parentX,  parentY,  start,  goal);

            return;

        end

        closed(cy,cx) = true;

        currentG =  gScore(cy,cx);

        %% EXPAND NEIGHBOURS

        for k = 1:8

            dx = DIRS(k,1);

            dy = DIRS(k,2);

            stepCost = DIRS(k,3);

            nx = cx + dx;

            ny = cy + dy;

            %% BOUNDS

            if nx < 1 || nx > G ||  ny < 1 || ny > G

                continue;

            end

            %% OBSTACLE

            if map(ny,nx)

                continue;

            end

            %% DIAGONAL CORNER SAFETY

            if stepCost > 1

                if map(cy,nx) ||  map(ny,cx)

                    continue;

                end

            end

            %% CLOSED

            if closed(ny,nx)

                continue;

            end

            tentativeG =  currentG + stepCost;

            if tentativeG <  gScore(ny,nx)

                parentX(ny,nx) = int16(cx);

                parentY(ny,nx) = int16(cy);

                gScore(ny,nx) =  tentativeG;

                h =  octileHeuristic(  nx,ny,gx,gy);

                fScore(ny,nx) =  tentativeG  + W*h;

                inOpen(ny,nx) = true;

            end

        end

    end

    path = [];

end

%% =========================================================

% OCTILE HEURISTIC

%

% More suitable than Euclidean/Manhattan

% for 8-direction movement.

% ==========================================================

function h =  octileHeuristic(x,y,gx,gy)

    dx = abs(x-gx);

    dy = abs(y-gy);

    h =  max(dx,dy)  + (sqrt(2)-1)*min(dx,dy);

end

%% =========================================================

% RECONSTRUCT PATH USING FIXED ARRAYS

% ==========================================================

function path =  reconstructPathArray(  parentX,parentY,start,goal)

    current = goal;

    path = double(goal);

    maxSteps =  numel(parentX);

    count = 0;

    while ~isequal(current,start)

        count = count + 1;

        if count > maxSteps

            path = [];

            return;

        end

        x = current(1);

        y = current(2);

        px = double(parentX(y,x));

        py = double(parentY(y,x));

        if px == 0 || py == 0

            path = [];

            return;

        end

        current = [px py];

        path =  [current;path];

    end

end

%% =========================================================

% PATH SMOOTHING

% ==========================================================

function smooth =  smoothPath(path,map)

    if size(path,1) <= 2

        smooth = path;

        return;

    end

    smooth = path(1,:);

    current = 1;

    N = size(path,1);

    while current < N

        next = N;

        while next > current+1

            if collisionFree(  map,  path(current,:),  path(next,:))

                break;

            end

            next = next - 1;

        end

        smooth =  [smooth;path(next,:)];

        current = next;

    end

end

%% =========================================================

% COLLISION FREE LINE CHECK

% ==========================================================

function ok =  collisionFree(map,p1,p2)

    ok = true;

    dist = norm(p2-p1);

    steps =  max(1,ceil(dist*2));

    for i = 0:steps

        alpha = i/steps;

        pt =  round(  p1 +  (p2-p1)*alpha);

        pt(1) =  max(1,  min(size(map,2),pt(1)));

        pt(2) =  max(1,  min(size(map,1),pt(2)));

        if map(pt(2),pt(1))

            ok = false;

            return;

        end

    end

end
