clc; clear; close all;

% Boat stabilization parameters. Tune with matched disturbance tests.
STAB_YAW_DAMPING = 8.0;
STAB_TORQUE_MAX = 60.0;

cd(fileparts(mfilename('fullpath')));

rng(1);   % Ensures same random map every run

%% =========================================================

%  DYNAMIC FLOOD RESCUE

%  ADAPTIVE PID + DYNAMIC WATER CURRENT

%  RRT + PID + DIFFERENTIAL-THRUST BOAT DYNAMICS

%

%  Features:

%    - RRT path planning

%    - Dynamic replanning

%    - PID heading control

%    - PID speed control

%    - Differential-thrust boat dynamics

%    - Dynamic wind and water current

%    - Moving debris and victims

%    - Drone victim detection

%    - Danger-based rescue priority

%    - Preemptive reassignment

%    - Boat collision avoidance

%    - All boats return to the mother ship after rescue operations

% =========================================================

%% ===================== CONFIG ============================

GRID         = 70;

DT           = 0.25;

N_VICTIMS    = 8;

N_DRONES     = 3;

N_BOATS      = 4;

N_DEBRIS     = 130;

waterFlow    = [0.035 0.022];

%% ================= DYNAMIC WATER CURRENT ===================

waterBase          = [0.035 0.022];

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

%% ================= DYNAMIC WIND ==========================

wind.angle      = 0.0;

wind.speed      = 0.045;

wind.turnRate   = 0.008;

wind.turnAccel  = 0.0006;

wind.turnVel    = 0.0;

wind.maxSpeed   = 0.085;

wind.minSpeed   = 0.010;

wind.speedDrift = 0.0012;

wind.vec = wind.speed * [cos(wind.angle) sin(wind.angle)];

%% ================= TURBULENCE ============================

turb.debris = 0.075;

turb.victim = 0.038;

turb.drone  = 0.010;

%% ================= BOAT SPEED ============================

SPEED_CRITICAL = 8.5;

SPEED_HIGH     = 6.5;

SPEED_MEDIUM   = 4.5;

SPEED_LOW      = 3.0;

MAX_SPEED      = SPEED_CRITICAL;

MIN_SPEED      = 0.70;

% Dynamic speed-reference tuning

SPEED_DIST_RADIUS     = 5.0;

SPEED_TURN_GAIN       = 0.65;

SPEED_OBSTACLE_RADIUS = 6.0;

SPEED_DISTURB_GAIN    = 1.20;

% Speed-reference slew limits

SPEED_ACCEL_LIMIT = 0.90;

SPEED_DECEL_LIMIT = 1.40;

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

%% ================= ADAPTIVE PID GAIN =======================

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

%% ================= RRT TUNING ===============================

RRT_MAX_ITER       = 2500;
RRT_STEP           = 3.0;
RRT_GOAL_RADIUS    = 3.0;
RRT_GOAL_BIAS      = 0.25;
RRT_DIRECTION_BIAS = 0.15;

%% ================= REPLANNING ============================

VICTIM_MOVE_THRESH     = 2.5;

REPLAN_INTERVAL        = 8.0;    % seconds

CURRENT_REPLAN_THRESH  = 0.010;

WIND_REPLAN_THRESH     = 0.010;

BOAT_DEVIATION_THRESH  = 2.0;

%% ================= COLLISION AVOIDANCE ===================

BOAT_SEP_RADIUS = 3.5;

BOAT_SEP_FORCE  = 2.2;

%% ================= DANGER SCORING ========================

DANGER_DEBRIS_RADIUS = 5.0;

CRIT_THRESH = 12;

HIGH_THRESH = 7;

MED_THRESH  = 3;

%% ================= PREEMPTION =============================

PREEMPT_ENABLE = true;

%% ================= DRONE SETTINGS =========================

DRONE_SCAN_BASE   = 30;

DRONE_SCAN_GROWTH = 4;

%% ================= INITIALISE =============================

outDir = fullfile(fileparts(mfilename('fullpath')),'web_dashboard');

if ~exist(outDir, 'dir')

mkdir(outDir);

end

%% ================= CREATE DEBRIS ==========================

debrisPos = double([ randi([3 GRID-2], N_DEBRIS, 1), randi([3 GRID-2], N_DEBRIS, 1)]);

%% ================= CREATE VICTIMS =========================

victims = struct();

for v = 1:N_VICTIMS

while true

    p = [ randi([6 GRID-5]), randi([6 GRID-5])];

    if ~tooClose(p, debrisPos, 5)

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

debrisPos = clearAround(debrisPos,p,5);

end

%% ================= CREATE DRONES ==========================

bandH = GRID / N_DRONES;

drones = struct();

for d = 1:N_DRONES

cy = round(bandH*(d-1) + bandH/2);

drones(d).pos = double([4 cy]);

drones(d).dir = 1;

drones(d).scanR = DRONE_SCAN_BASE;

drones(d).yLo = max(1,round(bandH*(d-1)+1));

drones(d).yHi = min(GRID,round(bandH*d));

end

%% ================= CREATE BOATS ===========================

BSTARTS = [ 2 2;

2 GRID-1;

GRID-1 2;

GRID-1 GRID-1];

% ================= MOTHER SHIP ==========================

% All boats return here after all rescue operations are complete.

MOTHERSHIP_POS = [round(GRID/2) GRID-3];

MOTHERSHIP_RADIUS = 2.5;

boats = struct();

for b = 1:N_BOATS

boats(b).pos = double(BSTARTS(b,:));

boats(b).assigned = false;

boats(b).victimIdx = 0;

boats(b).path = [];

boats(b).pathIdx = 1;

boats(b).speed = SPEED_LOW;

boats(b).speedDes = SPEED_LOW;

boats(b).speedError = 0;

boats(b).preempted = false;

% ================= PID STATES =================

boats(b).psi = 0;

boats(b).vel = 0;

boats(b).r = 0;

boats(b).headingI = 0;

boats(b).prevHeadE = 0;

boats(b).speedI = 0;

boats(b).prevSpeedE = 0;

% ================= CONTROL OUTPUTS ============

boats(b).Fcmd = 0;

boats(b).tauCmd = 0;
    boats(b).tauStabilization = 0;
    boats(b).stabilizationEffortPct = 0;
    boats(b).headingErrorDeg = 0;

boats(b).TL = 0;

boats(b).TR = 0;

boats(b).trackError = 0;

boats(b).KpHeading = PID_KP_HEADING;

boats(b).KiHeading = PID_KI_HEADING;

boats(b).KdHeading = PID_KD_HEADING;

boats(b).KpSpeed = PID_KP_SPEED;

boats(b).KiSpeed = PID_KI_SPEED;

boats(b).KdSpeed = PID_KD_SPEED;

boats(b).pidGainFactor = 1.0;

boats(b).waterDisturbance = 0;

boats(b).waterSeverity = 0;

% ================= DYNAMIC REPLANNING STATE ================

boats(b).lastPlanPos     = boats(b).pos;

boats(b).lastPlanCurrent = waterFlow;

boats(b).lastPlanWind    = wind.vec;

boats(b).lastPlanTime    = 0;

boats(b).planCount       = 0;

% ================= MOTHER-SHIP RETURN STATE =================

boats(b).returning = false;

boats(b).returnedToMothership = false;

debrisPos = clearAround(debrisPos, boats(b).pos, 4);

end

debrisPos = clearAround(debrisPos, MOTHERSHIP_POS, 6);

%% ================= METRICS ================================

boatRescueCounts = zeros(1, N_BOATS);

totalPathLength = 0;

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

history.boatX = [];
history.boatY = [];
%% ================= MAIN LOOP ==============================

t = 0;

nRescued = 0;

nDead = 0;

fprintf('=== Flood Rescue RRT ===\n');

fprintf('Open http\://localhost:8000\n\n');

while ((nRescued + nDead) < N_VICTIMS || any(~[boats.returnedToMothership])) && t < 470

t = t + 1;

%% =====================================================

% 1. DYNAMIC WIND PHYSICS

% ======================================================

wind.turnVel =     wind.turnVel + wind.turnAccel * (rand()-0.5)*2;

wind.turnVel =  max(-wind.turnRate,    min(wind.turnRate, wind.turnVel));

wind.angle =   wind.angle + wind.turnVel;

wind.speed =    wind.speed + wind.speedDrift * (rand()-0.5)*2;

wind.speed =     max(wind.minSpeed,  min(wind.maxSpeed, wind.speed));

wind.vec =  wind.speed *     [cos(wind.angle)     sin(wind.angle)];

%% ================= DYNAMIC FLOOD-WATER CURRENT ============

waveX = waterWaveAmp * sin(waterWaveFreq(1) * t);

waveY = waterWaveAmp * cos(waterWaveFreq(2) * t);

waveComponent = [waveX waveY];

gustComponent = waterGustAmp * (rand(1,2) - 0.5);

if rand() < waterShockChance

    shockDirection = rand(1,2) - 0.5;

    if norm(shockDirection) < 1e-6

        shockDirection = [1 0];

    end

    shockComponent = waterShockAmp * shockDirection / norm(shockDirection);

else

    shockComponent = [0 0];

end

waterFlow = waterBase + waveComponent + gustComponent + shockComponent;

waterMag = norm(waterFlow);

if waterMag > waterMaxMagnitude

    waterFlow = waterFlow / waterMag * waterMaxMagnitude;

end

waterFlowRate = (waterFlow - waterFlowPrev) / DT;

% Numerical safety: replace any accidental non-finite current values.

if any(~isfinite(waterFlow))

    waterFlow = waterBase;

end

if any(~isfinite(waterFlowRate))

    waterFlowRate = [0 0];

end

waterDisturbance = norm(waterFlow) + waterChangeGain * norm(waterFlowRate);

waterSeverity = min(1, waterDisturbance / waterDisturbRef);

waterFlowPrev = waterFlow;

%% =====================================================

% 2. ENVIRONMENT PHYSICS

% ======================================================

% ---------------- DEBRIS ----------------

nD = size(debrisPos,1);

tj = turb.debris *     (rand(nD,2)-0.5);

debrisPos = debrisPos + repmat(waterFlow,nD,1) + repmat(wind.vec*0.65,nD,1) + tj;

debrisPos(:,1) =  max(1,     min(double(GRID),     debrisPos(:,1)));

debrisPos(:,2) =     max(1,   min(double(GRID),    debrisPos(:,2)));

% ---------------- VICTIMS ----------------

for v = 1:N_VICTIMS

    if victims(v).rescued ||  victims(v).dead

        continue;

    end

    tj = turb.victim *  (rand(1,2)-0.5);    newPos =  victims(v).pos         + waterFlow         + wind.vec*0.5        + tj;

    if newPos(1)<1 ||       newPos(1)>GRID || newPos(2)<1 ||        newPos(2)>GRID

        victims(v).dead = true;

        if victims(v).assignedB ~= 0

            b = victims(v).assignedB;

            boats(b).assigned = false;

            boats(b).victimIdx = 0;

            boats(b).path = [];

            boats(b).pathIdx = 1;

            boats(b).speed = SPEED_LOW;

            boats(b).preempted = false;

            boats(b) = resetBoatControl(boats(b), MIN_SPEED);

            victims(v).assignedB = 0;

        end

        nDead = nDead + 1;

        for d = 1:N_DRONES

            drones(d).scanR =                 DRONE_SCAN_BASE + nDead *                DRONE_SCAN_GROWTH;

        end

        continue;

    end

    victims(v).pos = newPos;

end

% ---------------- DRONES -----------------

for d = 1:N_DRONES

    tj = turb.drone *         (rand(1,2)-0.5);

    drones(d).pos(1) =       drones(d).pos(1)         + drones(d).dir*0.5        + wind.vec(1)*0.10         + tj(1);

    drones(d).pos(2) =    drones(d).pos(2)   + wind.vec(2)*0.10         + tj(2);

    drones(d).pos(2) = max(drones(d).yLo, min(drones(d).yHi, drones(d).pos(2)));

    if drones(d).pos(1) > GRID-2

        drones(d).dir = -1;

    end

    if drones(d).pos(1) < 2

        drones(d).dir = 1;

    end

    drones(d).pos = clampPos( drones(d).pos, GRID);

end

%% =====================================================

% 3. UPDATE DANGER SCORES

% ======================================================

for v = 1:N_VICTIMS

    if victims(v).rescued || victims(v).dead

        continue;

    end

    victims(v).prevLvl = victims(v).dangerLvl;

    vp = victims(v).pos;

    dists = sqrt( (debrisPos(:,1)-vp(1)).^2 + (debrisPos(:,2)-vp(2)).^2);

    nNear = sum(dists < DANGER_DEBRIS_RADIUS);

    margin = min([ vp(1)-1, GRID-vp(1), vp(2)-1, GRID-vp(2)]);

    bPenalty = max(0,10-margin);

    score = nNear + bPenalty;

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

% 4. BUILD OBSTACLE MAP

% ======================================================

oMap = buildObstacleMap( debrisPos, GRID);

for b = 1:N_BOATS

    bp = safeCell( boats(b).pos, GRID);

    oMap(bp(2),bp(1)) = 0;

end

ms = safeCell(MOTHERSHIP_POS, GRID);

oMap(ms(2),ms(1)) = 0;

for v = 1:N_VICTIMS

    if ~victims(v).rescued && ~victims(v).dead

        vp = safeCell( victims(v).pos, GRID);

        oMap(vp(2),vp(1)) = 0;

    end

end

%% =====================================================

% 5. DRONE DETECTION

% ======================================================

for d = 1:N_DRONES

    for v = 1:N_VICTIMS

        if victims(v).rescued || victims(v).dead || victims(v).detected

            continue;

        end

        if norm( drones(d).pos - victims(v).pos) <= drones(d).scanR

            victims(v).detected = true;

        end

    end

end

%% =====================================================

% 6. UPDATE BOAT SPEEDS

% ======================================================

for b = 1:N_BOATS

    if ~boats(b).assigned

        continue;

    end

    v = boats(b).victimIdx;

    boats(b).speed = dangerSpeed( victims(v).dangerLvl, SPEED_CRITICAL, SPEED_HIGH, SPEED_MEDIUM, SPEED_LOW);

end

%% =====================================================

% 7. PREEMPTIVE REASSIGNMENT

% ======================================================

if PREEMPT_ENABLE

    bestV = 0;

    bestScore = -1;

    for v = 1:N_VICTIMS

        if victims(v).rescued || victims(v).dead || ~victims(v).detected

            continue;

        end

        if victims(v).assignedB ~= 0

            continue;

        end

        if victims(v).danger > bestScore

            bestScore = victims(v).danger;

            bestV = v;

        end

    end

    if bestV > 0 && bestScore >= HIGH_THRESH

        for b = 1:N_BOATS

            if ~boats(b).assigned

                continue;

            end

            cv = boats(b).victimIdx;

            if dangerRank( victims(bestV).dangerLvl) > dangerRank( victims(cv).dangerLvl) + 1

                victims(cv).assignedB = 0;

                boats(b).victimIdx = bestV;

                victims(bestV).assignedB = b;

                boats(b).preempted = true;

                [boats(b).path, boats(b).pathIdx] = planPathRRT(oMap, boats(b).pos, victims(bestV).pos, RRT_MAX_ITER, RRT_STEP, RRT_GOAL_RADIUS, RRT_GOAL_BIAS, RRT_DIRECTION_BIAS);

                boats(b).speed = dangerSpeed( victims(bestV).dangerLvl, SPEED_CRITICAL, SPEED_HIGH, SPEED_MEDIUM, SPEED_LOW);

                boats(b) = resetBoatControl( boats(b), MIN_SPEED);

                break;

            end

        end

    end

end

%% =====================================================

% 8. PRIORITY DISPATCH

% ======================================================

freeBoats = [];

for b = 1:N_BOATS

    if ~boats(b).assigned && ~boats(b).returning && ~boats(b).returnedToMothership

        freeBoats(end+1) = b;

    end

end

if ~isempty(freeBoats)

    unassignedV = [];

    unassignedS = [];

    for v = 1:N_VICTIMS

        if victims(v).rescued || victims(v).dead || ~victims(v).detected

            continue;

        end

        if victims(v).assignedB ~= 0

            continue;

        end

        unassignedV(end+1) = v;

        unassignedS(end+1) = victims(v).danger;

    end

    [~,si] = sort(unassignedS,'descend');

    unassignedV = unassignedV(si);

    for vi = 1:min( length(unassignedV), length(freeBoats))

        v = unassignedV(vi);

        bestB = freeBoats(1);

        bestD = inf;

        for fb = freeBoats

            dd = norm( boats(fb).pos - victims(v).pos);

            if dd < bestD

                bestD = dd;

                bestB = fb;

            end

        end

        boats(bestB).assigned = true;

        boats(bestB).victimIdx = v;

        boats(bestB).preempted = false;

        boats(bestB).speed = dangerSpeed( victims(v).dangerLvl, SPEED_CRITICAL, SPEED_HIGH, SPEED_MEDIUM, SPEED_LOW);

        victims(v).assignedB = bestB;

        [boats(bestB).path, boats(bestB).pathIdx] = planPathRRT(oMap, boats(bestB).pos, victims(v).pos, RRT_MAX_ITER, RRT_STEP, RRT_GOAL_RADIUS, RRT_GOAL_BIAS, RRT_DIRECTION_BIAS);

        boats(bestB) = resetBoatControl( boats(bestB), MIN_SPEED);

        freeBoats( freeBoats == bestB) = [];

    end

end

%% =====================================================

% 9. START MOTHER-SHIP RETURN AFTER RESCUE OPERATIONS

% ======================================================

allVictimsFinished = (nRescued + nDead) >= N_VICTIMS;

if allVictimsFinished

    for b = 1:N_BOATS

        if ~boats(b).assigned && ~boats(b).returning && ~boats(b).returnedToMothership

            boats(b).returning = true;

            boats(b).victimIdx = 0;

            boats(b).path = [];

            boats(b).pathIdx = 1;

            boats(b).speed = SPEED_LOW;

            boats(b).speedDes = SPEED_LOW;

            boats(b).preempted = false;

            boats(b) = resetBoatControl( boats(b), MIN_SPEED);

            [returnPath,returnIdx] = planPathRRT(oMap, boats(b).pos, MOTHERSHIP_POS, RRT_MAX_ITER, RRT_STEP, RRT_GOAL_RADIUS, RRT_GOAL_BIAS, RRT_DIRECTION_BIAS);

            if ~isempty(returnPath)

                boats(b).path = returnPath;

                boats(b).pathIdx = returnIdx;

                boats(b).lastPlanPos = boats(b).pos;

                boats(b).lastPlanCurrent = waterFlow;

                boats(b).lastPlanWind = wind.vec;

                boats(b).lastPlanTime = t*DT;

                boats(b).planCount = boats(b).planCount + 1;

            end

        end

    end

end

%% =====================================================

% 10. BOAT MOVEMENT

% ======================================================

for b = 1:N_BOATS

    if ~boats(b).assigned && ~boats(b).returning

        continue;

    end

    isReturning = boats(b).returning;

    if isReturning

        targetPos = MOTHERSHIP_POS;

    else

        v = boats(b).victimIdx;

        %% ------------ TARGET DIED ------------

        if v <= 0 || victims(v).dead

            boats(b).assigned = false;

            boats(b).victimIdx = 0;

            boats(b).path = [];

            boats(b).pathIdx = 1;

            boats(b).speed = SPEED_LOW;

            boats(b).preempted = false;

            boats(b).returning = false;

            boats(b) = resetBoatControl( boats(b), MIN_SPEED);

            continue;

        end

        targetPos = victims(v).pos;

    end

    %% ------------ CHECK PATH ------------

    pathBlocked = isempty(boats(b).path);

    if ~pathBlocked

        pi0 = min( boats(b).pathIdx, size(boats(b).path,1));

        rem = boats(b).path(pi0:end,:);

        for pi = 1:size(rem,1)

            wp = safeCell( rem(pi,:), GRID);

            if oMap(wp(2),wp(1)) == 1

                pathBlocked = true;

                break;

            end

        end

    end

    if isReturning

        targetMoved = isempty(boats(b).path) || norm(double(boats(b).path(end,:)) - MOTHERSHIP_POS) > 1.0;

    else

        targetMoved = isempty(boats(b).path) || norm(double(boats(b).path(end,:)) - targetPos) > VICTIM_MOVE_THRESH;

    end

    currentChanged = norm(waterFlow - boats(b).lastPlanCurrent) > CURRENT_REPLAN_THRESH;

    windChanged = norm(wind.vec - boats(b).lastPlanWind) > WIND_REPLAN_THRESH;

    boatDeviation = norm(boats(b).pos - boats(b).lastPlanPos) > BOAT_DEVIATION_THRESH;

    periodicReplan = (t*DT - boats(b).lastPlanTime) >= REPLAN_INTERVAL;

    replanNeeded = pathBlocked || targetMoved || currentChanged || windChanged || boatDeviation || periodicReplan;

    %% ------------ REPLAN WITH A* -----------

    if replanNeeded

        [newPath,newIdx] = planPathRRT(oMap, boats(b).pos, targetPos, RRT_MAX_ITER, RRT_STEP, RRT_GOAL_RADIUS, RRT_GOAL_BIAS, RRT_DIRECTION_BIAS);

        if ~isempty(newPath)

            boats(b).path = newPath;

            boats(b).pathIdx = newIdx;

            boats(b).lastPlanPos = boats(b).pos;

            boats(b).lastPlanCurrent = waterFlow;

            boats(b).lastPlanWind = wind.vec;

            boats(b).lastPlanTime = t*DT;

            boats(b).planCount = boats(b).planCount + 1;

        end

    end

    %% =====================================

    % PID WAYPOINT TRACKING

    % ======================================

    if ~isempty(boats(b).path) && boats(b).pathIdx <= size(boats(b).path,1)

        target = double( boats(b).path( boats(b).pathIdx,:));

        dir = target - boats(b).pos;

        dist = norm(dir);

        boats(b).trackError = dist;

        if dist < WAYPOINT_TOL

            boats(b).pathIdx = boats(b).pathIdx + 1;

        else

            oldPos = boats(b).pos;

            %% =================================

            % GUIDANCE

            % ==================================

            psiDes = atan2(dir(2),dir(1));

            % DYNAMIC SPEED REFERENCE

            headE = wrapAngle( psiDes - boats(b).psi);

            nearestDebris = nearestObstacleDistance( boats(b).pos, debrisPos);

            % Disturbance used for adaptive PID gain scheduling.

            disturbance = norm( waterFlow*BOAT_CURRENT + wind.vec*BOAT_WIND_GAIN) + waterChangeGain*norm(waterFlowRate);

            pidGainFactor = PID_HEADING_SCALE_MIN + waterSeverity*(PID_HEADING_SCALE_MAX-PID_HEADING_SCALE_MIN);

            KpHeading = PID_KP_HEADING*pidGainFactor;

            KdHeading = PID_KD_HEADING*pidGainFactor;

            KiHeading = PID_KI_HEADING*(1-PID_KI_REDUCTION*waterSeverity);

            speedGainFactor = PID_SPEED_SCALE_MIN + waterSeverity*(PID_SPEED_SCALE_MAX-PID_SPEED_SCALE_MIN);

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

            speedDesRaw = dynamicSpeedTarget( boats(b).speed, dist, headE, nearestDebris, disturbance, SPEED_DIST_RADIUS, SPEED_TURN_GAIN, SPEED_OBSTACLE_RADIUS, SPEED_DISTURB_GAIN, MIN_SPEED);

            % Slew-limit speed reference for realistic acceleration.

            speedDelta = speedDesRaw - boats(b).speedDes;

            if speedDelta >= 0

                speedDelta = min(speedDelta, SPEED_ACCEL_LIMIT*DT);

            else

                speedDelta = max(speedDelta, -SPEED_DECEL_LIMIT*DT);

            end

            boats(b).speedDes = max(MIN_SPEED, min(MAX_SPEED, boats(b).speedDes + speedDelta));

            speedDes = boats(b).speedDes;

            %% =================================

            % HEADING PID

            % ==================================

            boats(b).headingI = max( -HEADING_I_MAX, min( HEADING_I_MAX, boats(b).headingI + headE*DT));

            headD = wrapAngle( headE - boats(b).prevHeadE) / DT;

            tauCmd = KpHeading*headE + KiHeading*boats(b).headingI + KdHeading*headD;

            % Additional yaw-rate damping stabilizes unwanted boat rotation.
            tauPID = tauCmd;
            tauStab = -STAB_YAW_DAMPING * boats(b).r;
            tauStab = max(-STAB_TORQUE_MAX, min(STAB_TORQUE_MAX, tauStab));
            tauCmd = tauPID + tauStab;
            boats(b).tauStabilization = tauStab;
            boats(b).stabilizationEffortPct = 100 * abs(tauStab) / max(STAB_TORQUE_MAX, eps);
            boats(b).headingErrorDeg = rad2deg(headE);


            boats(b).prevHeadE = headE;

            %% =================================

            % SPEED PID

            % ==================================

            speedE = speedDes - boats(b).vel;

            boats(b).speedError = speedE;

            boats(b).speedI = max( -SPEED_I_MAX, min( SPEED_I_MAX, boats(b).speedI + speedE*DT));

            speedD = (speedE - boats(b).prevSpeedE) / DT;

            % Additional speed-error scheduling on top of water-adaptive gains.

            speedErrorScale = 1 + 0.25*min(1,abs(speedE)/MAX_SPEED);

            KpSpeed = KpSpeedAdaptive*speedErrorScale;

            KdSpeed = KdSpeedAdaptive*(1+0.10*min(1,abs(speedD)));

            KiSpeed = KiSpeedAdaptive;

            boats(b).KpSpeed = KpSpeed;

            boats(b).KiSpeed = KiSpeed;

            boats(b).KdSpeed = KdSpeed;

            Fcmd = max(0, KpSpeed*speedE + KiSpeed*boats(b).speedI + KdSpeed*speedD);

            boats(b).prevSpeedE = speedE;

            %% =================================

            % DIFFERENTIAL THRUSTER MIXER

            % ==================================

            TL = 0.5*Fcmd - tauCmd/THRUSTER_SEP;

            TR = 0.5*Fcmd + tauCmd/THRUSTER_SEP;

            TL = max(-TMAX, min(TMAX,TL));

            TR = max(-TMAX, min(TMAX,TR));

            F = TL + TR;

            tau = (TR-TL) * THRUSTER_SEP / 2;

            boats(b).Fcmd = Fcmd;

            boats(b).tauCmd = tauCmd;

            boats(b).TL = TL;

            boats(b).TR = TR;

            %% =================================

            % BOAT DYNAMICS

            %

            % m*v_dot = F - Dv*v

            %

            % Iz*r_dot = tau - Dr*r

            % ==================================

            vDot = (F - DRAG_V*boats(b).vel) / BOAT_MASS;

            rDot = (tau - DRAG_R*boats(b).r) / BOAT_IZ;

            boats(b).vel = max(0, min(MAX_SPEED, boats(b).vel + vDot*DT));

            boats(b).r = boats(b).r + rDot*DT;

            boats(b).psi = wrapAngle( boats(b).psi + boats(b).r*DT);

            %% =================================

            % WORLD MOTION

            % ==================================

            bodyVel = boats(b).vel * [cos(boats(b).psi), sin(boats(b).psi)];

            currentVel = waterFlow * BOAT_CURRENT;

            windVel = wind.vec * BOAT_WIND_GAIN;

            boats(b).pos = boats(b).pos + (bodyVel + currentVel + windVel) * DT;

            totalPathLength = totalPathLength + norm( boats(b).pos - oldPos);

        end

    end

    %% =====================================

    % BOAT-BOAT COLLISION AVOIDANCE

    % ======================================

    sepForce = [0 0];

    for ob = 1:N_BOATS

        if ob == b

            continue;

        end

        diff = boats(b).pos - boats(ob).pos;

        d = norm(diff);

        if d < BOAT_SEP_RADIUS && d > 1e-4

            pushMag = BOAT_SEP_FORCE * (1 - d/BOAT_SEP_RADIUS);

            sepForce = sepForce + (diff/d) * pushMag;

        end

    end

    boats(b).pos = boats(b).pos + sepForce*DT;

    boats(b).pos = clampPos( boats(b).pos, GRID);

    %% =====================================

    % RESCUE / MOTHER-SHIP ARRIVAL CHECK

    % ======================================

    if ~isReturning && norm(boats(b).pos - targetPos) < 1.8

        victims(v).rescued = true;

        victims(v).assignedB = 0;

        boats(b).assigned = false;

        boats(b).victimIdx = 0;

        boats(b).path = [];

        boats(b).pathIdx = 1;

        boats(b).speed = SPEED_LOW;

        boats(b).speedDes = SPEED_LOW;

        boats(b).preempted = false;

        boats(b) = resetBoatControl( boats(b), MIN_SPEED);

        nRescued = nRescued + 1;

        boatRescueCounts(b) = boatRescueCounts(b) + 1;

    elseif isReturning && norm(boats(b).pos - MOTHERSHIP_POS) < MOTHERSHIP_RADIUS

        boats(b).pos = MOTHERSHIP_POS;

        boats(b).returning = false;

        boats(b).returnedToMothership = true;

        boats(b).path = [];

        boats(b).pathIdx = 1;

        boats(b).speed = 0;

        boats(b).speedDes = 0;

        boats(b).vel = 0;

        boats(b).r = 0;

        boats(b) = resetBoatControl( boats(b), 0);

    end

end

%% =====================================================

% 10. EXPORT JSON

% ======================================================

vDat = struct();

for v = 1:N_VICTIMS

    vDat(v).id = v;

    vDat(v).x = victims(v).pos(1);

    vDat(v).y = victims(v).pos(2);

    vDat(v).lvl = victims(v).dangerLvl;

    vDat(v).danger = victims(v).danger;

    vDat(v).dead = victims(v).dead;

    vDat(v).rescued = victims(v).rescued;

    vDat(v).detected = victims(v).detected;

end

%% ---------------- BOAT DATA ----------------

bDat = struct();

for b = 1:N_BOATS

    bDat(b).id = b;

    bDat(b).x = boats(b).pos(1);

    bDat(b).y = boats(b).pos(2);

    bDat(b).target = boats(b).victimIdx;

    bDat(b).speed = boats(b).speed;

    bDat(b).preempted = boats(b).preempted;

    bDat(b).rescues = boatRescueCounts(b);

    bDat(b).heading = rad2deg(boats(b).psi);

    bDat(b).velocity = boats(b).vel;

    bDat(b).speedTarget = boats(b).speedDes;

    bDat(b).speedError = boats(b).speedError;

    bDat(b).yawRate = boats(b).r;

    bDat(b).Fcmd = boats(b).Fcmd;

    bDat(b).tauCmd = boats(b).tauCmd;
    bDat(b).tauStabilization = boats(b).tauStabilization;
    bDat(b).stabilizationEffortPct = boats(b).stabilizationEffortPct;
    bDat(b).headingErrorDeg = boats(b).headingErrorDeg;

    bDat(b).TL = boats(b).TL;

    bDat(b).TR = boats(b).TR;

    bDat(b).trackErr = boats(b).trackError;

    % Adaptive PID and water telemetry for dashboard plotting.

    bDat(b).KpHeading = boats(b).KpHeading;

    bDat(b).KiHeading = boats(b).KiHeading;

    bDat(b).KdHeading = boats(b).KdHeading;

    bDat(b).KpSpeed = boats(b).KpSpeed;

    bDat(b).KiSpeed = boats(b).KiSpeed;

    bDat(b).KdSpeed = boats(b).KdSpeed;

    bDat(b).pidGainFactor = boats(b).pidGainFactor;

    bDat(b).waterDisturbance = boats(b).waterDisturbance;

    bDat(b).waterSeverity = boats(b).waterSeverity;

    bDat(b).plans = boats(b).planCount;

    bDat(b).currentX = waterFlow(1);

    bDat(b).currentY = waterFlow(2);

    bDat(b).currentMagnitude = norm(waterFlow);

    bDat(b).currentRate = norm(waterFlowRate);

    if ~isempty(boats(b).path)

        bDat(b).path = boats(b).path;

    else

        bDat(b).path = [];

    end

end

%% ---------------- DRONE DATA ----------------

dDat = struct();

for d = 1:N_DRONES

    dDat(d).id = d;

    dDat(d).x = drones(d).pos(1);

    dDat(d).y = drones(d).pos(2);

    dDat(d).r = drones(d).scanR;

end

%% ---------------- STATE ---------------------

S.t = t;

S.wind = wind.speed;

S.windAng = rad2deg(wind.angle);

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

S.totalReplans = sum([boats.planCount]);

S.rescued = nRescued;

S.dead = nDead;

S.victims = vDat;

S.boats = bDat;

S.drones = dDat;

S.debris = debrisPos;

%% ---------------- WRITE JSON ----------------

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


history.boatX(end+1) = boats(1).pos(1);

history.boatY(end+1) = boats(1).pos(2);


pause(0.1);

end
%% ================= SAVE DYNAMIC HISTORY =================

save(fullfile(outDir,'rrt_dynamics.mat'),'history');

%% ================= SIMULATION COMPLETE ===================

fprintf( 'Simulation complete. Rescued: %d | Lost: %d | Returned to Mother Ship: %d/%d\n', nRescued, nDead, sum([boats.returnedToMothership]), N_BOATS);

metrics.algorithm = "RRT + Adaptive PID + Dynamic Water + Mother Ship";

metrics.time = t;

metrics.rescued = nRescued;

metrics.dead = nDead;

metrics.pathLength = totalPathLength;

metrics.avgPathLength = totalPathLength / max(nRescued,1);

metrics.pid = struct();

metrics.pid.baseKpHeading = PID_KP_HEADING;

metrics.pid.baseKiHeading = PID_KI_HEADING;

metrics.pid.baseKdHeading = PID_KD_HEADING;

metrics.pid.baseKpSpeed = PID_KP_SPEED;

metrics.pid.baseKiSpeed = PID_KI_SPEED;

metrics.pid.baseKdSpeed = PID_KD_SPEED;

metrics.pid.headingScaleMin = PID_HEADING_SCALE_MIN;

metrics.pid.headingScaleMax = PID_HEADING_SCALE_MAX;

metrics.pid.speedScaleMin = PID_SPEED_SCALE_MIN;

metrics.pid.speedScaleMax = PID_SPEED_SCALE_MAX;

metrics.pid.waterDisturbanceReference = waterDisturbRef;

metrics.pid.adaptive = true;

metrics.totalReplans = sum([boats.planCount]);

metrics.avgReplansPerBoat = mean([boats.planCount]);

metrics.motherShip = MOTHERSHIP_POS;

metrics.returnedBoats = sum([boats.returnedToMothership]);

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

fid = fopen( fullfile( outDir, 'rrt_metrics.json'), 'w');

fprintf(fid,'%s',jsonStr);

fclose(fid);

%% =========================================================

% HELPER FUNCTIONS

% ==========================================================

function map = buildObstacleMap(pos,G)

map = zeros(G,G);

xi = max(1, min(G, round(pos(:,1))));

yi = max(1, min(G, round(pos(:,2))));

map( sub2ind([G G],yi,xi)) = 1;

end

function p = clampPos(p,G)

p(1) = max(1, min(double(G),p(1)));

p(2) = max(1, min(double(G),p(2)));

end

function c = safeCell(p,G)

c = max(1, min(G, round(p)));

end

function dpos = clearAround(dpos,point,r)

d = sqrt( (dpos(:,1)-point(1)).^2 + (dpos(:,2)-point(2)).^2);

dpos(d<r,:) = [];

end

function result = tooClose(p,positions,r)

if isempty(positions)

    result = false;

    return;

end

d = sqrt( (positions(:,1)-p(1)).^2 + (positions(:,2)-p(2)).^2);

result = any(d<r);

end

function spd = dangerSpeed(lvl,sc,sh,sm,sl)

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

% DYNAMIC SPEED HELPERS

% ==========================================================

function d = nearestObstacleDistance(point,obstacles)

if isempty(obstacles)

    d = inf;

    return;

end

dx = obstacles(:,1) - point(1);

dy = obstacles(:,2) - point(2);

d = min(sqrt(dx.^2 + dy.^2));

end

function speedTarget = dynamicSpeedTarget( baseSpeed,dist,headingError,nearestDebris,disturbance, distRadius,turnGain,obstacleRadius,disturbanceGain,minSpeed)

distanceFactor = min(1,dist/distRadius);

turnFactor = 1 - turnGain*min(1,abs(headingError)/(pi/2));

if isinf(nearestDebris)

    obstacleFactor = 1;

else

    obstacleFactor = 0.45 + 0.55*min(1,nearestDebris/obstacleRadius);

end

disturbanceFactor = 1/(1 + disturbanceGain*disturbance);

speedTarget = baseSpeed * max(0.15,distanceFactor) * max(0.35,turnFactor) * max(0.45,obstacleFactor) * max(0.50,disturbanceFactor);

speedTarget = max(minSpeed,min(baseSpeed,speedTarget));

end

%% =========================================================

% RESET BOAT CONTROL

% ==========================================================

function boat = resetBoatControl(boat,minSpeed)

boat.vel = 0;

boat.r = 0;

boat.headingI = 0;

boat.prevHeadE = 0;

boat.speedI = 0;

boat.prevSpeedE = 0;

boat.speedDes = minSpeed;

boat.speedError = 0;

boat.Fcmd = 0;

boat.tauCmd = 0;

boat.TL = 0;

boat.TR = 0;

boat.trackError = 0;

end

function [path,idx] = planPathRRT(oMap,boatPos,targetPos,MAX_ITER,STEP,GOAL_RADIUS,GOAL_BIAS,DIRECTION_BIAS)

G = size(oMap,1);

s = max(1,min(G,round(boatPos)));
g = max(1,min(G,round(targetPos)));

% Ensure start and goal cells are free.
oMap(s(2),s(1)) = 0;
oMap(g(2),g(1)) = 0;

path = rrtGrid(oMap,s,g,MAX_ITER,STEP,GOAL_RADIUS,GOAL_BIAS,DIRECTION_BIAS);

% Fallback if RRT cannot find a path.
if isempty(path)
    path = [s;g];
end

% Remove unnecessary intermediate waypoints.
path = smoothPath(path,oMap);

idx = 1;

end

%% =========================================================
% STANDARD RRT GRID PATH PLANNER
%
% Includes:
%   1. Random sampling
%   2. Goal-biased sampling
%   3. Direction-biased sampling
%   4. Nearest-node search
%   5. Steer operation
%   6. Collision checking
%   7. Goal connection
%
% This is standard RRT, not RRT*:
% there is no rewiring and no best-parent optimization.
% ==========================================================

function path = rrtGrid(map,start,goal,MAX_ITER,STEP,GOAL_RADIUS, GOAL_BIAS,DIRECTION_BIAS)

G = size(map,1);

nodes = double(start);
parents = 0;

goalNode = 0;

for iter = 1:MAX_ITER

    % SAMPLE
    r = rand();

    if r < GOAL_BIAS

        sample = double(goal);

    elseif r < GOAL_BIAS + DIRECTION_BIAS

        alpha = rand();
        sample = double(start) + alpha *  (double(goal) - double(start));

        sample = sample + randn(1,2) * 3.0;

    else

        sample = [1 + rand() * (G - 1),  1 + rand() * (G - 1)];

    end

    sample = max(1,min(G,sample));

    % FIND NEAREST NODE
    dists = sqrt(sum((nodes-sample).^2,2));
    [~,nearestIdx] = min(dists);
    nearest = nodes(nearestIdx,:);

    % STEER
    direction = sample - nearest;
    d = norm(direction);

    if d < 1e-9
        continue;
    end

    stepLength = min(STEP,d);
    newNode = nearest + (direction/d) * stepLength;
    newNode = round(newNode);
    newNode = max(1,min(G,newNode));

    % Do not add duplicate nodes.
    if any(all(nodes == newNode,2))
        continue;
    end

    % Check the complete edge.
    if ~collisionFree(map,nearest,newNode)
        continue;
    end

    nodes(end+1,:) = newNode;
    parents(end+1,1) = nearestIdx;
    newIdx = size(nodes,1);

    % Try to connect directly to the goal.
    goalDistance = norm(newNode-double(goal));

    if goalDistance <= GOAL_RADIUS || goalDistance <= STEP

        if collisionFree(map,newNode,double(goal))
            nodes(end+1,:) = double(goal);
            parents(end+1,1) = newIdx;
            goalNode = size(nodes,1);
            break;
        end

    end

end

% RECONSTRUCT PATH
if goalNode == 0
    path = [];
    return;
end

path = reconstructRRTPath(nodes,parents,goalNode);

end

%% =========================================================
% RECONSTRUCT RRT PATH
% ==========================================================

function path = reconstructRRTPath(nodes,parents,goalIdx)

path = [];
current = goalIdx;

while current ~= 0
    path = [nodes(current,:);path];
    current = parents(current);
end

end

%% =========================================================

% COLLISION CHECK

%

% Checks whether a straight line between

% two points is obstacle free.

% ==========================================================

function ok = collisionFree(map,p1,p2)

ok = true;

dist = norm(p2-p1);

steps = max(1,ceil(dist*2));

for i = 0:steps

    alpha = i/steps;

    pt = round( p1 + (p2-p1)*alpha);

    pt(1) = max(1, min(size(map,2), pt(1)));

    pt(2) = max(1, min(size(map,1), pt(2)));

    if map(pt(2),pt(1)) == 1

        ok = false;

        return;

    end

end

end

%% =========================================================

% PATH SMOOTHING

%

% Removes unnecessary intermediate RRT waypoints.

% ==========================================================

function smooth = smoothPath(path,map)

if size(path,1) <= 2

    smooth = path;

    return;

end

smooth = path(1,:);

current = 1;

N = size(path,1);

while current < N

    next = N;

    while next > current + 1

        if collisionFree( map, path(current,:), path(next,:))

            break;

        end

        next = next - 1;

    end

    smooth = [smooth;path(next,:)];

    current = next;

end

end
