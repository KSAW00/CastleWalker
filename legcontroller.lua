local cfg = require("config")

rednet.open(cfg.modem)
-- Host a specific service so the Contraption Brain can find this leg automatically
rednet.host("leg_controllers", cfg.prefix .. "brain")

-- Direct local speed controller connections
local speedhip1 = peripheral.wrap("speedController_0")
local speedhip2 = peripheral.wrap("speedController_1")
local speedknee = peripheral.wrap("speedController_2")

local hip1ID = assert(rednet.lookup("joint", cfg.prefix .. cfg.joints.hip1))
local hip2ID = assert(rednet.lookup("joint", cfg.prefix .. cfg.joints.hip2))
local kneeID = assert(rednet.lookup("joint", cfg.prefix .. cfg.joints.knee))

------------------------------------------------------------------
-- ARCHITECTURE TRAJECTORY STATE TRACKING
------------------------------------------------------------------
-- Where the foot physically is right now
local current_pos = { x = cfg.rest_pos.x, y = cfg.rest_pos.y, z = cfg.rest_pos.z }
-- Where the foot started its current movement step
local start_pos   = { x = cfg.rest_pos.x, y = cfg.rest_pos.y, z = cfg.rest_pos.z }
-- Where the foot is trying to go
local target_pos  = { x = cfg.rest_pos.x, y = cfg.rest_pos.y, z = cfg.rest_pos.z }

-- Track angles from the previous loop turn to derive physical speeds
local last_angles = { yaw = 0, hip = 0, knee = 0 }

-- Progress Tracking parameters
local progress = 1.0       -- 1.0 means fully arrived and resting
local step_duration = 0.40 -- How many seconds a full leg movement should take (e.g., 8 ticks)
local elapsed_time = 0.0

------------------------------------------------------------------
-- HELPER FUNCTIONS
------------------------------------------------------------------
-- Finds the true shortest distance across a circular boundary (prevents 360° snaps)
local function getShortestDiff(target, current)
    return math.abs((target - current + 180) % 360 - 180)
end

------------------------------------------------------------------
-- CORE INVERSE KINEMATICS MATRIX
------------------------------------------------------------------
local function solveIK(x, y, z)
    -- 1. Hip 1 (XY plane, Y to X sweep)
    local t1 = math.atan2(x, y)
    local r_xy = math.sqrt(x * x + y * y)

    -- 2. Origin Translation
    local px = r_xy - cfg.length1
    local pz = z
    local d2 = px * px + pz * pz

    -- 3. Reachability Clamp
    local d = math.sqrt(d2)
    local maxReach = cfg.length2 + cfg.length3 - 1e-6
    if d > maxReach then
        x = x * maxReach / d
        z = z * maxReach / d
        r_xy = math.sqrt(x * x + y * y)
        px = r_xy - cfg.length1
        pz = z
        d2 = px * px + pz * pz
    end

    -- 4. Knee & Hip 2 Solutions (Elbow Up)
    local c3 = (d2 - cfg.length2^2 - cfg.length3^2) / (2 * cfg.length2 * cfg.length3)
    c3 = math.max(-1, math.min(1, c3))
    local s3 = math.sqrt(1 - c3 * c3)
    local t3 = math.atan2(s3, c3)

    local k1 = cfg.length2 + cfg.length3 * c3
    local k2 = cfg.length3 * s3
    local t2 = math.atan2(pz, px) - math.atan2(k2, k1)

    -- 5. Relative Chain De-rotation (t2 and t3 collinear)
    local rel_t1 = t1
    local rel_t2 = t2
    local rel_t3 = t3 - t2

    -- 6. Convert to calibrated physical degree outputs
    return -math.deg(rel_t1), math.deg(rel_t2), -math.deg(rel_t3)
end

------------------------------------------------------------------
-- MAIN LISTENER & EXECUTION LOOP
------------------------------------------------------------------
while true do
    -- 1. Check for new gait/coordinate instructions from the Contraption Brain without blocking
    local id, msg = rednet.receive("gait.command",0)
    
    if msg and msg.type == "move_to" then
        -- Snapshot our current coordinate position as the new starting baseline
        start_pos.x = current_pos.x
        start_pos.y = current_pos.y
        start_pos.z = current_pos.z
        
        -- Register the new spatial destination targets
        target_pos.x = msg.x or target_pos.x
        target_pos.y = msg.y or target_pos.y
        target_pos.z = msg.z or target_pos.z
        
        -- Reset our progress timer to start the new path interpolation
        progress = 0.0
        elapsed_time = 0.0
    end
    ------------------------------------------------------------------
    -- PROGRESS TRAJECTORY INTERPOLATION
    ------------------------------------------------------------------
    if progress < 1.0 then
        elapsed_time = elapsed_time + 0.05
        progress = math.min(1.0, elapsed_time / step_duration)

        -- CORES SMOOTHING FUNCTION: Cosine S-Curve
        -- This maps a flat linear step into an organic ramp-up and cushion-down profile
        local smooth_t = (1 - math.cos(progress * math.pi)) / 2

        -- Slide our active spatial coordinates along the calculated tracking vector
        current_pos.x = start_pos.x + (target_pos.x - start_pos.x) * smooth_t
        current_pos.y = start_pos.y + (target_pos.y - start_pos.y) * smooth_t
        current_pos.z = start_pos.z + (target_pos.z - start_pos.z) * smooth_t
    end

    ------------------------------------------------------------------
    -- RUN GEOMETRY GENERATOR
    ------------------------------------------------------------------
    local yaw, hip, knee = solveIK(current_pos.x, current_pos.y, current_pos.z)

    ------------------------------------------------------------------
    -- PROGRESS-RATIO REVOLUTION SPEED CALCULATOR
    ------------------------------------------------------------------
    local diff_yaw  = getShortestDiff(yaw, last_angles.yaw)
    local diff_hip  = getShortestDiff(hip, last_angles.hip)
    local diff_knee = getShortestDiff(knee, last_angles.knee)

    local rpm_yaw, rpm_hip, rpm_knee = 0, 0, 0

    -- If we are actively moving along the path, calculate proportional arrival seeds
    if progress < 1.0 or diff_yaw > 0.05 or diff_hip > 0.05 or diff_knee > 0.05 then
        -- Find the master link tracking bottleneck
        local max_diff = math.max(diff_yaw, math.max(diff_hip, diff_knee))
        
        if max_diff > 0.01 then
            -- Target an aggressive base multiplier for our master speed calculation
            local Kp = 4.5
            local master_speed = max_diff * Kp

            -- Scale each motor's RPM perfectly to matching ratios
            -- Clamp at 24 RPM to protect Create Mod physics frames from overloading
            rpm_yaw  = math.min(math.max(2, (diff_yaw / max_diff) * master_speed), 24)
            rpm_hip  = math.min(math.max(2, (diff_hip / max_diff) * master_speed), 24)
            rpm_knee = math.min(math.max(2, (diff_knee / max_diff) * master_speed), 24)
        end
    end

    ------------------------------------------------------------------
    -- TRANSMIT CONTROL VECTORS & HARDWARE COMMANDS
    ------------------------------------------------------------------
    -- Commit synchronized speeds to the Create controllers
    speed0.setTargetSpeed(rpm_yaw)
    speed1.setTargetSpeed(-rpm_hip) -- Keeps your physical reversed hip axis fix
    speed2.setTargetSpeed(rpm_knee)

    -- Update tracking states
    last_angles.yaw  = yaw
    last_angles.hip  = hip
    last_angles.knee = knee

    -- Command position angles out to nodes
    rednet.send(hip1ID, { angle = yaw }, "joint.command")
    rednet.send(hip2ID, { angle = hip }, "joint.command")
    rednet.send(kneeID, { angle = knee }, "joint.command")

    sleep(0.05)
end