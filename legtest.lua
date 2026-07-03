local cfg = require("config")

rednet.open(cfg.modem)

local relay = peripheral.wrap(cfg.relay)

local hip1ID = assert(rednet.lookup("joint", cfg.joints.hip1))
local hip2ID = assert(rednet.lookup("joint", cfg.joints.hip2))
local kneeID = assert(rednet.lookup("joint", cfg.joints.knee))

-- Direct local speed controller connections
local speed0 = peripheral.wrap("Create_RotationSpeedController_0")
local speed1 = peripheral.wrap("Create_RotationSpeedController_1")
local speed2 = peripheral.wrap("Create_RotationSpeedController_2")

local last_angles = { yaw = 0, hip = 0, knee = 0 }

local function getShortestDiff(target, current)
    return math.abs((target - current + 180) % 360 - 180)
end

while true do
    local left    = relay.getAnalogInput("left")
    local right   = relay.getAnalogInput("right")
    local front   = relay.getAnalogInput("front")
    local back    = relay.getAnalogInput("back")
    local pressed = relay.getInput("top")

    ------------------------------------------------------------------
    -- Desired foot position
    ------------------------------------------------------------------

    local pos     = {
        x = cfg.rest_pos.x, -- Home: 17
        y = cfg.rest_pos.y, -- Home: 18
        z = cfg.rest_pos.z, -- Home: 7
    }

    -- Axis Map: +X = Forward, +Y = Downward, +Z = Leftward
    if pressed then
        -- Front moves foot UP (-Y), Back moves foot DOWN (+Y)
        pos.y = pos.y + (back - front) * cfg.joystick_travel / 15
    else
        -- Front moves FORWARD (+X), Back moves BACKWARD (-X)
        pos.x = pos.x + (front - back) * cfg.joystick_travel / 15
        -- Right moves RIGHT (-Z), Left moves LEFT (+Z)
        pos.z = pos.z + (left - right) * cfg.joystick_travel / 15
    end

    ------------------------------------------------------------------
    -- Inverse Kinematics (Hip 1 rotates in XY plane)
    ------------------------------------------------------------------

    -- 1. Solve Hip 1 (t1): Measured from Y sweeping toward X
    local t1 = math.atan2(pos.x, pos.y)

    -- 2. Find total radial length of the arm's projection in the XY plane
    local r_xy = math.sqrt(pos.x * pos.x + pos.y * pos.y)

    -- 3. Translate the origin from Hip 1 to Hip 2 along baseline l1
    local px = r_xy - cfg.length1

    -- 4. Hip 2 and Knee handle cross-plane deflection along the Z axis
    local pz = pos.z

    -- 5. Total squared distance from Hip 2 axis center to target foot
    local d2 = px * px + pz * pz

    -- Reachability clamp to protect physical motors
    local d = math.sqrt(d2)
    local maxReach = cfg.length2 + cfg.length3 - 1e-6
    local minReach = math.abs(cfg.length2 - cfg.length3) + 1e-6

    if d > maxReach then
        px = px * maxReach / d
        pz = pz * maxReach / d
        d2 = px * px + pz * pz
    elseif d < minReach then
        px = px * minReach / d
        pz = pz * minReach / d
        d2 = px * px + pz * pz
    end

    ------------------------------------------------------------------
    -- Knee / Elbow (t3)
    ------------------------------------------------------------------

    local c3                         = (d2 - cfg.length2 ^ 2 - cfg.length3 ^ 2) / (2 * cfg.length2 * cfg.length3)
    c3                               = math.max(-1, math.min(1, c3))

    -- Choose stable human-like bend direction
    local s3                         = -math.sqrt(1 - c3 * c3)
    local t3                         = math.atan2(s3, c3)

    ------------------------------------------------------------------
    -- Hip 2 / Shoulder Pitch (t2)
    ------------------------------------------------------------------

    local k1                         = cfg.length2 + cfg.length3 * c3
    local k2                         = cfg.length3 * s3

    local t2                         = math.atan2(pz, px) - math.atan2(k2, k1)

    ------------------------------------------------------------------
    -- Convert to degrees
    ------------------------------------------------------------------
    local yaw                        = -math.deg(t1)
    local hip                        = math.deg(t2)
    local knee                       = -math.deg(t3)

    ------------------------------------------------------------------
    -- DYNAMIC VELOCITY MATCHING (RPM CALCULATION)
    ------------------------------------------------------------------
    -- 1. Calculate true absolute angular distance traveled since last tick
    local diff_yaw                   = getShortestDiff(yaw, last_angles.yaw)
    local diff_hip                   = getShortestDiff(hip, last_angles.hip)
    local diff_knee                  = getShortestDiff(knee, last_angles.knee)

    local abs_yaw                    = math.abs(diff_yaw)
    local abs_hip                    = math.abs(diff_hip)
    local abs_knee                   = math.abs(diff_knee)

    local max_error                  = math.max(abs_yaw, math.max(abs_hip, abs_knee))
    local rpm_yaw, rpm_hip, rpm_knee = 2, 2, 2

    if max_error > 0.1 then
        -- Kp controls the master speed of the bottleneck joint.
        -- Turn up to make the whole leg faster, down to make it slower.
        local Kp           = 4.0
        local master_speed = max_error * Kp

        -- Proportional Scaling: Every joint's speed is perfectly matched to the master joint
        rpm_yaw            = math.max(2, (abs_yaw / max_error) * master_speed)
        rpm_hip            = math.max(2, (abs_hip / max_error) * master_speed)
        rpm_knee           = math.max(2, (abs_knee / max_error) * master_speed)
    end
    ------------------------------------------------------------------
    -- Send commands
    ------------------------------------------------------------------

    speed0.setTargetSpeed(rpm_yaw)
    speed1.setTargetSpeed(-rpm_hip)
    speed2.setTargetSpeed(rpm_knee)

    last_angles.yaw  = last_angles.yaw  + (diff_yaw )
    last_angles.knee = last_angles.knee + (diff_knee)
    
    -- FIXED: Subtract the error delta to match the reversed physical motor tracking
    last_angles.hip  = last_angles.hip  - (diff_hip)

    rednet.send(hip1ID, { angle = yaw }, "joint.command")
    rednet.send(hip2ID, { angle = hip }, "joint.command")
    rednet.send(kneeID, { angle = knee }, "joint.command")

    sleep(0.05)
end
