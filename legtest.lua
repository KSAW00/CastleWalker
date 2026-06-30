local cfg = require("config")

rednet.open(cfg.modem)

local relay = peripheral.wrap(cfg.relay)

local hip1ID = assert(rednet.lookup("joint", cfg.joints.hip1))
local hip2ID = assert(rednet.lookup("joint", cfg.joints.hip2))
local kneeID = assert(rednet.lookup("joint", cfg.joints.knee))

while true do
    local left  = relay.getAnalogInput("left")
    local right = relay.getAnalogInput("right")
    local front = relay.getAnalogInput("front")
    local back  = relay.getAnalogInput("back")
    local pressed = relay.getInput("top")

    ------------------------------------------------------------------
    -- Desired foot position
    ------------------------------------------------------------------

    local pos = {
        x = cfg.rest_pos.x,
        y = cfg.rest_pos.y,
        z = cfg.rest_pos.z,
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
    -- Inverse Kinematics (Anthropomorphic Configuration)
    ------------------------------------------------------------------

    -- 1. Solve Hip 1 (t1): Sway/Yaw rotation in the XY plane.
    -- This handles how far forward or backward the arm pitches from rest.
    local t1 = math.atan2(pos.x, pos.y)

    -- 2. Find total radial length of the arm's projection in the XY plane
    local r_xy = math.sqrt(pos.x * pos.x + pos.y * pos.y)

    -- 3. Translate the origin from Hip 1 to Hip 2 along the baseline l1
    -- For anthropomorphic joints, px is the effective vertical extension
    local px = r_xy - cfg.length1
    
    -- 4. Hip 2 and Knee handle the cross-plane deflection (Z-axis)
    local pz = pos.z 

    -- 5. Total squared distance from Hip 2 axis center to target foot
    local d2 = px * px + pz * pz

    -- Reachability clamp to protect physical motors from crashing
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

    local c3 = (d2 - cfg.length2^2 - cfg.length3^2) / (2 * cfg.length2 * cfg.length3)
    c3 = math.max(-1, math.min(1, c3))

    -- Choose bend direction (- square root keeps a standard human arm bend profile)
    local s3 = -math.sqrt(1 - c3 * c3)
    local t3 = math.atan2(s3, c3)

    ------------------------------------------------------------------
    -- Hip 2 / Shoulder Pitch (t2)
    ------------------------------------------------------------------

    local k1 = cfg.length2 + cfg.length3 * c3
    local k2 = cfg.length3 * s3

    local t2 = math.atan2(pz, px) - math.atan2(k2, k1)

    ------------------------------------------------------------------
    -- Convert to degrees
    ------------------------------------------------------------------

    local yaw = math.deg(t1)
    local hip = math.deg(t2)
    local knee = math.deg(t3)

    ------------------------------------------------------------------
    -- Send commands
    ------------------------------------------------------------------

    rednet.send(hip1ID, { angle = -90+yaw }, "joint.command")
    rednet.send(hip2ID, { angle = 90+hip }, "joint.command")
    rednet.send(kneeID, { angle = -knee }, "joint.command")

    sleep(0.05)
end
