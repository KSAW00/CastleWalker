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

    -- Apply inputs based on your axes:
    -- X = Front/Back, Y = Up/Down, Z = Left/Right
    if pressed then
        pos.y = pos.y + (front - back) * cfg.joystick_travel / 15
    else
        pos.x = pos.x + (front - back) * cfg.joystick_travel / 15
        pos.z = pos.z + (right - left) * cfg.joystick_travel / 15
    end

    ------------------------------------------------------------------
    -- Inverse Kinematics
    ------------------------------------------------------------------

    -- Hip yaw: Angle on the horizontal ground plane (Z and X)
    local t1 = math.atan2(pos.z, pos.x)

    -- Distance from the hip center to the foot projection on the ground
    local r = math.sqrt(pos.x * pos.x + pos.z * pos.z)

    -- Translate origin from Hip1 to Hip2
    -- px is horizontal distance from Hip2, pz is vertical distance (Y)
    local px = r - cfg.length1
    local pz = pos.y

    -- Distance from Hip2 to target
    local d2 = px * px + pz * pz

    -- Reachability clamp
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
    -- Knee
    ------------------------------------------------------------------

    local c3 =
        (d2 - cfg.length2^2 - cfg.length3^2) /
        (2 * cfg.length2 * cfg.length3)

    c3 = math.max(-1, math.min(1, c3))

    -- Choose bend direction
    local s3 = -math.sqrt(1 - c3 * c3)

    local t3 = math.atan2(s3, c3)

    ------------------------------------------------------------------
    -- Hip pitch
    ------------------------------------------------------------------

    local k1 = cfg.length2 + cfg.length3 * c3
    local k2 = cfg.length3 * s3

    local t2 =
        math.atan2(pz, px) -
        math.atan2(k2, k1)

    ------------------------------------------------------------------
    -- Convert to degrees
    ------------------------------------------------------------------

    local yaw = math.deg(t1)
    local hip = math.deg(t2)
    local knee = math.deg(t3)

    ------------------------------------------------------------------
    -- Send commands
    ------------------------------------------------------------------

    rednet.send(hip1ID, { angle = yaw }, "joint.command")
    rednet.send(hip2ID, { angle = hip }, "joint.command")
    rednet.send(kneeID, { angle = -knee }, "joint.command")

    sleep(0.05)
end
